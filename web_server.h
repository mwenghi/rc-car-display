#pragma once
#include <WebServer.h>
#include <SD.h>
#include <WiFi.h>
#include "web_ui.h"

WebServer webServer(80);

// Download job state (driven from loop())
struct DownloadJob {
  bool active = false;
  bool cancel = false;
  int z, x0, y0, x1, y1;
  int curX, curY;
  int done = 0;
  int total = 0;
  int errors = 0;
  char current[32] = "";
};
DownloadJob dlJob;

// Forward declarations
extern LGFX_Waveshare disp2;
extern SPIClass hspi;
extern void scanTileGrid();

// Helper: release display bus for SD access
void sdAcquire() {
  disp2.endWrite();
  digitalWrite(15, HIGH);
}

void sdRelease() {
  digitalWrite(SD_CS, HIGH);
}

// ─── Route handlers ──────────────────────────────────────────────────────────

void handleRoot() {
  webServer.sendHeader("Content-Encoding", "gzip");
  webServer.send_P(200, "text/html", (const char*)index_html_gz, index_html_gz_len);
}

void handleListTiles() {
  sdAcquire();
  File dir = SD.open("/tiles");

  // Stream chunked response — avoids building a huge String in RAM
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "application/json", "");
  webServer.sendContent("{\"tiles\":[");

  int count = 0;
  uint32_t totalBytes = 0;
  bool first = true;

  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f) {
      String name = f.name();
      if (name.endsWith(".png")) {
        String base = name;
        int lastSlash = base.lastIndexOf('/');
        if (lastSlash >= 0) base = base.substring(lastSlash + 1);
        base = base.substring(0, base.length() - 4);
        int u1 = base.indexOf('_');
        int u2 = base.indexOf('_', u1 + 1);
        if (u1 > 0 && u2 > u1) {
          int z = base.substring(0, u1).toInt();
          int x = base.substring(u1 + 1, u2).toInt();
          int y = base.substring(u2 + 1).toInt();
          uint32_t sz = f.size();
          totalBytes += sz;
          char entry[80];
          snprintf(entry, sizeof(entry), "%s{\"z\":%d,\"x\":%d,\"y\":%d,\"size\":%u}",
                   first ? "" : ",", z, x, y, (unsigned)sz);
          webServer.sendContent(entry);
          first = false;
          count++;
        }
      }
      f = dir.openNextFile();
    }
    dir.close();
  }
  sdRelease();

  char tail[64];
  snprintf(tail, sizeof(tail), "],\"total_count\":%d,\"total_bytes\":%u}", count, (unsigned)totalBytes);
  webServer.sendContent(tail);
  webServer.sendContent("");  // end chunked transfer
}

void handleDeleteTile() {
  // Extract tile name from URI: /api/tiles/15_17941_11373
  String uri = webServer.uri();
  String name = uri.substring(uri.lastIndexOf('/') + 1);
  char path[48];
  snprintf(path, sizeof(path), "/tiles/%s.png", name.c_str());

  sdAcquire();
  bool ok = SD.remove(path);
  sdRelease();
  webServer.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"not found\"}");
  if (ok) scanTileGrid();  // update grid bounds + save cache (response already sent)
}

void handleDeleteAll() {
  sdAcquire();

  File dir = SD.open("/tiles");
  int deleted = 0;
  if (dir && dir.isDirectory()) {
    String files[200];
    int n = 0;
    File f = dir.openNextFile();
    while (f && n < 200) {
      String fname = String("/tiles/") + f.name();
      // Some SD libs return full path, some just filename
      if (fname.indexOf("/tiles//tiles/") >= 0) fname = String("/tiles/") + f.name();
      files[n++] = fname;
      f = dir.openNextFile();
    }
    dir.close();
    for (int i = 0; i < n; i++) {
      if (SD.remove(files[i].c_str())) deleted++;
    }
  }

  sdRelease();
  webServer.send(200, "application/json", "{\"ok\":true,\"deleted\":" + String(deleted) + "}");
  scanTileGrid();  // update grid bounds + save cache (response already sent)
}

void handleStartDownload() {
  if (dlJob.active) {
    webServer.send(409, "application/json", "{\"ok\":false,\"error\":\"download in progress\"}");
    return;
  }

  String body = webServer.arg("plain");
  auto getVal = [&](const char* key) -> int {
    int pos = body.indexOf(String("\"") + key + "\"");
    if (pos < 0) return -1;
    pos = body.indexOf(':', pos);
    if (pos < 0) return -1;
    return body.substring(pos + 1).toInt();
  };

  dlJob.z = getVal("z");
  dlJob.x0 = getVal("x0");
  dlJob.y0 = getVal("y0");
  dlJob.x1 = getVal("x1");
  dlJob.y1 = getVal("y1");

  if (dlJob.z < 1 || dlJob.x0 < 0 || dlJob.y0 < 0) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad params\"}");
    return;
  }

  dlJob.total = (dlJob.x1 - dlJob.x0 + 1) * (dlJob.y1 - dlJob.y0 + 1);
  if (dlJob.total > 500) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"too many tiles\"}");
    return;
  }

  dlJob.curX = dlJob.x0;
  dlJob.curY = dlJob.y0;
  dlJob.done = 0;
  dlJob.errors = 0;
  dlJob.cancel = false;
  dlJob.active = true;

  Serial.printf("Web DL: z=%d x=%d..%d y=%d..%d (%d tiles)\n",
                 dlJob.z, dlJob.x0, dlJob.x1, dlJob.y0, dlJob.y1, dlJob.total);

  webServer.send(200, "application/json", "{\"ok\":true,\"total\":" + String(dlJob.total) + "}");
}

void handleDownloadStatus() {
  String json = "{\"active\":" + String(dlJob.active ? "true" : "false");
  json += ",\"done\":" + String(dlJob.done);
  json += ",\"total\":" + String(dlJob.total);
  json += ",\"errors\":" + String(dlJob.errors);
  json += ",\"current\":\"" + String(dlJob.current) + "\"}";
  webServer.send(200, "application/json", json);
}

void handleCancelDownload() {
  dlJob.cancel = true;
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleDeviceStatus() {
  String json = "{\"heap_free\":" + String(ESP.getFreeHeap());
  json += ",\"largest_block\":" + String(ESP.getMaxAllocHeap());
  json += ",\"sd_total_mb\":" + String((uint32_t)(SD.totalBytes() / 1048576));
  json += ",\"sd_used_mb\":" + String((uint32_t)(SD.usedBytes() / 1048576));
  json += ",\"wifi_rssi\":" + String(WiFi.RSSI());
  json += ",\"uptime_s\":" + String(millis() / 1000);
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleStartNav() {
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// Forward declarations for screen enums/variables
extern D1Screen d1Screen;
extern D2Screen d2Screen;
extern const char* d1ScreenNames[];
extern const char* d2ScreenNames[];

void handleGetScreens() {
  String json = "{\"d1\":{\"current\":" + String((int)d1Screen) + ",\"screens\":[";
  for (int i = 0; i < D1_SCREEN_COUNT; i++) {
    if (i) json += ",";
    json += "\"" + String(d1ScreenNames[i]) + "\"";
  }
  json += "]},\"d2\":{\"current\":" + String((int)d2Screen) + ",\"screens\":[";
  for (int i = 0; i < D2_SCREEN_COUNT; i++) {
    if (i) json += ",";
    json += "\"" + String(d2ScreenNames[i]) + "\"";
  }
  json += "]}}";
  webServer.send(200, "application/json", json);
}

void handleSetScreen() {
  String body = webServer.arg("plain");
  // Parse {"display":1,"screen":0} or {"display":2,"screen":3}
  int dispPos = body.indexOf("\"display\"");
  int scrPos = body.indexOf("\"screen\"");
  if (dispPos < 0 || scrPos < 0) {
    webServer.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  int disp = body.substring(body.indexOf(':', dispPos) + 1).toInt();
  int scr = body.substring(body.indexOf(':', scrPos) + 1).toInt();

  if (disp == 1 && scr >= 0 && scr < D1_SCREEN_COUNT) {
    d1Screen = (D1Screen)scr;
    Serial.printf("D1 screen -> %s\n", d1ScreenNames[scr]);
    saveScreenState();
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else if (disp == 2 && scr >= 0 && scr < D2_SCREEN_COUNT) {
    d2Screen = (D2Screen)scr;
    Serial.printf("D2 screen -> %s\n", d2ScreenNames[scr]);
    saveScreenState();
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid\"}");
  }
}

// ─── Route API ───────────────────────────────────────────────────────────────

extern NavRoute navRoute;
extern bool loadRoute(const char* filename);
extern void setActiveRoute(const char* filename);

// Chunked upload state
static File routeUploadFile;
static char routeUploadFilename[32] = "";
static char routeUploadName[16] = "";
static uint16_t routeUploadCount = 0;
static uint16_t routeUploadReceived = 0;

void handleRouteList() {
  sdAcquire();
  // Read active route name
  char activeName[32] = "";
  if (SD.exists("/routes/active.cfg")) {
    File af = SD.open("/routes/active.cfg", FILE_READ);
    if (af) { int l = af.readBytesUntil('\n', activeName, 31); activeName[l] = '\0'; af.close(); }
  }

  String json = "{\"routes\":[";
  File dir = SD.open("/routes");
  bool first = true;
  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f) {
      String fn = f.name();
      if (fn.endsWith(".rte")) {
        // Read header
        char magic[4]; f.read((uint8_t*)magic, 4);
        uint8_t ver = f.read(); f.read(); // flags
        uint16_t cnt; f.read((uint8_t*)&cnt, 2);
        uint32_t dist; f.read((uint8_t*)&dist, 4);
        char name[16]; f.read((uint8_t*)name, 16); name[15] = '\0';

        // Get just filename without path
        String basename = fn;
        int sl = basename.lastIndexOf('/');
        if (sl >= 0) basename = basename.substring(sl + 1);

        bool isActive = (strcmp(basename.c_str(), activeName) == 0);

        if (!first) json += ",";
        json += "{\"filename\":\"" + basename + "\",\"name\":\"" + String(name) + "\",\"points\":" + String(cnt) + ",\"distance\":" + String(dist) + ",\"active\":" + (isActive ? "true" : "false") + "}";
        first = false;
      }
      f = dir.openNextFile();
    }
    dir.close();
  }
  sdRelease();
  json += "]}";
  webServer.send(200, "application/json", json);
}

void handleRouteStart() {
  String body = webServer.arg("plain");
  // Parse name, filename, count
  int nPos = body.indexOf("\"name\"");
  int fPos = body.indexOf("\"filename\"");
  int cPos = body.indexOf("\"count\"");

  if (nPos < 0 || fPos < 0 || cPos < 0) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad params\"}");
    return;
  }

  // Extract filename string
  int fq1 = body.indexOf('"', body.indexOf(':', fPos) + 1);
  int fq2 = body.indexOf('"', fq1 + 1);
  String fname = body.substring(fq1 + 1, fq2);
  strncpy(routeUploadFilename, fname.c_str(), 31);

  // Extract name
  int nq1 = body.indexOf('"', body.indexOf(':', nPos) + 1);
  int nq2 = body.indexOf('"', nq1 + 1);
  strncpy(routeUploadName, body.substring(nq1 + 1, nq2).c_str(), 15);

  routeUploadCount = body.substring(body.indexOf(':', cPos) + 1).toInt();
  routeUploadReceived = 0;

  // Create route file with header
  char path[48];
  snprintf(path, sizeof(path), "/routes/%s", routeUploadFilename);

  sdAcquire();
  SD.mkdir("/routes");
  routeUploadFile = SD.open(path, FILE_WRITE);
  if (!routeUploadFile) {
    sdRelease();
    webServer.send(500, "application/json", "{\"ok\":false,\"error\":\"file create failed\"}");
    return;
  }

  // Write header
  routeUploadFile.write((uint8_t*)"RTE\0", 4);
  uint8_t ver = 1, flags = 0;
  routeUploadFile.write(ver);
  routeUploadFile.write(flags);
  routeUploadFile.write((uint8_t*)&routeUploadCount, 2);
  uint32_t distPlaceholder = 0;
  routeUploadFile.write((uint8_t*)&distPlaceholder, 4);
  char nameBuf[16] = {0};
  strncpy(nameBuf, routeUploadName, 15);
  routeUploadFile.write((uint8_t*)nameBuf, 16);
  sdRelease();

  Serial.printf("Route upload start: %s (%d pts)\n", routeUploadFilename, routeUploadCount);
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleRoutePoints() {
  String body = webServer.arg("plain");
  // Parse pts array: {"pts":[[lat,lon],[lat,lon],...]}
  int idx = body.indexOf('[', body.indexOf("pts"));
  if (idx < 0) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"no pts\"}");
    return;
  }

  sdAcquire();
  if (!routeUploadFile) { sdRelease(); webServer.send(400, "application/json", "{\"ok\":false}"); return; }

  // Parse point pairs
  int pos = idx;
  while (pos < (int)body.length()) {
    int ob = body.indexOf('[', pos + 1);
    if (ob < 0) break;
    int cb = body.indexOf(']', ob);
    if (cb < 0) break;
    String pair = body.substring(ob + 1, cb);
    int comma = pair.indexOf(',');
    if (comma > 0) {
      float lat = pair.substring(0, comma).toFloat();
      float lon = pair.substring(comma + 1).toFloat();
      int32_t ilat = (int32_t)(lat * 1e7f);
      int32_t ilon = (int32_t)(lon * 1e7f);
      routeUploadFile.write((uint8_t*)&ilat, 4);
      routeUploadFile.write((uint8_t*)&ilon, 4);
      routeUploadReceived++;
    }
    pos = cb + 1;
  }
  sdRelease();

  webServer.send(200, "application/json", "{\"ok\":true,\"received\":" + String(routeUploadReceived) + "}");
}

void handleRouteFinish() {
  if (!routeUploadFile) {
    webServer.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  sdAcquire();
  routeUploadFile.flush();
  routeUploadFile.close();
  sdRelease();

  // Reload to compute distance and rewrite header
  // (loadRoute reads the file, computes distances)
  Serial.printf("Route upload done: %s (%d pts received)\n", routeUploadFilename, routeUploadReceived);

  webServer.send(200, "application/json", "{\"ok\":true,\"filename\":\"" + String(routeUploadFilename) + "\",\"points\":" + String(routeUploadReceived) + "}");
}

void handleRouteActivate() {
  String body = webServer.arg("plain");
  int fq1 = body.indexOf("\"filename\"");
  if (fq1 < 0) { webServer.send(400, "application/json", "{\"ok\":false}"); return; }
  int q1 = body.indexOf('"', body.indexOf(':', fq1) + 1);
  int q2 = body.indexOf('"', q1 + 1);
  String fname = body.substring(q1 + 1, q2);

  setActiveRoute(fname.c_str());
  loadRoute(fname.c_str());
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleRouteGet() {
  // GET /api/routes/filename.rte — return as JSON with points
  String uri = webServer.uri();
  String fname = uri.substring(uri.lastIndexOf('/') + 1);
  char path[48];
  snprintf(path, sizeof(path), "/routes/%s", fname.c_str());

  sdAcquire();
  File f = SD.open(path, FILE_READ);
  if (!f) { sdRelease(); webServer.send(404, "application/json", "{\"error\":\"not found\"}"); return; }

  // Read header
  f.seek(4); f.read(); f.read(); // skip magic, ver, flags
  uint16_t cnt; f.read((uint8_t*)&cnt, 2);
  uint32_t dist; f.read((uint8_t*)&dist, 4);
  char name[16]; f.read((uint8_t*)name, 16); name[15] = '\0';

  String json = "{\"name\":\"" + String(name) + "\",\"distance\":" + String(dist) + ",\"points\":[";
  for (int i = 0; i < cnt; i++) {
    int32_t lat, lon;
    f.read((uint8_t*)&lat, 4);
    f.read((uint8_t*)&lon, 4);
    if (i > 0) json += ",";
    json += "[" + String(lat / 1e7f, 7) + "," + String(lon / 1e7f, 7) + "]";
  }
  json += "]}";
  f.close();
  sdRelease();
  webServer.send(200, "application/json", json);
}

void handleRouteDelete() {
  String uri = webServer.uri();
  String fname = uri.substring(uri.lastIndexOf('/') + 1);
  char path[48];
  snprintf(path, sizeof(path), "/routes/%s", fname.c_str());
  sdAcquire();
  bool ok = SD.remove(path);
  sdRelease();
  webServer.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// Serve route editor HTML from SD
void handleRouteEditor() {
  sdAcquire();
  File f = SD.open("/web/route.html", FILE_READ);
  if (!f) {
    sdRelease();
    webServer.send(404, "text/plain", "route.html not found on SD. Upload it to /web/route.html");
    return;
  }
  size_t fsize = f.size();
  webServer.setContentLength(fsize);
  webServer.send(200, "text/html", "");
  uint8_t buf[512];
  while (f.available()) {
    int len = f.read(buf, sizeof(buf));
    webServer.sendContent((const char*)buf, len);
  }
  f.close();
  sdRelease();
}

void handleNotFound() {
  if (webServer.method() == HTTP_DELETE && webServer.uri().startsWith("/api/tiles/")) {
    handleDeleteTile();
    return;
  }
  // Route file GET/DELETE
  if (webServer.uri().startsWith("/api/routes/") && webServer.uri().endsWith(".rte")) {
    if (webServer.method() == HTTP_DELETE) { handleRouteDelete(); return; }
    if (webServer.method() == HTTP_GET) { handleRouteGet(); return; }
  }
  webServer.send(404, "text/plain", "Not found");
}

// Setup all routes
void setupWebServer() {
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/tiles", HTTP_GET, handleListTiles);
  webServer.on("/api/tiles", HTTP_DELETE, handleDeleteAll);
  webServer.on("/api/download", HTTP_POST, handleStartDownload);
  webServer.on("/api/download/status", HTTP_GET, handleDownloadStatus);
  webServer.on("/api/download/cancel", HTTP_POST, handleCancelDownload);
  webServer.on("/api/status", HTTP_GET, handleDeviceStatus);
  webServer.on("/api/nav", HTTP_POST, handleStartNav);
  webServer.on("/api/screens", HTTP_GET, handleGetScreens);
  webServer.on("/api/screens", HTTP_POST, handleSetScreen);
  webServer.on("/api/routes", HTTP_GET, handleRouteList);
  webServer.on("/api/routes/start", HTTP_POST, handleRouteStart);
  webServer.on("/api/routes/points", HTTP_POST, handleRoutePoints);
  webServer.on("/api/routes/finish", HTTP_POST, handleRouteFinish);
  webServer.on("/api/routes/activate", HTTP_POST, handleRouteActivate);
  webServer.on("/route.html", HTTP_GET, handleRouteEditor);
  webServer.onNotFound(handleNotFound);

  webServer.begin();
  Serial.println("Web server started on port 80");
}
