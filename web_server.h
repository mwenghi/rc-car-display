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

  String json = "{\"tiles\":[";
  File dir = SD.open("/tiles");
  int count = 0;
  size_t totalBytes = 0;
  bool first = true;

  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f) {
      String name = f.name();
      if (name.endsWith(".png")) {
        String base = name;
        // Remove path prefix if present
        int lastSlash = base.lastIndexOf('/');
        if (lastSlash >= 0) base = base.substring(lastSlash + 1);
        base = base.substring(0, base.length() - 4);  // remove .png
        int u1 = base.indexOf('_');
        int u2 = base.indexOf('_', u1 + 1);
        if (u1 > 0 && u2 > u1) {
          int z = base.substring(0, u1).toInt();
          int x = base.substring(u1 + 1, u2).toInt();
          int y = base.substring(u2 + 1).toInt();
          size_t sz = f.size();
          totalBytes += sz;
          if (!first) json += ",";
          json += "{\"z\":" + String(z) + ",\"x\":" + String(x) + ",\"y\":" + String(y) + ",\"size\":" + String(sz) + "}";
          first = false;
          count++;
        }
      }
      f = dir.openNextFile();
    }
    dir.close();
  }

  sdRelease();
  json += "],\"total_count\":" + String(count) + ",\"total_bytes\":" + String(totalBytes) + "}";
  webServer.send(200, "application/json", json);
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

void handleNotFound() {
  // Handle DELETE /api/tiles/xxx — WebServer doesn't support wildcard routes natively
  if (webServer.method() == HTTP_DELETE && webServer.uri().startsWith("/api/tiles/")) {
    handleDeleteTile();
    return;
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
  webServer.onNotFound(handleNotFound);

  webServer.begin();
  Serial.println("Web server started on port 80");
}
