/*
 * GPS Navigation Demo + Web Map Manager
 * LilyGo T-Display v1.1 + Waveshare 1.47" LCD + SD Card
 *
 * D1 (built-in 240x135 landscape): dashboard gauges
 * D2 (Waveshare 172x320): OpenStreetMap tile map + compass
 * SD card on HSPI (shared with D2): tile cache
 *
 * Boot: hold BOOT button → web config mode (WiFi + web UI)
 * Normal boot → navigation mode (auto-downloads if needed)
 *
 * Waveshare: DIN=13, CLK=27, CS=15, DC=17, RST=22, BL=25
 * SD Card:   MOSI=13, SCK=27, MISO=21, CS=33
 */

#include <LovyanGFX.hpp>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// Pins
#define SD_MISO  21
#define SD_CS    33
#define BTN_BOOT  0
#define BTN_USER  35  // second built-in button (input-only GPIO)

// Steering wheel servo — pin defined here, functions after i2c_slave.h include
#define SERVO_PIN  2

// WiFi
const char* WIFI_SSID = "Chewifi";
const char* WIFI_PASS = "1236987456";

// Tile config — dynamic, scanned from SD at boot
#define TPX 256
int TILE_ZOOM = 15;
int GRID_X0 = 17939, GRID_Y0 = 11371;
int GRID_X1 = 17943, GRID_Y1 = 11375;
int GRID_W = 5, GRID_H = 5;
int MAP_PX_W = 1280, MAP_PX_H = 1280;

// tile grid functions defined after globals (need disp2 / sdAcquire)
void scanTileGrid();
void saveTileGridCache();
bool loadTileGridCache();

// Static PNG buffer in BSS — doesn't touch heap, leaves room for drawPng's zlib
static uint8_t pngBuf[40000] __attribute__((aligned(4)));

// ─── Display 1: Built-in T-Display (VSPI) ───────────────────────────────────

class LGFX_TDisplay : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX_TDisplay() {
    { auto cfg = _bus.config();
      cfg.spi_host = VSPI_HOST;  cfg.spi_mode = 0;
      cfg.freq_write = 40000000; cfg.freq_read = 16000000;
      cfg.pin_sclk = 18; cfg.pin_mosi = 19; cfg.pin_miso = -1; cfg.pin_dc = 16;
      _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg = _panel.config();
      cfg.pin_cs = 5; cfg.pin_rst = 23; cfg.pin_busy = -1;
      cfg.memory_width = 240; cfg.memory_height = 320;
      cfg.panel_width = 135;  cfg.panel_height = 240;
      cfg.offset_x = 52; cfg.offset_y = 40;
      cfg.readable = false; cfg.invert = true;
      cfg.rgb_order = false; cfg.dlen_16bit = false; cfg.bus_shared = false;
      _panel.config(cfg); }
    { auto cfg = _light.config();
      cfg.pin_bl = 4; cfg.invert = false; cfg.freq = 12000; cfg.pwm_channel = 0;
      _light.config(cfg); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

// ─── Display 2: Waveshare 1.47" (HSPI) ──────────────────────────────────────

class LGFX_Waveshare : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX_Waveshare() {
    { auto cfg = _bus.config();
      cfg.spi_host = HSPI_HOST; cfg.spi_mode = 0;
      cfg.freq_write = 40000000; cfg.freq_read = 16000000;
      cfg.pin_sclk = 27; cfg.pin_mosi = 13; cfg.pin_miso = SD_MISO; cfg.pin_dc = 17;
      _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg = _panel.config();
      cfg.pin_cs = 15; cfg.pin_rst = 22; cfg.pin_busy = -1;
      cfg.memory_width = 240; cfg.memory_height = 320;
      cfg.panel_width = 172;  cfg.panel_height = 320;
      cfg.offset_x = 34; cfg.offset_y = 0;
      cfg.readable = false; cfg.invert = true;
      cfg.rgb_order = false; cfg.dlen_16bit = false; cfg.bus_shared = true;
      _panel.config(cfg); }
    { auto cfg = _light.config();
      cfg.pin_bl = 25; cfg.invert = false; cfg.freq = 12000; cfg.pwm_channel = 1;
      _light.config(cfg); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

// ─── Globals ─────────────────────────────────────────────────────────────────

LGFX_TDisplay disp1;
LGFX_Waveshare disp2;
LGFX_Sprite sprite1(&disp1);
SPIClass hspi(HSPI);

enum AppState { STATE_INIT, STATE_NAVIGATE, STATE_WEBSERVER };
AppState appState = STATE_INIT;
bool navInitDone = false;

// ─── Screen system ───────────────────────────────────────────────────────────

// D1 screens (main display, 240x135 landscape)
enum D1Screen {
  D1_DASHBOARD = 0,   // speedometer gauge + throttle (implemented)
  D1_NAVIGATION,      // placeholder
  D1_MEDIA,           // placeholder
  D1_IMU,             // IMU tilt visualization
  D1_ABOUT,           // about/credits
  D1_SCREEN_COUNT
};

// D2 screens (Waveshare, 172x320)
enum D2Screen {
  D2_NAVIGATION = 0,  // map top + nav arrows bottom
  D2_MAP,             // full OSM tile map
  D2_BATTERY,         // battery indicator
  D2_SIGNAL,          // signal + battery combo
  D2_MEDIA,           // media player
  D2_DEBUG,           // placeholder
  D2_SCREEN_COUNT
};

D1Screen d1Screen = D1_DASHBOARD;
D2Screen d2Screen = D2_NAVIGATION;

// Previous screen tracking (to clear on switch)
D1Screen d1PrevScreen = D1_DASHBOARD;
D2Screen d2PrevScreen = D2_NAVIGATION;

const char* d1ScreenNames[] = {"Dashboard", "Navigation", "Media", "IMU", "About"};
const char* d2ScreenNames[] = {"Navigation", "Map", "Battery", "Signal", "Media", "Debug"};

// Forward declarations for sdAcquire/sdRelease (defined in web_server.h)
void sdAcquire();
void sdRelease();

// Save/load screen state to SD
void saveScreenState() {
  sdAcquire();
  File f = SD.open("/screens.cfg", FILE_WRITE);
  if (f) {
    f.printf("%d %d\n", (int)d1Screen, (int)d2Screen);
    f.close();
  }
  sdRelease();
}

void loadScreenState() {
  sdAcquire();
  if (SD.exists("/screens.cfg")) {
    File f = SD.open("/screens.cfg", FILE_READ);
    if (f) {
      int d1 = f.parseInt();
      int d2 = f.parseInt();
      f.close();
      if (d1 >= 0 && d1 < D1_SCREEN_COUNT) d1Screen = (D1Screen)d1;
      if (d2 >= 0 && d2 < D2_SCREEN_COUNT) d2Screen = (D2Screen)d2;
      d1PrevScreen = (D1Screen)(((int)d1Screen + 1) % D1_SCREEN_COUNT);
      d2PrevScreen = (D2Screen)(((int)d2Screen + 1) % D2_SCREEN_COUNT);
      Serial.printf("Loaded screens: D1=%s D2=%s\n", d1ScreenNames[d1Screen], d2ScreenNames[d2Screen]);
    }
  }
  sdRelease();
}

// Button: short press = cycle D1, long press = cycle D2
unsigned long btnPressStart = 0;
bool btnWasPressed = false;
bool longPressHandled = false;
#define LONG_PRESS_MS 800

void checkScreenButton() {
  bool pressed = (digitalRead(BTN_USER) == LOW);

  if (pressed && !btnWasPressed) {
    btnPressStart = millis();
    longPressHandled = false;
  } else if (pressed && btnWasPressed && !longPressHandled) {
    if (millis() - btnPressStart >= LONG_PRESS_MS) {
      d2Screen = (D2Screen)(((int)d2Screen + 1) % D2_SCREEN_COUNT);
      Serial.printf("D2 -> %s\n", d2ScreenNames[d2Screen]);
      saveScreenState();
      longPressHandled = true;
    }
  } else if (!pressed && btnWasPressed) {
    if (!longPressHandled && (millis() - btnPressStart < LONG_PRESS_MS)) {
      d1Screen = (D1Screen)(((int)d1Screen + 1) % D1_SCREEN_COUNT);
      Serial.printf("D1 -> %s\n", d1ScreenNames[d1Screen]);
      saveScreenState();
    }
  }

  btnWasPressed = pressed;
}

#include "i2c_slave.h"
#include "navigation.h"
#include "web_server.h"

// ─── Steering wheel servo (continuous rotation) ──────────────────────────────
// CR servo: 1500µs = stop, <1500 = CW, >1500 = CCW
// steer_pos 0-255 (128=center) → target ±180° (1 turn lock-to-lock)
// Open-loop P-controller with full-speed return to center

#define SERVO_SPEED_DPS  200.0f   // degrees/sec at ±500µs — calibrate this

static float servoAngle = 0.0f;  // estimated current angle (degrees)

void servoSetup() {
  ledcAttach(SERVO_PIN, 50, 16);
  ledcWrite(SERVO_PIN, 1500u * 65535u / 20000u);  // start stopped
}

void servoUpdate() {
  static unsigned long prevMs = 0;
  unsigned long now = millis();
  float dt = constrain((now - prevMs) * 0.001f, 0.0f, 0.1f);
  prevMs = now;

  float target = (liveData.steer_pos - 128.0f) / 128.0f * 180.0f;
  float err = target - servoAngle;

  int us;
  if (fabsf(err) < 2.0f) {
    us = 1500;                                           // dead zone — stop
    // Re-anchor: when stopped near center, reset estimate to kill drift
    if (fabsf(target) < 5.0f) servoAngle = 0;
  } else {
    bool returning = fabsf(target) < fabsf(servoAngle);  // moving toward center?
    int dev;
    if (returning) {
      dev = (err > 0) ? 500 : -500;                      // full speed to center
    } else {
      dev = (int)constrain(err * 3.0f, -500.0f, 500.0f); // proportional to turn
    }
    us = constrain(1500 + dev, 1000, 2000);
    servoAngle += (dev / 500.0f) * SERVO_SPEED_DPS * dt;
    servoAngle = constrain(servoAngle, -180.0f, 180.0f);
  }
  ledcWrite(SERVO_PIN, (uint32_t)us * 65535u / 20000u);
}

// Simulation
float simX, simY, simHeading, simSpeed;
float simKmh = 0;           // smoothed display speed
float simThrottle = 0;      // 0-100%
float targetThrottle = 50;  // random target

// WiFi/Web state
bool wifiActive = false;
bool webServerStarted = false;
char wifiIP[20] = "";

// ─── Helpers ─────────────────────────────────────────────────────────────────

void getTilePath(char* buf, int tz, int tx, int ty) {
  snprintf(buf, 40, "/tiles/%d_%d_%d.png", tz, tx, ty);
}

void showD1(const char* l1, const char* l2 = nullptr, const char* l3 = nullptr,
            const char* l4 = nullptr, const char* l5 = nullptr, const char* l6 = nullptr) {
  sprite1.fillScreen(TFT_BLACK);
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(1);
  const char* lines[] = {l1, l2, l3, l4, l5, l6};
  int y = 20;
  for (int i = 0; i < 6; i++) {
    if (lines[i]) { sprite1.setCursor(8, y); sprite1.print(lines[i]); }
    y += 22;
  }
  sprite1.pushSprite(0, 0);
}

void showD2(const char* msg) {
  disp2.fillScreen(TFT_BLACK);
  disp2.setTextColor(TFT_WHITE);
  disp2.setTextSize(2);
  disp2.setCursor(10, 140);
  disp2.print(msg);
}

// ─── Tile download ───────────────────────────────────────────────────────────

bool allTilesExist() {
  // Need to access SD — release display bus
  disp2.endWrite();
  digitalWrite(15, HIGH);

  for (int gy = 0; gy < GRID_H; gy++)
    for (int gx = 0; gx < GRID_W; gx++) {
      char path[40];
      getTilePath(path, TILE_ZOOM, GRID_X0 + gx, GRID_Y0 + gy);
      if (!SD.exists(path)) return false;
    }
  return true;
}

bool downloadTile(int tileX, int tileY) {
  char url[128];
  snprintf(url, sizeof(url), "https://tile.openstreetmap.org/%d/%d/%d.png",
           TILE_ZOOM, tileX, tileY);

  WiFiClientSecure *client = new WiFiClientSecure;
  client->setInsecure();

  HTTPClient http;
  http.begin(*client, url);
  http.addHeader("User-Agent", "ESP32-NavDemo/1.0 (hobby project)");
  http.setTimeout(10000);
  int code = http.GET();

  if (code != 200) {
    Serial.printf("HTTP %d for %d/%d\n", code, tileX, tileY);
    http.end();
    delete client;
    return false;
  }

  char path[40];
  getTilePath(path, TILE_ZOOM, tileX, tileY);

  // Release display bus for SD write
  disp2.endWrite();
  digitalWrite(15, HIGH);

  File f = SD.open(path, FILE_WRITE);
  if (!f) { http.end(); delete client; return false; }

  int total = http.getSize();
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[512];
  int written = 0;
  while (http.connected() && (written < total || total == -1)) {
    int avail = stream->available();
    if (avail > 0) {
      int len = stream->readBytes(buf, min((int)sizeof(buf), avail));
      f.write(buf, len);
      written += len;
    } else if (total > 0 && written >= total) {
      break;
    } else {
      delay(1);
    }
  }
  f.close();
  http.end();
  delete client;

  Serial.printf("Tile %d/%d: %d bytes\n", tileX, tileY, written);
  return written > 0;
}

// ─── Coordinate conversion ──────────────────────────────────────────────────

void pixelToLatLon(float px, float py, float* lat, float* lon) {
  float tx = GRID_X0 + px / TPX;
  float ty = GRID_Y0 + py / TPX;
  int n = 1 << TILE_ZOOM;
  *lon = tx / n * 360.0f - 180.0f;
  *lat = atanf(sinhf(PI * (1.0f - 2.0f * ty / n))) * 180.0f / PI;
}

// ─── Simulation ──────────────────────────────────────────────────────────────

void updateSim() {
  simHeading += (random(100) - 50) * 0.003f;
  simSpeed = 1.5f + sinf(millis() * 0.0008f) * 1.0f;

  float nx = simX + simSpeed * sinf(simHeading);
  float ny = simY - simSpeed * cosf(simHeading);

  float margin = TPX;
  if (nx < margin || nx > MAP_PX_W - margin) {
    simHeading = -simHeading;
    nx = simX + simSpeed * sinf(simHeading);
  }
  if (ny < margin || ny > MAP_PX_H - margin) {
    simHeading = PI - simHeading;
    ny = simY - simSpeed * cosf(simHeading);
  }

  simX = constrain(nx, margin, (float)(MAP_PX_W - margin));
  simY = constrain(ny, margin, (float)(MAP_PX_H - margin));

  // Smooth speed for gauge (simulate realistic driving)
  float rawKmh = simSpeed * 4.8f * 2.5f * 3.6f;
  simKmh += (rawKmh - simKmh) * 0.1f;  // low-pass filter

  // Throttle: random target changes, smooth approach
  if (random(100) < 5) targetThrottle = random(100);
  simThrottle += (targetThrottle - simThrottle) * 0.08f;
}

// ─── Tile grid cache ─────────────────────────────────────────────────────────

#define TILE_CACHE_PATH "/tiles/grid.cfg"

void saveTileGridCache() {
  sdAcquire();
  SD.mkdir("/tiles");
  File f = SD.open(TILE_CACHE_PATH, FILE_WRITE);
  if (!f) { sdRelease(); Serial.println("Cache save failed"); return; }
  f.printf("%d %d %d %d %d\n", TILE_ZOOM, GRID_X0, GRID_Y0, GRID_X1, GRID_Y1);
  f.close();
  sdRelease();
  Serial.printf("Tile grid cache saved: z=%d x=%d..%d y=%d..%d\n",
                 TILE_ZOOM, GRID_X0, GRID_X1, GRID_Y0, GRID_Y1);
}

bool loadTileGridCache() {
  sdAcquire();
  if (!SD.exists(TILE_CACHE_PATH)) { sdRelease(); return false; }
  File f = SD.open(TILE_CACHE_PATH, FILE_READ);
  if (!f) { sdRelease(); return false; }
  char buf[48];
  int len = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';
  f.close();
  sdRelease();

  int z, x0, y0, x1, y1;
  if (sscanf(buf, "%d %d %d %d %d", &z, &x0, &y0, &x1, &y1) != 5
      || z < 1 || z > 19 || x0 < 0 || y0 < 0 || x1 < x0 || y1 < y0) {
    Serial.println("Tile grid cache invalid");
    return false;
  }
  TILE_ZOOM = z;
  GRID_X0 = x0; GRID_Y0 = y0;
  GRID_X1 = x1; GRID_Y1 = y1;
  GRID_W = x1 - x0 + 1;
  GRID_H = y1 - y0 + 1;
  MAP_PX_W = GRID_W * TPX;
  MAP_PX_H = GRID_H * TPX;
  Serial.printf("Tile grid from cache: z=%d x=%d..%d y=%d..%d (%dx%d)\n",
                 TILE_ZOOM, GRID_X0, GRID_X1, GRID_Y0, GRID_Y1, GRID_W, GRID_H);
  return true;
}

// ─── Tile grid scanner ───────────────────────────────────────────────────────

void scanTileGrid() {
  Serial.println("Scanning tiles...");
  disp2.endWrite(); digitalWrite(15, HIGH);
  File dir = SD.open("/tiles");
  if (!dir || !dir.isDirectory()) { digitalWrite(SD_CS, HIGH); return; }

  int bestZ = -1, bestCount = 0;
  int zoomCounts[20] = {0};

  int fileCount = 0;
  File f = dir.openNextFile();
  while (f) {
    String fn = f.name();
    if (fileCount < 3) Serial.printf("  file: '%s'\n", fn.c_str());
    fileCount++;
    if (fn.endsWith(".png")) {
      String base = fn;
      int sl = base.lastIndexOf('/');
      if (sl >= 0) base = base.substring(sl + 1);
      base = base.substring(0, base.length() - 4);
      int u1 = base.indexOf('_');
      int u2 = base.indexOf('_', u1 + 1);
      if (u1 > 0 && u2 > u1) {
        int z = base.substring(0, u1).toInt();
        if (z >= 1 && z <= 19) zoomCounts[z]++;
      }
    }
    f = dir.openNextFile();
  }
  dir.close();
  Serial.printf("  scanned %d files\n", fileCount);

  for (int z = 1; z <= 19; z++) {
    if (zoomCounts[z] > 0) Serial.printf("  z%d: %d tiles\n", z, zoomCounts[z]);
    if (zoomCounts[z] > bestCount) { bestCount = zoomCounts[z]; bestZ = z; }
  }
  if (bestZ < 0 || bestCount == 0) {
    Serial.println("No tiles found on SD");
    digitalWrite(SD_CS, HIGH);
    return;
  }

  int minX = 999999, maxX = 0, minY = 999999, maxY = 0;
  dir = SD.open("/tiles");
  f = dir.openNextFile();
  while (f) {
    String fn = f.name();
    if (fn.endsWith(".png")) {
      String base = fn;
      int sl = base.lastIndexOf('/');
      if (sl >= 0) base = base.substring(sl + 1);
      base = base.substring(0, base.length() - 4);
      int u1 = base.indexOf('_');
      int u2 = base.indexOf('_', u1 + 1);
      if (u1 > 0 && u2 > u1) {
        int z = base.substring(0, u1).toInt();
        int x = base.substring(u1 + 1, u2).toInt();
        int y = base.substring(u2 + 1).toInt();
        if (z == bestZ) {
          if (x < minX) minX = x; if (x > maxX) maxX = x;
          if (y < minY) minY = y; if (y > maxY) maxY = y;
        }
      }
    }
    f = dir.openNextFile();
  }
  dir.close();
  digitalWrite(SD_CS, HIGH);

  TILE_ZOOM = bestZ;
  GRID_X0 = minX; GRID_Y0 = minY;
  GRID_X1 = maxX; GRID_Y1 = maxY;
  GRID_W = maxX - minX + 1;
  GRID_H = maxY - minY + 1;
  MAP_PX_W = GRID_W * TPX;
  MAP_PX_H = GRID_H * TPX;

  Serial.printf("Tile grid: z=%d x=%d..%d y=%d..%d (%dx%d = %d tiles)\n",
                 TILE_ZOOM, GRID_X0, GRID_X1, GRID_Y0, GRID_Y1, GRID_W, GRID_H, bestCount);
  saveTileGridCache();
}

// ─── Tile drawing helper ─────────────────────────────────────────────────────

void drawTile(int gx, int gy, int vpX, int vpY) {
  char path[40];
  getTilePath(path, TILE_ZOOM, GRID_X0 + gx, GRID_Y0 + gy);

  disp2.endWrite();
  digitalWrite(15, HIGH);

  File f = SD.open(path, FILE_READ);
  if (!f) return;
  size_t fsize = f.size();
  if (fsize < 100 || fsize > sizeof(pngBuf)) { f.close(); return; }

  size_t len = f.read(pngBuf, fsize);
  f.close();
  digitalWrite(SD_CS, HIGH);

  disp2.startWrite();
  disp2.drawPng(pngBuf, len, gx * TPX - vpX, gy * TPX - vpY);
  disp2.endWrite();
}

// ─── Map rendering (D2) ─────────────────────────────────────────────────────

void renderMap() {
  // Viewport centered on position
  int vpX = (int)simX - 86;   // 172/2
  int vpY = (int)simY - 160;  // 320/2
  vpX = constrain(vpX, 0, MAP_PX_W - 172);
  vpY = constrain(vpY, 0, MAP_PX_H - 320);

  // Visible tile range
  int tx0 = vpX / TPX, tx1 = (vpX + 171) / TPX;
  int ty0 = vpY / TPX, ty1 = (vpY + 319) / TPX;

  // Determine which grid tile contains the marker (render it last)
  int markerGx = constrain((int)simX / TPX, tx0, tx1);
  int markerGy = constrain((int)simY / TPX, ty0, ty1);

  // Pass 1: draw all tiles EXCEPT the one containing the marker
  for (int ty = ty0; ty <= ty1; ty++) {
    for (int tx = tx0; tx <= tx1; tx++) {
      if (tx == markerGx && ty == markerGy) continue;
      if (tx < 0 || tx >= GRID_W || ty < 0 || ty >= GRID_H) continue;
      drawTile(tx, ty, vpX, vpY);
    }
  }

  // Pass 2: draw marker tile last, then marker immediately after
  if (markerGx >= 0 && markerGx < GRID_W && markerGy >= 0 && markerGy < GRID_H) {
    drawTile(markerGx, markerGy, vpX, vpY);
  }

  // Position marker — drawn right after its tile, minimal blink
  disp2.startWrite();

  int mx = (int)simX - vpX;
  int my = (int)simY - vpY;

  // Heading arrow
  float arrowLen = 14;
  int ax = mx + (int)(arrowLen * sinf(simHeading));
  int ay = my - (int)(arrowLen * cosf(simHeading));
  for (int d = -1; d <= 1; d++) {
    disp2.drawLine(mx + d, my, ax + d, ay, TFT_RED);
    disp2.drawLine(mx, my + d, ax, ay + d, TFT_RED);
  }

  // Arrowhead
  float ha = 0.4f;
  disp2.fillTriangle(ax, ay,
    ax + (int)(6 * sinf(simHeading + PI - ha)), ay - (int)(6 * cosf(simHeading + PI - ha)),
    ax + (int)(6 * sinf(simHeading + PI + ha)), ay - (int)(6 * cosf(simHeading + PI + ha)),
    TFT_RED);

  // Position dot
  if (navRoute.active && navRoute.waitingForGPS) {
    uint16_t col = ((millis() / 400) % 2) ? TFT_CYAN : TFT_GREEN;
    disp2.fillCircle(mx, my, 7, col);
    disp2.fillCircle(mx, my, 3, TFT_WHITE);
    disp2.drawFastVLine(mx, my - 16, 16, TFT_WHITE);
    disp2.fillRect(mx + 1, my - 16, 10, 6, TFT_GREEN);
  } else if (menu.previewMode && (millis() / 350) % 2) {
    disp2.fillCircle(mx, my, 7, TFT_RED);
    disp2.fillCircle(mx, my, 3, TFT_WHITE);
  } else {
    disp2.fillCircle(mx, my, 6, TFT_BLUE);
    disp2.fillCircle(mx, my, 3, TFT_WHITE);
  }

  disp2.endWrite();
}

// ─── D2 Navigation screen (map top half + nav info bottom) ──────────────────

static bool navBottomInit = false;

void renderD2Navigation() {
  // Top half: mini map (0..159)
  int vpX = (int)simX - 86;
  int vpY = (int)simY - 80;  // center vertically in top 160px
  vpX = constrain(vpX, 0, MAP_PX_W - 172);
  vpY = constrain(vpY, 0, MAP_PX_H - 160);

  int tx0 = vpX / TPX, tx1 = (vpX + 171) / TPX;
  int ty0 = vpY / TPX, ty1 = (vpY + 159) / TPX;

  // Draw tiles clipped to top half
  for (int ty = ty0; ty <= ty1; ty++) {
    for (int tx = tx0; tx <= tx1; tx++) {
      if (tx < 0 || tx >= GRID_W || ty < 0 || ty >= GRID_H) continue;
      char path[40];
      getTilePath(path, TILE_ZOOM, GRID_X0 + tx, GRID_Y0 + ty);
      disp2.endWrite();
      digitalWrite(15, HIGH);
      File f = SD.open(path, FILE_READ);
      if (!f) continue;
      size_t fsize = f.size();
      if (fsize < 100 || fsize > sizeof(pngBuf)) { f.close(); continue; }
      f.read(pngBuf, fsize);
      f.close();
      digitalWrite(SD_CS, HIGH);
      disp2.startWrite();
      disp2.setClipRect(0, 0, 172, 160);
      disp2.drawPng(pngBuf, fsize, tx * TPX - vpX, ty * TPX - vpY);
      disp2.clearClipRect();
      disp2.endWrite();
    }
  }

  // Position marker on mini map
  disp2.startWrite();
  int mx = (int)simX - vpX;
  int my = (int)simY - vpY;
  if (my >= 0 && my < 160) {
    if (navRoute.active && navRoute.waitingForGPS) {
      // Pulsing start-point marker (flag-like)
      uint16_t col = ((millis() / 400) % 2) ? TFT_CYAN : TFT_GREEN;
      disp2.fillCircle(mx, my, 6, col);
      disp2.fillCircle(mx, my, 3, TFT_WHITE);
      // Small flag pole + flag
      disp2.drawFastVLine(mx, my - 14, 14, TFT_WHITE);
      disp2.fillRect(mx + 1, my - 14, 8, 5, TFT_GREEN);
    } else if (menu.previewMode && (millis() / 350) % 2) {
      disp2.fillCircle(mx, my, 5, TFT_RED);
      disp2.fillCircle(mx, my, 2, TFT_WHITE);
    } else {
      disp2.fillCircle(mx, my, 4, TFT_BLUE);
      disp2.fillCircle(mx, my, 2, TFT_WHITE);
    }
  }

  // ── Bottom half: navigation panel (160..319) ──
  // Partial redraws: clear only changed areas to avoid full-screen blink

  static int8_t prevTurnDir = -99;
  static bool prevTurnUrgent = false;
  static bool prevNavWaiting = false;
  static bool prevPreviewMode = false;

  // Detect state transitions — force full redraw
  bool curWaiting = navRoute.active && navRoute.waitingForGPS;
  if (curWaiting != prevNavWaiting || menu.previewMode != prevPreviewMode) {
    navBottomInit = false;
    prevNavWaiting = curWaiting;
    prevPreviewMode = menu.previewMode;
  }

  // One-time full clear on first frame or screen/state change
  if (!navBottomInit) {
    disp2.fillRect(0, 160, 172, 160, TFT_BLACK);
    disp2.drawFastHLine(0, 160, 172, 0x4208);
    navBottomInit = true;
    prevTurnDir = -99;
  }

  float hdgDeg = liveData.heading_deg;
  float kmh = liveData.speed_kmh;
  int8_t turnDir = navRoute.active ? navRoute.nextTurnDir : 0;
  float turnDist = navRoute.active ? navRoute.distToNextTurnM : 9999;
  bool turnUrgent = (turnDist < 20);

  // Redraw arrow only when direction or urgency changes
  int arrowCx = 86, arrowCy = 210;
  int8_t effectiveDir = (turnDir == 0 || turnDist > 50) ? 0 : turnDir;

  // Waiting for GPS overrides the arrow area
  static bool prevWaiting = false;
  if (navRoute.active && navRoute.waitingForGPS) {
    disp2.fillRect(0, 175, 172, 90, TFT_BLACK);

    // Pulsing ring
    float pulse = (sinf(millis() * 0.004f) + 1.0f) * 0.5f;
    uint8_t br = (uint8_t)(80 + 175 * pulse);
    uint16_t ringCol = disp2.color565(0, br, br);
    disp2.drawCircle(arrowCx, arrowCy, 32, ringCol);
    disp2.drawCircle(arrowCx, arrowCy, 33, ringCol);

    // Arrow pointing toward start
    float bearRad = navRoute.startBearingDeg * PI / 180.0f;
    int tipX = arrowCx + (int)(26.0f * sinf(bearRad));
    int tipY = arrowCy - (int)(26.0f * cosf(bearRad));
    int bx1 = arrowCx + (int)(10.0f * sinf(bearRad + 2.5f));
    int by1 = arrowCy - (int)(10.0f * cosf(bearRad + 2.5f));
    int bx2 = arrowCx + (int)(10.0f * sinf(bearRad - 2.5f));
    int by2 = arrowCy - (int)(10.0f * cosf(bearRad - 2.5f));
    uint16_t arrowCol = ((millis() / 500) % 2) ? TFT_CYAN : disp2.color565(0, 100, 100);
    disp2.fillTriangle(tipX, tipY, bx1, by1, bx2, by2, arrowCol);

    // "GO TO START" label
    disp2.setTextSize(2);
    disp2.setTextColor(((millis() / 600) % 2) ? TFT_WHITE : 0x6B6D);
    disp2.setCursor(16, 252);
    disp2.print("GO TO START");

    // Distance
    disp2.setTextSize(1);
    disp2.setTextColor(TFT_CYAN);
    disp2.setCursor(50, 272);
    if (navRoute.startDistM < 1000)
      disp2.printf("%.0fm away", navRoute.startDistM);
    else
      disp2.printf("%.1fkm away", navRoute.startDistM / 1000.0f);

    prevTurnDir = -99;  // force redraw when nav starts
    prevWaiting = true;
  } else {
    if (prevWaiting) {
      prevWaiting = false;
      prevTurnDir = -99;  // force redraw after leaving waiting state
    }
    if (effectiveDir != prevTurnDir || turnUrgent != prevTurnUrgent) {
      disp2.fillRect(0, 175, 172, 90, TFT_BLACK);  // clear arrow + label area
      if (effectiveDir == 0) {
        uint16_t c = TFT_GREEN;
        disp2.fillTriangle(arrowCx, arrowCy - 30, arrowCx - 15, arrowCy, arrowCx + 15, arrowCy, c);
        disp2.fillRect(arrowCx - 6, arrowCy, 12, 20, c);
        disp2.setTextColor(c); disp2.setCursor(48, 250); disp2.setTextSize(2); disp2.print("AHEAD");
      } else if (effectiveDir > 0) {
        uint16_t c = turnUrgent ? TFT_RED : TFT_YELLOW;
        disp2.fillTriangle(arrowCx + 30, arrowCy, arrowCx, arrowCy - 15, arrowCx, arrowCy + 15, c);
        disp2.fillRect(arrowCx - 20, arrowCy - 6, 20, 12, c);
        disp2.setTextColor(c); disp2.setCursor(42, 250); disp2.setTextSize(2); disp2.print("RIGHT");
      } else {
        uint16_t c = turnUrgent ? TFT_RED : TFT_YELLOW;
        disp2.fillTriangle(arrowCx - 30, arrowCy, arrowCx, arrowCy - 15, arrowCx, arrowCy + 15, c);
        disp2.fillRect(arrowCx, arrowCy - 6, 20, 12, c);
        disp2.setTextColor(c); disp2.setCursor(48, 250); disp2.setTextSize(2); disp2.print("LEFT");
      }
      prevTurnDir = effectiveDir;
      prevTurnUrgent = turnUrgent;
    }
  }

  // Text rows — clear and redraw each row (narrow strips, minimal flash)
  disp2.setTextSize(1);

  // Row 1: route remaining + speed
  disp2.fillRect(0, 168, 172, 10, TFT_BLACK);
  disp2.setTextColor(TFT_WHITE); disp2.setCursor(8, 170);
  if (navRoute.active && navRoute.waitingForGPS) disp2.printf("WAIT: %s", navRoute.name);
  else if (navRoute.active) disp2.printf("REM: %.0fm", navRoute.remainingDistM);
  else disp2.print("No route");
  disp2.setCursor(100, 170); disp2.setTextColor(TFT_GREEN); disp2.printf("%.0f", kmh);
  disp2.setTextColor(0xC618); disp2.print(" km/h");

  // Row 2: turn distance + heading
  disp2.fillRect(0, 182, 172, 10, TFT_BLACK);
  disp2.setCursor(8, 184);
  if (navRoute.active && turnDir != 0) { disp2.setTextColor(TFT_WHITE); disp2.printf("TURN: %.0fm", turnDist); }
  else { disp2.setTextColor(0x8410); disp2.print("No turn ahead"); }
  disp2.setCursor(100, 184); disp2.setTextColor(0x8C71); disp2.printf("HDG %.0f%c", hdgDeg, (char)247);

  // Bottom row: ETA + coordinates
  disp2.fillRect(0, 298, 172, 10, TFT_BLACK);
  disp2.setCursor(8, 300); disp2.setTextColor(0xC618);
  int eta = 15 + (int)(sinf(millis() * 0.0001f) * 5);
  disp2.printf("ETA: %d min", eta);
  float lat, lon;
  pixelToLatLon(simX, simY, &lat, &lon);
  disp2.setCursor(80, 300); disp2.setTextColor(0x8C71); disp2.printf("%.4fN", lat);

  disp2.endWrite();
}

// ─── Nav info rendering (D1) ────────────────────────────────────────────────

void renderNavInfo() {
  // Landscape: 240 wide x 135 tall
  // Left side: speedometer gauge, Right side: throttle + RPM + info
  sprite1.fillScreen(TFT_BLACK);

  // ═══════════════════════════════════════════════════════════════════════════
  // SPEEDOMETER GAUGE (left half) — semicircular arc, 0-200 km/h
  // ═══════════════════════════════════════════════════════════════════════════
  int gx = 68, gy = 72, gr = 56;
  float startAng = -225.0f;
  float endAng = 45.0f;
  float sweepDeg = endAng - startAng;  // 270°
  float maxSpeed = 200.0f;

  // Gauge arc background
  for (int a = (int)startAng; a <= (int)endAng; a += 2) {
    float rad = a * PI / 180.0f;
    int ox = gx + (int)(gr * cosf(rad));
    int oy = gy + (int)(gr * sinf(rad));
    int ix = gx + (int)((gr - 4) * cosf(rad));
    int iy = gy + (int)((gr - 4) * sinf(rad));
    sprite1.drawLine(ix, iy, ox, oy, 0x2104);
  }

  // Colored outer arc (green → yellow → red)
  for (int a = (int)startAng; a <= (int)endAng; a++) {
    float frac = (float)(a - startAng) / sweepDeg;
    uint16_t col;
    if (frac < 0.5f) col = sprite1.color565(0, 160, 0);
    else if (frac < 0.75f) col = sprite1.color565(180, 180, 0);
    else col = sprite1.color565(200, 0, 0);
    float rad = a * PI / 180.0f;
    int ox = gx + (int)((gr + 3) * cosf(rad));
    int oy = gy + (int)((gr + 3) * sinf(rad));
    int ix = gx + (int)((gr + 1) * cosf(rad));
    int iy = gy + (int)((gr + 1) * sinf(rad));
    sprite1.drawLine(ix, iy, ox, oy, col);
  }

  // Major ticks + labels (every 40 km/h for readability)
  for (int spd = 0; spd <= 200; spd += 40) {
    float frac = spd / maxSpeed;
    float ang = (startAng + frac * sweepDeg) * PI / 180.0f;
    int ox = gx + (int)(gr * cosf(ang));
    int oy = gy + (int)(gr * sinf(ang));
    int ix = gx + (int)((gr - 9) * cosf(ang));
    int iy = gy + (int)((gr - 9) * sinf(ang));
    sprite1.drawLine(ix, iy, ox, oy, TFT_WHITE);
    int lx = gx + (int)((gr - 17) * cosf(ang)) - 8;
    int ly = gy + (int)((gr - 17) * sinf(ang)) - 4;
    sprite1.setTextColor(0xC618);
    sprite1.setTextSize(1);
    sprite1.setCursor(lx, ly);
    sprite1.printf("%d", spd);
  }

  // Minor ticks (every 20)
  for (int spd = 20; spd < 200; spd += 40) {
    float frac = spd / maxSpeed;
    float ang = (startAng + frac * sweepDeg) * PI / 180.0f;
    int ox = gx + (int)(gr * cosf(ang));
    int oy = gy + (int)(gr * sinf(ang));
    int ix = gx + (int)((gr - 5) * cosf(ang));
    int iy = gy + (int)((gr - 5) * sinf(ang));
    sprite1.drawLine(ix, iy, ox, oy, 0x6B6D);
  }

  // Needle
  float clampedKmh = constrain(liveData.speed_kmh, 0.0f, maxSpeed);
  float needleFrac = clampedKmh / maxSpeed;
  float needleAng = (startAng + needleFrac * sweepDeg) * PI / 180.0f;
  int nx = gx + (int)((gr - 12) * cosf(needleAng));
  int ny = gy + (int)((gr - 12) * sinf(needleAng));
  for (int d = -1; d <= 1; d++) {
    sprite1.drawLine(gx + d, gy, nx + d, ny, TFT_RED);
    sprite1.drawLine(gx, gy + d, nx, ny + d, TFT_RED);
  }
  sprite1.fillCircle(gx, gy, 5, TFT_DARKGREY);
  sprite1.fillCircle(gx, gy, 3, TFT_RED);

  // Digital speed — large, centered under gauge
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(2);
  char spdTxt[8];
  snprintf(spdTxt, sizeof(spdTxt), "%3d", (int)clampedKmh);
  sprite1.setCursor(38, gy + 12);
  sprite1.print(spdTxt);
  sprite1.setTextSize(1);
  sprite1.setCursor(86, gy + 18);
  sprite1.print("km/h");

  // ═══════════════════════════════════════════════════════════════════════════
  // RIGHT SIDE — throttle bar, RPM bar, coordinates
  // ═══════════════════════════════════════════════════════════════════════════
  int rx = 140;  // right panel x start

  // THROTTLE
  sprite1.setTextColor(0xC618);
  sprite1.setTextSize(1);
  sprite1.setCursor(rx, 4);
  sprite1.print("THROTTLE");
  sprite1.setCursor(rx + 60, 4);
  sprite1.printf("%3d%%", (int)liveData.throttle_pct);

  int barX = rx, barY = 16, barW = 95, barH = 16;
  sprite1.drawRect(barX, barY, barW, barH, 0x4208);
  int fillW = (int)(liveData.throttle_pct / 100.0f * (barW - 2));
  for (int x = 0; x < fillW; x++) {
    float f = (float)x / (barW - 2);
    uint8_t r, g;
    if (f < 0.5f) { g = 220; r = (uint8_t)(f * 2 * 220); }
    else { r = 220; g = (uint8_t)((1.0f - f) * 2 * 220); }
    sprite1.drawFastVLine(barX + 1 + x, barY + 1, barH - 2, sprite1.color565(r, g, 0));
  }

  // RPM
  sprite1.setTextColor(0xC618);
  sprite1.setCursor(rx, 38);
  sprite1.print("RPM");
  float rpmFrac = liveData.throttle_pct / 100.0f * 0.7f + liveData.speed_kmh / 200.0f * 0.3f;
  sprite1.setCursor(rx + 30, 38);
  sprite1.printf("%4d", (int)(rpmFrac * 8000));

  int rpmBarY = 50;
  sprite1.drawRect(rx, rpmBarY, barW, 12, 0x2104);
  int rpmFill = (int)(rpmFrac * (barW - 2));
  for (int x = 0; x < rpmFill; x++) {
    float f = (float)x / (barW - 2);
    uint16_t col = f < 0.7f ? sprite1.color565(0, 140, 0) :
                   f < 0.85f ? TFT_YELLOW : TFT_RED;
    sprite1.drawFastVLine(rx + 1 + x, rpmBarY + 1, 10, col);
  }

  // Separator line
  sprite1.drawFastHLine(rx, 68, barW, 0x2104);

  // ═══════════════════════════════════════════════════════════════════════════
  // VEHICLE STATUS ICONS (right side, below RPM)
  // ═══════════════════════════════════════════════════════════════════════════
  int iy = 76;  // icon row Y
  int iconW = 22, iconGap = 2;

  // Simulate blinker flashing (500ms cycle)
  bool blinkerPhase = (millis() / 500) % 2 == 0;
  uint8_t blinkers = liveData.blinker_state;
  bool leftOn  = (blinkers & 0x01) && blinkerPhase;
  bool rightOn = (blinkers & 0x02) && blinkerPhase;
  bool hazard  = (blinkers & 0x04) && blinkerPhase;

  // Left blinker arrow
  int ix = rx;
  uint16_t lbCol = (leftOn || hazard) ? TFT_ORANGE : 0x2104;
  sprite1.fillTriangle(ix, iy+8, ix+8, iy+2, ix+8, iy+14, lbCol);
  sprite1.fillRect(ix+8, iy+5, 6, 6, lbCol);

  // Right blinker arrow
  ix = rx + iconW + iconGap;
  uint16_t rbCol = (rightOn || hazard) ? TFT_ORANGE : 0x2104;
  sprite1.fillTriangle(ix+14, iy+8, ix+6, iy+2, ix+6, iy+14, rbCol);
  sprite1.fillRect(ix, iy+5, 6, 6, rbCol);

  // Headlights icon
  ix = rx + 2*(iconW + iconGap);
  uint16_t lightCol;
  switch (liveData.light_mode) {
    case 0: lightCol = 0x2104; break;  // off
    case 1: lightCol = 0x4208; break;  // parking (dim)
    case 2: lightCol = TFT_YELLOW; break;  // normal
    case 3: lightCol = TFT_CYAN; break;  // high beam
    default: lightCol = 0x2104;
  }
  // Light beam icon: circle + rays
  sprite1.fillCircle(ix+7, iy+8, 4, lightCol);
  sprite1.drawLine(ix+12, iy+4, ix+18, iy+2, lightCol);
  sprite1.drawLine(ix+12, iy+8, ix+19, iy+8, lightCol);
  sprite1.drawLine(ix+12, iy+12, ix+18, iy+14, lightCol);

  // Brake light icon
  ix = rx + 3*(iconW + iconGap);
  bool braking = liveData.motor_state & 0x08;
  uint16_t brakeCol = braking ? TFT_RED : 0x2104;
  // Circle with "!" inside
  sprite1.drawCircle(ix+7, iy+8, 7, brakeCol);
  sprite1.fillRect(ix+6, iy+3, 3, 7, brakeCol);
  sprite1.fillRect(ix+6, iy+12, 3, 2, brakeCol);

  // ── Second row: vehicle state + direction ──
  int iy2 = iy + 22;

  // Vehicle state
  uint16_t stateCol;
  const char* stateTxt;
  switch (liveData.vehicle_state) {
    case 0: stateCol = 0x4208; stateTxt = "OFF"; break;
    case 1: stateCol = TFT_YELLOW; stateTxt = "STBY"; break;
    case 2: stateCol = TFT_GREEN; stateTxt = "ARM"; break;
    default: stateCol = 0x4208; stateTxt = "?";
  }
  sprite1.setTextColor(stateCol);
  sprite1.setTextSize(1);
  sprite1.setCursor(rx, iy2);
  sprite1.print(stateTxt);

  // Direction indicator
  bool fwd = liveData.forward;
  sprite1.setCursor(rx + 32, iy2);
  sprite1.setTextColor(fwd ? TFT_GREEN : TFT_ORANGE);
  sprite1.print(fwd ? "FWD" : "REV");

  // Horn indicator
  if (liveData.horn_active) {
    sprite1.setCursor(rx + 60, iy2);
    sprite1.setTextColor(TFT_YELLOW);
    sprite1.print("HORN");
  }

  // ── Third row: heading ──
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(rx, iy2 + 14);
  sprite1.printf("HDG %.0f%c", liveData.heading_deg, (char)247);

  sprite1.pushSprite(0, 0);
}

// ─── Placeholder screen renderer ─────────────────────────────────────────────

// ─── D1 Navigation screen (240x135 landscape) ───────────────────────────────

void renderD1Navigation() {
  sprite1.fillScreen(TFT_BLACK);

  // Layout: [Speed+Throttle 30px] [Nav Arrow 75px] [Compass 135px]
  float hdgDeg = liveData.heading_deg;
  int8_t turnDir = navRoute.active ? navRoute.nextTurnDir : 0;
  float turnDist = navRoute.active ? navRoute.distToNextTurnM : 9999;

  // ── Left strip: Speed + vertical throttle bar (0..29) ──

  // Speed (large)
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(2);
  int spdVal = (int)liveData.speed_kmh;
  if (spdVal < 10) sprite1.setCursor(6, 4);
  else if (spdVal < 100) sprite1.setCursor(0, 4);
  else sprite1.setCursor(0, 4);
  sprite1.printf("%d", spdVal);
  sprite1.setTextSize(1);
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(4, 22);
  sprite1.print("km/h");

  // Vertical throttle bar (rotated -90: bottom=0%, top=100%)
  int tbX = 6, tbY = 34, tbW = 14, tbH = 96;
  sprite1.drawRect(tbX, tbY, tbW, tbH, 0x2104);
  int fillH = (int)(liveData.throttle_pct / 100.0f * (tbH - 2));
  for (int y = 0; y < fillH; y++) {
    float f = (float)y / (tbH - 2);
    uint8_t r = (f < 0.5f) ? (uint8_t)(f * 2 * 200) : 200;
    uint8_t g = (f < 0.5f) ? 200 : (uint8_t)((1.0f - f) * 2 * 200);
    sprite1.drawFastHLine(tbX + 1, tbY + tbH - 2 - y, tbW - 2, sprite1.color565(r, g, 0));
  }
  // Throttle percentage below bar
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(2, tbY + tbH + 2);
  sprite1.printf("%d%%", (int)liveData.throttle_pct);

  // ── Center: Navigation arrow (30..104) with padding ──
  int ax = 68, ay = 56;

  if (navRoute.active && navRoute.waitingForGPS) {
    // ── Waiting for GPS at start point ──
    // Pulsing ring
    float pulse = (sinf(millis() * 0.004f) + 1.0f) * 0.5f;  // 0..1
    uint8_t brightness = (uint8_t)(80 + 175 * pulse);
    uint16_t ringCol = sprite1.color565(0, brightness, brightness);
    sprite1.drawCircle(ax, ay, 28, ringCol);
    sprite1.drawCircle(ax, ay, 29, ringCol);
    sprite1.drawCircle(ax, ay, 30, ringCol);

    // Arrow pointing toward start point
    float bearRad = navRoute.startBearingDeg * PI / 180.0f;
    int tipX = ax + (int)(22.0f * sinf(bearRad));
    int tipY = ay - (int)(22.0f * cosf(bearRad));
    int baseX1 = ax + (int)(8.0f * sinf(bearRad + 2.5f));
    int baseY1 = ay - (int)(8.0f * cosf(bearRad + 2.5f));
    int baseX2 = ax + (int)(8.0f * sinf(bearRad - 2.5f));
    int baseY2 = ay - (int)(8.0f * cosf(bearRad - 2.5f));
    uint16_t arrowCol = ((millis() / 500) % 2) ? TFT_CYAN : sprite1.color565(0, 100, 100);
    sprite1.fillTriangle(tipX, tipY, baseX1, baseY1, baseX2, baseY2, arrowCol);

    // Distance text
    sprite1.setTextColor(TFT_CYAN);
    sprite1.setTextSize(1);
    sprite1.setCursor(36, 100);
    if (navRoute.startDistM < 1000)
      sprite1.printf("%.0fm away", navRoute.startDistM);
    else
      sprite1.printf("%.1fkm", navRoute.startDistM / 1000.0f);

    // "GO TO START" label
    sprite1.setTextColor(((millis() / 600) % 2) ? TFT_WHITE : 0x6B6D);
    sprite1.setCursor(34, 115);
    sprite1.print("GO TO START");
  } else if (turnDir == 0 || turnDist > 50) {
    // Straight
    sprite1.fillTriangle(ax, ay - 35, ax - 18, ay + 5, ax + 18, ay + 5, TFT_GREEN);
    sprite1.fillRect(ax - 8, ay + 5, 16, 24, TFT_GREEN);
    sprite1.setTextColor(TFT_GREEN);
    sprite1.setTextSize(1);
    sprite1.setCursor(46, 100);
    sprite1.print("AHEAD");
  } else if (turnDir > 0) {
    // Right
    uint16_t col = (turnDist < 20) ? TFT_RED : TFT_YELLOW;
    sprite1.fillTriangle(ax + 32, ay, ax - 2, ay - 18, ax - 2, ay + 18, col);
    sprite1.fillRect(ax - 24, ay - 8, 22, 16, col);
    sprite1.setTextColor(col);
    sprite1.setTextSize(1);
    sprite1.setCursor(46, 100);
    sprite1.print("RIGHT");
  } else {
    // Left
    uint16_t col = (turnDist < 20) ? TFT_RED : TFT_YELLOW;
    sprite1.fillTriangle(ax - 32, ay, ax + 2, ay - 18, ax + 2, ay + 18, col);
    sprite1.fillRect(ax + 2, ay - 8, 22, 16, col);
    sprite1.setTextColor(col);
    sprite1.setTextSize(1);
    sprite1.setCursor(50, 100);
    sprite1.print("LEFT");
  }

  // Turn distance from route engine
  if (!navRoute.waitingForGPS) {
    sprite1.setTextColor(TFT_WHITE);
    sprite1.setTextSize(1);
    sprite1.setCursor(36, 115);
    if (navRoute.active && turnDir != 0) {
      sprite1.printf("%.0fm", turnDist);
    } else if (navRoute.active) {
      sprite1.printf("%.0fm left", navRoute.remainingDistM);
    }
  }

  // ── Right: Compass (105..239) ──
  int cx = 172, cy = 67, cr = 56;

  // Compass ring
  sprite1.drawCircle(cx, cy, cr, 0x4208);
  sprite1.drawCircle(cx, cy, cr + 1, 0x4208);

  // Tick marks
  for (int a = 0; a < 360; a += 15) {
    float rad = (a - hdgDeg) * PI / 180.0f;
    int len = (a % 90 == 0) ? 8 : (a % 30 == 0) ? 5 : 3;
    int ox = cx + (int)(cr * sinf(rad));
    int oy = cy - (int)(cr * cosf(rad));
    int ix = cx + (int)((cr - len) * sinf(rad));
    int iy = cy - (int)((cr - len) * cosf(rad));
    sprite1.drawLine(ix, iy, ox, oy, (a % 90 == 0) ? 0xC618 : 0x4208);
  }

  // Cardinal directions
  const char* dirs[] = {"N", "E", "S", "W"};
  float dirAng[] = {0, 90, 180, 270};
  for (int i = 0; i < 4; i++) {
    float rad = (dirAng[i] - hdgDeg) * PI / 180.0f;
    int dx = cx + (int)((cr - 16) * sinf(rad)) - 3;
    int dy = cy - (int)((cr - 16) * cosf(rad)) - 4;
    sprite1.setTextColor(i == 0 ? TFT_RED : 0x8410);
    sprite1.setTextSize(1);
    sprite1.setCursor(dx, dy);
    sprite1.print(dirs[i]);
  }

  // Forward needle
  sprite1.fillTriangle(cx, cy - 30, cx - 6, cy - 18, cx + 6, cy - 18, TFT_RED);
  sprite1.fillTriangle(cx, cy + 30, cx - 5, cy + 18, cx + 5, cy + 18, 0x2104);

  // Heading text centered
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(1);
  char hdgTxt[8];
  snprintf(hdgTxt, sizeof(hdgTxt), "%.0f%c", hdgDeg, (char)247);
  int htw = strlen(hdgTxt) * 6;
  sprite1.setCursor(cx - htw / 2, cy - 3);
  sprite1.print(hdgTxt);

  sprite1.pushSprite(0, 0);
}

// ─── D1 Media screen (240x135 landscape) ─────────────────────────────────────

// Reuse song data from D2 media
extern const char* songNames[];
extern const char* artistNames[];
extern const int songCount;
extern int mediaSongIdx;
extern bool mediaPlaying;
extern float freqBands[];
extern float freqTargets[];

void renderD1Media() {
  sprite1.fillScreen(TFT_BLACK);

  // Layout: [Speed+Throttle 28px] [Album+Info 90px] [EQ bottom]

  // ── Left strip: Speed + vertical throttle (0..27) ──
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(2);
  int spdVal = (int)liveData.speed_kmh;
  sprite1.setCursor(spdVal < 10 ? 6 : 0, 4);
  sprite1.printf("%d", spdVal);
  sprite1.setTextSize(1);
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(4, 22);
  sprite1.print("km/h");

  // Vertical throttle bar
  int tbX = 6, tbY = 34, tbW = 14, tbH = 96;
  sprite1.drawRect(tbX, tbY, tbW, tbH, 0x2104);
  int fillH = (int)(liveData.throttle_pct / 100.0f * (tbH - 2));
  for (int y = 0; y < fillH; y++) {
    float f = (float)y / (tbH - 2);
    uint8_t r = (f < 0.5f) ? (uint8_t)(f * 2 * 200) : 200;
    uint8_t g = (f < 0.5f) ? 200 : (uint8_t)((1.0f - f) * 2 * 200);
    sprite1.drawFastHLine(tbX + 1, tbY + tbH - 2 - y, tbW - 2, sprite1.color565(r, g, 0));
  }
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(2, tbY + tbH + 2);
  sprite1.printf("%d%%", (int)liveData.throttle_pct);

  // ── Album art (small) ──
  int artX = 30, artY = 4, artS = 44;
  uint16_t artCol = sprite1.color565(
    40 + (mediaSongIdx * 23) % 80,
    30 + (mediaSongIdx * 37) % 60,
    50 + (mediaSongIdx * 41) % 90
  );
  sprite1.fillRoundRect(artX, artY, artS, artS, 5, artCol);
  sprite1.fillCircle(artX + 14, artY + 30, 5, TFT_WHITE);
  sprite1.fillCircle(artX + 28, artY + 26, 5, TFT_WHITE);
  sprite1.fillRect(artX + 18, artY + 9, 2, 23, TFT_WHITE);
  sprite1.fillRect(artX + 32, artY + 6, 2, 22, TFT_WHITE);
  sprite1.drawFastHLine(artX + 18, artY + 9, 16, TFT_WHITE);

  // ── Song info (right of art) ──
  int tx = 80;
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(1);
  char songBuf[22];
  strncpy(songBuf, songNames[mediaSongIdx], 21); songBuf[21] = '\0';
  sprite1.setCursor(tx, 6);
  sprite1.print(songBuf);

  sprite1.setTextColor(0x8410);
  sprite1.setCursor(tx, 18);
  sprite1.print(artistNames[mediaSongIdx]);

  // Play/pause icon + progress bar
  int pcx = tx, pcy = 34;
  if (mediaPlaying) {
    sprite1.fillRect(pcx, pcy, 4, 10, TFT_WHITE);
    sprite1.fillRect(pcx + 6, pcy, 4, 10, TFT_WHITE);
  } else {
    sprite1.fillTriangle(pcx, pcy, pcx, pcy + 10, pcx + 9, pcy + 5, TFT_WHITE);
  }

  float progress = fmodf(millis() * 0.00003f, 1.0f);
  int pbX = pcx + 16, pbW = 232 - pbX;
  sprite1.drawRoundRect(pbX, pcy + 2, pbW, 5, 2, 0x4208);
  sprite1.fillRoundRect(pbX + 1, pcy + 3, (int)(progress * (pbW - 2)), 3, 1, TFT_CYAN);

  int totalSec = 240;
  int curSec = (int)(progress * totalSec);
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(pcx + 16, pcy + 10);
  sprite1.printf("%d:%02d / %d:%02d", curSec / 60, curSec % 60, totalSec / 60, totalSec % 60);

  // ── Frequency visualizer (bottom, y=54..132) ──
  int vizX = 28, vizY = 56, vizH = 76, vizW = 208;
  int bandCount = 32;
  int bandW = vizW / bandCount;

  sprite1.drawFastHLine(vizX, vizY - 2, vizW, 0x2104);

  for (int i = 0; i < bandCount; i++) {
    int bx = vizX + i * bandW;
    int bh = (int)(freqBands[i] * vizH / 120.0f);
    if (bh > vizH) bh = vizH;
    int by = vizY + vizH - bh;

    for (int y = 0; y < bh; y++) {
      float f = (float)y / max(1, vizH);
      uint8_t r = (uint8_t)(f * 50);
      uint8_t g = (uint8_t)(100 + f * 155);
      uint8_t b = (uint8_t)(255 - f * 200);
      sprite1.drawFastHLine(bx, by + (bh - 1 - y), bandW - 1, sprite1.color565(r, g, b));
    }
  }

  sprite1.pushSprite(0, 0);
}

// ─── D1 About screen (240x135 landscape) ─────────────────────────────────────

static const char* aboutLines[] = {
  "",
  "RC CAR NAV SYSTEM",
  "v1.0",
  "",
  "Hardware:",
  "  LilyGo T-Display v1.1",
  "  Waveshare 1.47\" LCD",
  "  MEGA 2560 PRO Mini",
  "  MicroSD 32GB",
  "",
  "Displays:",
  "  D1: ST7789 135x240",
  "  D2: ST7789V2 172x320",
  "",
  "Communication:",
  "  I2C Slave @ 0x42",
  "  SDA=GPIO26 SCL=GPIO32",
  "  SPI: VSPI + HSPI",
  "",
  "Features:",
  "  OSM Tile Navigation",
  "  Web Map Manager",
  "  Dashboard Gauges",
  "  Media Player",
  "  Battery Monitor",
  "  Signal Monitor",
  "  Debug Console",
  "",
  "Built with:",
  "  LovyanGFX",
  "  Arduino ESP32 Core",
  "  Leaflet.js",
  "",
  "Designed & coded with",
  "assistance of Claude",
  "",
  "",
  "",
};
static const int aboutLineCount = sizeof(aboutLines) / sizeof(aboutLines[0]);
static float aboutScrollY = 0;

void renderD1About() {
  sprite1.fillScreen(TFT_BLACK);

  // ── Left strip: Speed + vertical throttle (same as nav/media) ──
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(2);
  int spdVal = (int)liveData.speed_kmh;
  sprite1.setCursor(spdVal < 10 ? 6 : 0, 4);
  sprite1.printf("%d", spdVal);
  sprite1.setTextSize(1);
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(4, 22);
  sprite1.print("km/h");

  int tbX = 6, tbY = 34, tbW = 14, tbH = 96;
  sprite1.drawRect(tbX, tbY, tbW, tbH, 0x2104);
  int fillH = (int)(liveData.throttle_pct / 100.0f * (tbH - 2));
  for (int y = 0; y < fillH; y++) {
    float f = (float)y / (tbH - 2);
    uint8_t r = (f < 0.5f) ? (uint8_t)(f * 2 * 200) : 200;
    uint8_t g = (f < 0.5f) ? 200 : (uint8_t)((1.0f - f) * 2 * 200);
    sprite1.drawFastHLine(tbX + 1, tbY + tbH - 2 - y, tbW - 2, sprite1.color565(r, g, 0));
  }
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(2, tbY + tbH + 2);
  sprite1.printf("%d%%", (int)liveData.throttle_pct);

  // ── Right area: scrolling about text (28..239, 0..134) ──
  int txArea = 30, txW = 208;
  int lh = 12;
  int totalH = aboutLineCount * lh;

  // Scroll speed: ~0.5 px per frame
  aboutScrollY += 0.5f;
  if (aboutScrollY > totalH) aboutScrollY = -135;  // restart after all lines scroll past

  // Separator
  sprite1.drawFastVLine(txArea - 3, 0, 135, 0x2104);

  for (int i = 0; i < aboutLineCount; i++) {
    int lineY = (int)(i * lh - aboutScrollY);
    if (lineY < -lh || lineY >= 135) continue;  // off screen

    const char* line = aboutLines[i];
    if (line[0] == '\0') continue;

    // Title lines in cyan, rest in white/gray
    bool isTitle = (i == 1);  // "RC CAR NAV SYSTEM"
    bool isVersion = (i == 2);
    bool isSection = (line[0] != ' ' && i > 2);

    if (isTitle) {
      sprite1.setTextColor(TFT_CYAN);
      sprite1.setTextSize(1);
      int tw = strlen(line) * 6;
      sprite1.setCursor(txArea + (txW - tw) / 2, lineY);
    } else if (isVersion) {
      sprite1.setTextColor(0x8410);
      sprite1.setTextSize(1);
      int tw = strlen(line) * 6;
      sprite1.setCursor(txArea + (txW - tw) / 2, lineY);
    } else if (isSection) {
      sprite1.setTextColor(TFT_GREEN);
      sprite1.setTextSize(1);
      sprite1.setCursor(txArea + 4, lineY);
    } else {
      sprite1.setTextColor(0xC618);
      sprite1.setTextSize(1);
      sprite1.setCursor(txArea + 4, lineY);
    }
    sprite1.print(line);
  }

  sprite1.pushSprite(0, 0);
}

// ─── D1 IMU tilt visualization ──────────────────────────────────────────────

// Helper: draw a filled quad from 4 points
static void fillQuad(LGFX_Sprite& s, int x0,int y0, int x1,int y1,
                     int x2,int y2, int x3,int y3, uint16_t c) {
  s.fillTriangle(x0,y0,x1,y1,x2,y2, c);
  s.fillTriangle(x1,y1,x2,y2,x3,y3, c);
}

// Draw rear view of monster truck, centered at (cx,cy), rotated by roll angle
static void drawTruckRear(LGFX_Sprite& s, int cx, int cy, float ang) {
  float sr = sinf(ang), cr = cosf(ang);
  #define R(px,py,ox,oy) ox=cx+(int)((px)*cr-(py)*sr); oy=cy+(int)((px)*sr+(py)*cr)
  int _x0,_y0,_x1,_y1,_x2,_y2,_x3,_y3;

  // ── Ground ──
  R(-52, 28, _x0, _y0); R(52, 28, _x1, _y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x18C3);

  // ── Tires (rectangular from behind — we see the tread face) ──
  int wS = 24, wY = 16;  // wheel spread, wheel center Y
  int wW = 10, wH = 20;  // tire width and height as seen from rear
  for (int side = -1; side <= 1; side += 2) {
    // Tire rectangle corners
    int _a,_b,_c,_d,_e,_f,_g,_h;
    R(side*wS - wW/2, wY - wH/2, _a, _b);
    R(side*wS + wW/2, wY - wH/2, _c, _d);
    R(side*wS + wW/2, wY + wH/2, _e, _f);
    R(side*wS - wW/2, wY + wH/2, _g, _h);
    // Outer tire (dark)
    fillQuad(s, _a,_b, _c,_d, _e,_f, _g,_h, 0x2945);
    // Tread pattern — horizontal lines across tire face
    for (int t = -3; t <= 3; t++) {
      R(side*wS - wW/2+1, wY + t*3, _x0,_y0);
      R(side*wS + wW/2-1, wY + t*3, _x1,_y1);
      s.drawLine(_x0,_y0,_x1,_y1, 0x39C7);
    }
    // Tire outline
    s.drawLine(_a,_b,_c,_d, 0x4A69);
    s.drawLine(_c,_d,_e,_f, 0x4A69);
    s.drawLine(_e,_f,_g,_h, 0x4A69);
    s.drawLine(_g,_h,_a,_b, 0x4A69);
    // Hub cap (small circle in center)
    int hx,hy; R(side*wS, wY, hx, hy);
    s.fillCircle(hx,hy, 3, 0x6B6D);
    s.drawCircle(hx,hy, 3, 0x8410);
    s.fillCircle(hx,hy, 1, 0xAD75);
  }

  // ── Axle (connecting wheels) ──
  R(-wS, wY, _x0,_y0); R(wS, wY, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x4A69);

  // ── Suspension arms ──
  R(-wS, wY-wH/2+2, _x0,_y0); R(-16, 0, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x6B6D);
  s.drawLine(_x0+1,_y0,_x1+1,_y1, 0x4A69);
  R(wS, wY-wH/2+2, _x0,_y0); R(16, 0, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x6B6D);
  s.drawLine(_x0-1,_y0,_x1-1,_y1, 0x4A69);
  // Shocks
  R(-wS+3, wY-wH/2, _x0,_y0); R(-12, -8, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x8410);
  R(wS-3, wY-wH/2, _x0,_y0); R(12, -8, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x8410);

  // ── Body/bed (wide rectangle) ──
  R(-22, 1, _x0,_y0); R(22, 1, _x1,_y1);
  R(-24, -7, _x2,_y2); R(24, -7, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x1928);
  // Fender flares (arches over wheels)
  R(-26, 0, _x0,_y0); R(-22, 1, _x1,_y1);
  R(-28, -7, _x2,_y2); R(-24, -7, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x18E3);
  R(22, 1, _x0,_y0); R(26, 0, _x1,_y1);
  R(24, -7, _x2,_y2); R(28, -7, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x18E3);
  // Body outline
  R(-28, -7, _x0,_y0); R(28, -7, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x528A);

  // ── Cab (symmetric from rear) ──
  R(-18, -7, _x0,_y0); R(18, -7, _x1,_y1);
  R(-14, -24, _x2,_y2); R(14, -24, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x1928);
  s.drawLine(_x0,_y0,_x1,_y1, 0x528A);
  s.drawLine(_x0,_y0,_x2,_y2, 0x528A);
  s.drawLine(_x1,_y1,_x3,_y3, 0x528A);
  s.drawLine(_x2,_y2,_x3,_y3, 0x528A);

  // ── Rear window (symmetric rectangle) ──
  R(-12, -9, _x0,_y0); R(12, -9, _x1,_y1);
  R(-10, -21, _x2,_y2); R(10, -21, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x0010);
  s.drawLine(_x0,_y0,_x1,_y1, 0x2945);
  s.drawLine(_x2,_y2,_x3,_y3, 0x2945);
  s.drawLine(_x0,_y0,_x2,_y2, 0x2945);
  s.drawLine(_x1,_y1,_x3,_y3, 0x2945);
  // Center divider
  R(0, -9, _x0,_y0); R(0, -21, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x2945);

  // ── Roof + light bar ──
  R(-14, -24, _x0,_y0); R(14, -24, _x1,_y1);
  R(-15, -26, _x2,_y2); R(15, -26, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x2104);
  for (int i = -2; i <= 2; i++) {
    int lx,ly; R(i*6, -27, lx, ly);
    uint16_t lc = liveData.light_mode >= 2 ? TFT_YELLOW : 0x3186;
    s.fillRect(lx-1, ly-1, 3, 2, lc);
  }

  // ── Tail lights (symmetric, on body sides) ──
  R(-23, -4, _x0,_y0); s.fillRect(_x0-1,_y0-2, 3,4, TFT_RED);
  R( 23, -4, _x0,_y0); s.fillRect(_x0-1,_y0-2, 3,4, TFT_RED);

  // ── License plate area ──
  R(-6, -1, _x0,_y0); R(6, -1, _x1,_y1);
  R(-6, -5, _x2,_y2); R(6, -5, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x4208);

  #undef R
}

// Draw side view of monster truck (front faces LEFT), centered at (cx,cy)
static void drawTruckSide(LGFX_Sprite& s, int cx, int cy, float ang) {
  float sr = sinf(ang), cr = cosf(ang);
  // Note: X coords are negated vs old version to flip horizontally (front=left)
  #define R(px,py,ox,oy) ox=cx+(int)(-(px)*cr-(py)*sr); oy=cy+(int)(-(px)*sr+(py)*cr)
  int _x0,_y0,_x1,_y1,_x2,_y2,_x3,_y3;

  // ── Ground ──
  R(-55, 28, _x0, _y0); R(55, 28, _x1, _y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x18C3);

  // ── Wheels (front & rear, round from side) ──
  int wR = 12, wY = 16;
  int fwx,fwy,rwx,rwy;
  R(-26, wY, rwx, rwy);  // rear
  R( 26, wY, fwx, fwy);  // front
  for (int w = 0; w < 2; w++) {
    int wx = w ? fwx : rwx, wy = w ? fwy : rwy;
    s.fillCircle(wx,wy, wR, 0x2945);
    s.drawCircle(wx,wy, wR, 0x4A69);
    s.drawCircle(wx,wy, wR-1, 0x39C7);
    s.fillCircle(wx,wy, wR-3, 0x2104);
    // Rim spokes
    s.drawCircle(wx,wy, 5, 0x8410);
    s.fillCircle(wx,wy, 3, 0x6B6D);
    s.fillCircle(wx,wy, 1, 0xAD75);
  }

  // ── Wheel fenders (arcs above wheels) ──
  // Rear fender
  R(-26, wY-wR-1, _x0,_y0); R(-36, wY-4, _x1,_y1);
  R(-16, wY-4, _x2,_y2);
  s.drawLine(_x1,_y1,_x0,_y0, 0x528A);
  s.drawLine(_x0,_y0,_x2,_y2, 0x528A);
  // Rear fender fill
  R(-36, wY-4, _x0,_y0); R(-16, wY-4, _x1,_y1);
  R(-34, wY-8, _x2,_y2); R(-18, wY-8, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x18E3);
  // Front fender
  R(26, wY-wR-1, _x0,_y0); R(16, wY-4, _x1,_y1);
  R(36, wY-4, _x2,_y2);
  s.drawLine(_x1,_y1,_x0,_y0, 0x528A);
  s.drawLine(_x0,_y0,_x2,_y2, 0x528A);
  R(16, wY-4, _x0,_y0); R(36, wY-4, _x1,_y1);
  R(18, wY-8, _x2,_y2); R(34, wY-8, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x18E3);

  // ── Suspension ──
  R(-26, wY-wR+3, _x0,_y0); R(-20, 0, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x6B6D);
  R(26, wY-wR+3, _x0,_y0); R(20, 0, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x6B6D);
  // Shocks
  R(-22, wY-wR, _x0,_y0); R(-14, -8, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x8410);
  R(22, wY-wR, _x0,_y0); R(14, -8, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x8410);

  // ── Chassis rail ──
  R(-32, 1, _x0,_y0); R(30, 1, _x1,_y1);
  R(-32, -3, _x2,_y2); R(30, -3, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x10A2);

  // ── Rear bed (front=left, so bed is on RIGHT side now) ──
  R(-36, -3, _x0,_y0); R(-8, -3, _x1,_y1);
  R(-34, -10, _x2,_y2); R(-8, -10, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x1928);
  s.drawLine(_x0,_y0,_x2,_y2, 0x528A);
  s.drawLine(_x2,_y2,_x3,_y3, 0x528A);

  // ── Cab ──
  R(-8, -3, _x0,_y0); R(28, -3, _x1,_y1);
  R(-8, -22, _x2,_y2); R(22, -22, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x1928);
  s.drawLine(_x0,_y0,_x2,_y2, 0x528A);
  s.drawLine(_x2,_y2,_x3,_y3, 0x528A);
  // Windshield slope
  R(28, -3, _x0,_y0); R(22, -22, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x528A);

  // ── Side window ──
  R(-5, -8, _x0,_y0); R(16, -8, _x1,_y1);
  R(-3, -20, _x2,_y2); R(14, -20, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x0010);
  s.drawLine(_x0,_y0,_x1,_y1, 0x2945);
  s.drawLine(_x2,_y2,_x3,_y3, 0x2945);
  s.drawLine(_x0,_y0,_x2,_y2, 0x2945);
  s.drawLine(_x1,_y1,_x3,_y3, 0x2945);
  // Door line
  R(4, -7, _x0,_y0); R(4, -2, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x39C7);

  // ── Roof ──
  R(-9, -22, _x0,_y0); R(22, -22, _x1,_y1);
  R(-10, -24, _x2,_y2); R(22, -24, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x2104);

  // ── Bumpers ──
  R(30, -1, _x0,_y0); R(36, -1, _x1,_y1);
  R(30, -5, _x2,_y2); R(36, -5, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x4208);
  R(-36, -1, _x0,_y0); R(-34, -1, _x1,_y1);
  R(-36, -5, _x2,_y2); R(-34, -5, _x3,_y3);
  fillQuad(s, _x0,_y0,_x1,_y1,_x3,_y3,_x2,_y2, 0x4208);

  // ── Lights (NOT flipped — keep correct orientation) ──
  // Headlight on front (left side of screen since front=left)
  // Use original (non-negated) coords for lights
  #undef R
  #define R2(px,py,ox,oy) ox=cx+(int)((px)*cr-(py)*sr); oy=cy+(int)((px)*sr+(py)*cr)
  R2(-34, -5, _x0,_y0);  // front-left = headlight
  s.fillCircle(_x0,_y0, 2, TFT_RED);
  R2(35, -5, _x0,_y0);   // rear-right = tail light
  s.fillCircle(_x0,_y0, 2, liveData.light_mode >= 2 ? TFT_YELLOW : 0x4208);
  #undef R2

  // ── Exhaust (rear side, now on right) ──
  #define R(px,py,ox,oy) ox=cx+(int)(-(px)*cr-(py)*sr); oy=cy+(int)(-(px)*sr+(py)*cr)
  R(-32, 3, _x0,_y0); R(-38, 5, _x1,_y1);
  s.drawLine(_x0,_y0,_x1,_y1, 0x6B6D);

  #undef R
}

void renderD1IMU() {
  sprite1.fillScreen(TFT_BLACK);

  // ── Left strip: Speed + vertical throttle ──
  sprite1.setTextColor(TFT_WHITE);
  sprite1.setTextSize(2);
  int spdVal = (int)liveData.speed_kmh;
  sprite1.setCursor(spdVal < 10 ? 6 : 0, 4);
  sprite1.printf("%d", spdVal);
  sprite1.setTextSize(1);
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(4, 22);
  sprite1.print("km/h");

  int tbX = 6, tbY = 34, tbW = 14, tbH = 76;
  sprite1.drawRect(tbX, tbY, tbW, tbH, 0x2104);
  int fillH = (int)(liveData.throttle_pct / 100.0f * (tbH - 2));
  for (int y = 0; y < fillH; y++) {
    float f = (float)y / (tbH - 2);
    uint8_t r = (f < 0.5f) ? (uint8_t)(f * 2 * 200) : 200;
    uint8_t g = (f < 0.5f) ? 200 : (uint8_t)((1.0f - f) * 2 * 200);
    sprite1.drawFastHLine(tbX + 1, tbY + tbH - 2 - y, tbW - 2, sprite1.color565(r, g, 0));
  }
  sprite1.setTextColor(0x6B6D);
  sprite1.setCursor(2, tbY + tbH + 2);
  sprite1.printf("%d%%", (int)liveData.throttle_pct);

  sprite1.drawFastVLine(27, 0, 135, 0x2104);

  // ── IMU values ──
  float roll  = constrain(liveData.accel_x, -1.0f, 1.0f);
  float pitch = constrain(liveData.accel_y, -1.0f, 1.0f);
  float rollAng  = roll  * 0.52f;   // max ±30°
  float pitchAng = pitch * 0.52f;

  // ── Labels ──
  sprite1.setTextSize(1);
  sprite1.setTextColor(0x4A69);
  sprite1.setCursor(50, 1);  sprite1.print("REAR");
  sprite1.setCursor(155, 1); sprite1.print("SIDE");

  // ── Divider ──
  sprite1.drawFastVLine(132, 0, 112, 0x18C3);

  // ── Rear view (roll tilt) — left panel ──
  drawTruckRear(sprite1, 80, 55, rollAng);

  // ── Side view (pitch tilt) — right panel ──
  drawTruckSide(sprite1, 186, 55, pitchAng);

  // ── Bottom 2 lines: IMU text data (indented past left column) ──
  int tx = 30;
  sprite1.drawFastHLine(tx, 113, 240 - tx, 0x2104);
  sprite1.setTextSize(1);
  char buf[48];

  sprite1.setTextColor(0x8410);
  sprite1.setCursor(tx, 117);
  snprintf(buf, sizeof(buf), "R%+.1f P%+.1f Y%+.1f%c/s",
           liveData.accel_x, liveData.accel_y, liveData.gyro_z, (char)247);
  sprite1.print(buf);

  sprite1.setCursor(tx, 127);
  snprintf(buf, sizeof(buf), "Z%+.1fg T%.0f%cC %.0fPa",
           liveData.accel_z, liveData.temperature, (char)247, liveData.pressure_pa);
  sprite1.print(buf);

  sprite1.pushSprite(0, 0);
}

void renderD1Placeholder(const char* title) {
  sprite1.fillScreen(TFT_BLACK);
  sprite1.setTextColor(0x4208);
  sprite1.setTextSize(2);
  int tw = strlen(title) * 12;
  sprite1.setCursor((240 - tw) / 2, 55);
  sprite1.print(title);
  sprite1.pushSprite(0, 0);
}

// ─── D2 Battery screen ───────────────────────────────────────────────────────

// Battery screen state (for partial updates)
static bool battDrawn = false;
static int prevFillH = -1;
static int prevPct = -1;
static bool prevCharging = false;

void renderD2Battery() {
  float battLevel = liveData.batt_pct;
  bool charging = liveData.charging;

  int bw = 100, bh = 200;
  int bx = (172 - bw) / 2;
  int by = 60;
  int innerMargin = 5;
  int innerW = bw - innerMargin * 2;
  int innerH = bh - innerMargin * 2;
  int fillH = (int)(battLevel / 100.0f * innerH);
  int pct = (int)battLevel;

  // First frame: draw static parts
  if (!battDrawn) {
    disp2.startWrite();
    disp2.fillScreen(TFT_BLACK);

    // Terminal nub
    int tipW = 40, tipH = 14;
    int tx = bx + (bw - tipW) / 2;
    disp2.fillRoundRect(tx, by - tipH + 4, tipW, tipH, 4, 0x4208);

    // Body outline
    disp2.drawRoundRect(bx, by, bw, bh, 8, 0x8410);
    disp2.drawRoundRect(bx + 1, by + 1, bw - 2, bh - 2, 7, 0x8410);
    disp2.endWrite();

    battDrawn = true;
    prevFillH = -1;
    prevPct = -1;
    prevCharging = !charging;
  }

  // Only update if something changed
  if (fillH == prevFillH && pct == prevPct && charging == prevCharging) return;

  disp2.startWrite();

  // Update fill level (only redraw the changed area)
  int fillY = by + innerMargin + (innerH - fillH);
  int prevFillY = (prevFillH >= 0) ? by + innerMargin + (innerH - prevFillH) : by + innerMargin;

  // Clear area above new fill (if level decreased)
  if (fillH < prevFillH) {
    disp2.fillRect(bx + innerMargin, prevFillY, innerW, fillY - prevFillY, TFT_BLACK);
  }

  // Draw fill
  for (int y = 0; y < fillH; y++) {
    float f = (float)y / max(1, fillH);
    uint16_t col;
    if (battLevel > 50) {
      col = disp2.color565(0, 140 + (uint8_t)(f * 80), 0);
    } else if (battLevel > 20) {
      col = disp2.color565(140 + (uint8_t)(f * 80), 120 + (uint8_t)(f * 60), 0);
    } else {
      col = disp2.color565(140 + (uint8_t)(f * 80), 0, 0);
    }
    disp2.drawFastHLine(bx + innerMargin, fillY + y, innerW, col);
  }

  // Percentage text (clear old, draw new)
  if (pct != prevPct) {
    // Clear text area
    disp2.fillRect(bx + innerMargin, by + bh / 2 - 12, innerW, 24, TFT_BLACK);
    // Redraw fill behind text area
    int txtTop = by + bh / 2 - 12;
    int txtBot = txtTop + 24;
    for (int y = max(fillY, txtTop); y < min(fillY + fillH, txtBot); y++) {
      float f = (float)(y - fillY) / max(1, fillH);
      uint16_t col;
      if (battLevel > 50) col = disp2.color565(0, 140 + (uint8_t)(f * 80), 0);
      else if (battLevel > 20) col = disp2.color565(140 + (uint8_t)(f * 80), 120 + (uint8_t)(f * 60), 0);
      else col = disp2.color565(140 + (uint8_t)(f * 80), 0, 0);
      disp2.drawFastHLine(bx + innerMargin, y, innerW, col);
    }
    disp2.setTextSize(3);
    char pctTxt[6];
    snprintf(pctTxt, sizeof(pctTxt), "%d%%", pct);
    int ptw = strlen(pctTxt) * 18;
    disp2.setTextColor(TFT_WHITE);
    disp2.setCursor(bx + (bw - ptw) / 2, by + bh / 2 - 10);
    disp2.print(pctTxt);
  }

  // Charging indicator (only update on change)
  if (charging != prevCharging) {
    int cx = 172 / 2, cy = by + bh + 20;
    // Clear charging area
    disp2.fillRect(0, cy - 12, 172, 26, TFT_BLACK);
    if (charging) {
      disp2.fillTriangle(cx - 3, cy - 10, cx + 8, cy - 2, cx, cy - 2, TFT_YELLOW);
      disp2.fillTriangle(cx + 3, cy + 10, cx - 8, cy + 2, cx, cy + 2, TFT_YELLOW);
      disp2.setTextSize(1);
      disp2.setTextColor(TFT_YELLOW);
      disp2.setCursor(cx + 16, cy - 4);
      disp2.print("CHARGING");
    }
  }

  // Voltage
  float voltage = liveData.voltage;
  disp2.fillRect(0, by + bh + 40, 172, 12, TFT_BLACK);
  disp2.setTextSize(1);
  disp2.setTextColor(0x8410);
  char vTxt[16];
  snprintf(vTxt, sizeof(vTxt), "%.2fV", voltage);
  int vtw = strlen(vTxt) * 6;
  disp2.setCursor((172 - vtw) / 2, by + bh + 44);
  disp2.print(vTxt);

  disp2.endWrite();

  prevFillH = fillH;
  prevPct = pct;
  prevCharging = charging;
}

// ─── D2 Signal+Battery screen ────────────────────────────────────────────────

static bool sigBattDrawn = false;
static int prevSigBars = -1;
static int prevSigRssi = 999;
static int prevSigLq = -1;
static int prevSigFillH = -1;
static int prevSigPct = -1;

void renderD2SignalBattery() {
  int rssi = liveData.rssi_dbm;
  int lq = liveData.link_quality;
  int bars = liveData.signal_bars;
  if (bars < 1) bars = 1;

  float battLevel = liveData.batt_pct;
  int pct = (int)battLevel;

  // Static parts on first draw
  if (!sigBattDrawn) {
    disp2.startWrite();
    disp2.fillScreen(TFT_BLACK);

    // Labels
    disp2.setTextSize(1);
    disp2.setTextColor(0x4208);
    disp2.setCursor(8, 4);
    disp2.print("SIGNAL");

    // Divider
    disp2.drawFastHLine(4, 158, 164, 0x4208);

    disp2.setTextColor(0x4208);
    disp2.setCursor(8, 164);
    disp2.print("BATTERY");

    // Battery outline (bottom half: y=180..310)
    int bw = 80, bh = 120;
    int bx = (172 - bw) / 2, by = 185;
    int tipW = 30, tipH = 10;
    disp2.fillRoundRect(bx + (bw - tipW) / 2, by - tipH + 3, tipW, tipH, 3, 0x4208);
    disp2.drawRoundRect(bx, by, bw, bh, 6, 0x8410);
    disp2.drawRoundRect(bx + 1, by + 1, bw - 2, bh - 2, 5, 0x8410);

    disp2.endWrite();
    sigBattDrawn = true;
    prevSigBars = -1;
    prevSigRssi = 999;
    prevSigFillH = -1;
    prevSigPct = -1;
  }

  // ── Top half: Signal bars (y=16..150) ──
  if (bars != prevSigBars || rssi != prevSigRssi || lq != prevSigLq) {
    disp2.startWrite();

    // Clear bars area
    disp2.fillRect(8, 20, 156, 100, TFT_BLACK);

    // Draw signal bars (5 bars, increasing height)
    int barW = 22, barGap = 6;
    int barBaseY = 115;
    int totalW = 5 * barW + 4 * barGap;
    int startX = (172 - totalW) / 2;

    for (int i = 0; i < 5; i++) {
      int barH = 20 + i * 18;  // 20, 38, 56, 74, 92
      int bx = startX + i * (barW + barGap);
      int by = barBaseY - barH;

      if (i < bars) {
        // Active bar — color based on signal strength
        uint16_t col;
        if (bars >= 4) col = TFT_GREEN;
        else if (bars >= 3) col = TFT_YELLOW;
        else col = TFT_RED;
        disp2.fillRoundRect(bx, by, barW, barH, 3, col);
      } else {
        // Inactive bar — dim outline
        disp2.drawRoundRect(bx, by, barW, barH, 3, 0x2104);
      }
    }

    // RSSI and LQ text
    disp2.fillRect(4, 122, 164, 34, TFT_BLACK);
    disp2.setTextSize(1);
    disp2.setTextColor(TFT_WHITE);
    disp2.setCursor(8, 126);
    disp2.printf("RSSI: %d dBm", rssi);

    disp2.setCursor(8, 142);
    uint16_t lqCol = (lq > 70) ? TFT_GREEN : (lq > 40) ? TFT_YELLOW : TFT_RED;
    disp2.setTextColor(lqCol);
    disp2.printf("LQ:   %d%%", lq);

    disp2.endWrite();
    prevSigBars = bars;
    prevSigRssi = rssi;
    prevSigLq = lq;
  }

  // ── Bottom half: Battery (y=180..310) ──
  int bw = 80, bh = 120;
  int bx = (172 - bw) / 2, by = 185;
  int innerMargin = 4;
  int innerW = bw - innerMargin * 2;
  int innerH = bh - innerMargin * 2;
  int fillH = (int)(battLevel / 100.0f * innerH);

  if (fillH != prevSigFillH || pct != prevSigPct) {
    disp2.startWrite();

    int fillY = by + innerMargin + (innerH - fillH);

    // Clear empty part above fill
    if (prevSigFillH < 0 || fillH < prevSigFillH) {
      int prevFillY = (prevSigFillH >= 0) ? by + innerMargin + (innerH - prevSigFillH) : by + innerMargin;
      disp2.fillRect(bx + innerMargin, prevFillY, innerW, fillY - prevFillY, TFT_BLACK);
    }

    // Draw fill
    for (int y = 0; y < fillH; y++) {
      float f = (float)y / max(1, fillH);
      uint16_t col;
      if (battLevel > 50) col = disp2.color565(0, 140 + (uint8_t)(f * 80), 0);
      else if (battLevel > 20) col = disp2.color565(140 + (uint8_t)(f * 80), 120 + (uint8_t)(f * 60), 0);
      else col = disp2.color565(140 + (uint8_t)(f * 80), 0, 0);
      disp2.drawFastHLine(bx + innerMargin, fillY + y, innerW, col);
    }

    // Percentage text
    if (pct != prevSigPct) {
      disp2.fillRect(bx + innerMargin, by + bh / 2 - 8, innerW, 20, TFT_BLACK);
      // Redraw fill behind text
      int txtTop = by + bh / 2 - 8, txtBot = txtTop + 20;
      for (int y = max(fillY, txtTop); y < min(fillY + fillH, txtBot); y++) {
        float f = (float)(y - fillY) / max(1, fillH);
        uint16_t col;
        if (battLevel > 50) col = disp2.color565(0, 140 + (uint8_t)(f * 80), 0);
        else if (battLevel > 20) col = disp2.color565(140 + (uint8_t)(f * 80), 120 + (uint8_t)(f * 60), 0);
        else col = disp2.color565(140 + (uint8_t)(f * 80), 0, 0);
        disp2.drawFastHLine(bx + innerMargin, y, innerW, col);
      }

      disp2.setTextSize(2);
      char pctTxt[6];
      snprintf(pctTxt, sizeof(pctTxt), "%d%%", pct);
      int ptw = strlen(pctTxt) * 12;
      disp2.setTextColor(TFT_WHITE);
      disp2.setCursor(bx + (bw - ptw) / 2, by + bh / 2 - 6);
      disp2.print(pctTxt);
    }

    disp2.endWrite();
    prevSigFillH = fillH;
    prevSigPct = pct;
  }
}

// ─── D2 Media player screen ──────────────────────────────────────────────────

const char* songNames[] = {
  "Bohemian Rhapsody", "Hotel California", "Stairway to Heaven",
  "Back in Black", "Sweet Child O'Mine", "Comfortably Numb",
  "Wish You Were Here", "Paranoid Android", "Smells Like Teen Spirit"
};
const char* artistNames[] = {
  "Queen", "Eagles", "Led Zeppelin",
  "AC/DC", "Guns N' Roses", "Pink Floyd",
  "Pink Floyd", "Radiohead", "Nirvana"
};
const int songCount = 9;

bool mediaDrawn = false;
int mediaSongIdx = 0;
bool mediaPlaying = true;
unsigned long lastSongChange = 0;
// Frequency band heights (smoothed)
float freqBands[32] = {0};
float freqTargets[32] = {0};

// Update media state + EQ bands (called every frame from loop, regardless of active screen)
void updateMediaState() {
  unsigned long now = millis();

  // Auto-change song every 15-25 seconds
  if (now - lastSongChange > 15000 + random(10000)) {
    mediaSongIdx = (mediaSongIdx + 1) % songCount;
    lastSongChange = now;
    mediaDrawn = false;
  }

  // Toggle play/pause occasionally
  if (random(500) == 0) {
    mediaPlaying = !mediaPlaying;
    mediaDrawn = false;
  }

  // Update EQ targets + smooth
  if (mediaPlaying) {
    if (random(3) == 0) {
      for (int i = 0; i < 32; i++) {
        float bassBoost = 1.0f - (float)i / 32 * 0.5f;
        freqTargets[i] = random(20, (int)(120 * bassBoost * 0.9f));
      }
    }
  } else {
    for (int i = 0; i < 32; i++) freqTargets[i] = 0;
  }
  for (int i = 0; i < 32; i++) {
    freqBands[i] += (freqTargets[i] - freqBands[i]) * 0.25f;
    if (freqBands[i] < 1) freqBands[i] = 0;
  }
}

void renderD2Media() {
  unsigned long now = millis();

  // Draw static parts once
  if (!mediaDrawn) {
    disp2.startWrite();
    disp2.fillScreen(TFT_BLACK);

    // Album art placeholder (colored square)
    int artX = (172 - 80) / 2, artY = 18, artS = 80;
    uint16_t artCol = disp2.color565(
      40 + (mediaSongIdx * 23) % 80,
      30 + (mediaSongIdx * 37) % 60,
      50 + (mediaSongIdx * 41) % 90
    );
    disp2.fillRoundRect(artX, artY, artS, artS, 8, artCol);
    // Music note icon
    disp2.fillCircle(artX + 30, artY + 52, 8, 0xFFFF);
    disp2.fillCircle(artX + 50, artY + 46, 8, 0xFFFF);
    disp2.fillRect(artX + 37, artY + 18, 3, 36, 0xFFFF);
    disp2.fillRect(artX + 57, artY + 12, 3, 36, 0xFFFF);
    disp2.drawFastHLine(artX + 37, artY + 18, 23, 0xFFFF);
    disp2.drawFastHLine(artX + 37, artY + 19, 23, 0xFFFF);

    // Song name
    disp2.setTextSize(1);
    disp2.setTextColor(TFT_WHITE);
    const char* song = songNames[mediaSongIdx];
    int sw = strlen(song) * 6;
    disp2.setCursor(max(0, (172 - sw) / 2), 108);
    disp2.print(song);

    // Artist
    disp2.setTextColor(0x8410);
    const char* artist = artistNames[mediaSongIdx];
    int aw = strlen(artist) * 6;
    disp2.setCursor(max(0, (172 - aw) / 2), 122);
    disp2.print(artist);

    // Progress bar background
    disp2.drawRoundRect(12, 140, 148, 6, 3, 0x4208);

    // Play/pause and controls
    int cy = 158;
    // Prev icon (<<)
    disp2.fillTriangle(42, cy + 8, 52, cy, 52, cy + 16, 0x8410);
    disp2.fillTriangle(30, cy + 8, 40, cy, 40, cy + 16, 0x8410);
    // Next icon (>>)
    disp2.fillTriangle(130, cy + 8, 120, cy, 120, cy + 16, 0x8410);
    disp2.fillTriangle(142, cy + 8, 132, cy, 132, cy + 16, 0x8410);

    disp2.endWrite();
    mediaDrawn = true;
    // Reset freq bands
    for (int i = 0; i < 32; i++) { freqBands[i] = 0; freqTargets[i] = 0; }
  }

  disp2.startWrite();

  // Progress bar (animated)
  float progress = fmodf(now * 0.00003f, 1.0f);
  int progW = (int)(progress * 144);
  disp2.fillRoundRect(14, 142, progW, 2, 1, TFT_CYAN);

  // Play/pause button (center, update each frame)
  int cy = 158;
  disp2.fillRect(76, cy - 2, 20, 20, TFT_BLACK);
  if (mediaPlaying) {
    // Pause icon (two bars)
    disp2.fillRect(79, cy, 5, 16, TFT_WHITE);
    disp2.fillRect(88, cy, 5, 16, TFT_WHITE);
  } else {
    // Play icon (triangle)
    disp2.fillTriangle(80, cy, 80, cy + 16, 94, cy + 8, TFT_WHITE);
  }

  // ── Frequency visualizer (y=185..310) ──
  int vizX = 6, vizY = 190, vizH = 120, vizW = 160;
  int bandCount = 32;
  int bandW = vizW / bandCount;  // 5px per band

  // Draw bands (EQ updated in updateMediaState())
  for (int i = 0; i < bandCount; i++) {
    int bx = vizX + i * bandW;
    int bh = (int)freqBands[i];
    int by = vizY + vizH - bh;

    // Clear above band
    disp2.fillRect(bx, vizY, bandW - 1, vizH - bh, TFT_BLACK);

    // Draw band with color gradient (blue bottom → cyan → green top)
    for (int y = 0; y < bh; y++) {
      float f = (float)y / max(1, vizH);
      uint8_t r = (uint8_t)(f * 50);
      uint8_t g = (uint8_t)(100 + f * 155);
      uint8_t b = (uint8_t)(255 - f * 200);
      disp2.drawFastHLine(bx, by + (bh - 1 - y), bandW - 1, disp2.color565(r, g, b));
    }
  }

  disp2.endWrite();
}

// ─── D2 Debug screen ─────────────────────────────────────────────────────────

static bool debugDrawn = false;

void renderD2Debug() {
  int lh = 11, lx = 6, vx = 42;  // label x, value x

  // Static parts: draw once
  if (!debugDrawn) {
    disp2.startWrite();
    disp2.fillScreen(TFT_BLACK);
    disp2.setTextSize(1);

    // Title
    disp2.setTextColor(TFT_CYAN);
    disp2.setCursor(50, 4);
    disp2.print("DEBUG INFO");
    disp2.drawFastHLine(4, 14, 164, 0x4208);

    int y = 18;
    disp2.setTextColor(0x4208);
    const char* labels[] = {
      "SRC","SEQ", NULL,
      "SPD","THR","RPM","MTR", NULL,
      "LAT","LON","ALT","FIX", NULL,
      "HDG","CRS","HOME", NULL,
      "BATT","CUR", NULL,
      "RSSI","LQ","RX", NULL,
      "ACC","GYRO","TEMP","PRES", NULL,
      "HEAP","UP"
    };
    for (int i = 0; i < (int)(sizeof(labels)/sizeof(labels[0])); i++) {
      if (labels[i] == NULL) {
        disp2.drawFastHLine(4, y, 164, 0x2104);
        y += 3;
      } else {
        disp2.setCursor(lx, y);
        disp2.print(labels[i]);
        y += lh;
      }
    }

    disp2.endWrite();
    debugDrawn = true;
  }

  // Dynamic values only
  disp2.startWrite();
  disp2.setTextSize(1);

  int y = 18;
  char buf[22];
  // Helper: clear value area and print
  auto pv = [&](uint16_t c, const char* s) {
    disp2.fillRect(vx, y, 168 - vx, lh, TFT_BLACK);
    disp2.setTextColor(c);
    disp2.setCursor(vx, y);
    disp2.print(s);
    y += lh;
  };
  auto sep = [&]() { y += 3; };

  // SRC
  pv(liveData.masterPresent ? TFT_GREEN : TFT_YELLOW, liveData.masterPresent ? "I2C LIVE" : "SIMULATION");
  snprintf(buf, sizeof(buf), "%d", liveData.lastSeq); pv(TFT_WHITE, buf);
  sep();

  // Drive
  snprintf(buf, sizeof(buf), "%.1f km/h", liveData.speed_kmh); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%d%%", (int)liveData.throttle_pct); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%d", liveData.rpm); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "0x%02X s=%d", liveData.motor_state, liveData.steer_pos); pv(TFT_WHITE, buf);
  sep();

  // GPS
  snprintf(buf, sizeof(buf), "%.6f", (float)liveData.latitude); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.6f", (float)liveData.longitude); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.1fm s=%d", liveData.altitude_m, liveData.satellites); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%s", liveData.fix_type==2?"3D":liveData.fix_type==1?"2D":"NONE");
  pv(liveData.fix_type >= 2 ? TFT_GREEN : TFT_RED, buf);
  sep();

  // Nav
  snprintf(buf, sizeof(buf), "%.1f/%.1f", liveData.heading_deg, liveData.heading_mag_deg); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.1f", liveData.course_deg); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%lum", liveData.distance_home_m); pv(TFT_WHITE, buf);
  sep();

  // Battery
  snprintf(buf, sizeof(buf), "%.2fV %d%%", liveData.voltage, liveData.batt_pct);
  pv(liveData.batt_pct > 20 ? TFT_GREEN : TFT_RED, buf);
  snprintf(buf, sizeof(buf), "%.1fA %s%s", liveData.current_a, liveData.charging?"CHG ":"", liveData.low_warning?"LOW":"");
  pv(TFT_WHITE, buf);
  sep();

  // Link
  snprintf(buf, sizeof(buf), "%d dBm", liveData.rssi_dbm);
  pv(liveData.rssi_dbm > -65 ? TFT_GREEN : liveData.rssi_dbm > -80 ? TFT_YELLOW : TFT_RED, buf);
  snprintf(buf, sizeof(buf), "%d%% b=%d", liveData.link_quality, liveData.signal_bars); pv(TFT_WHITE, buf);
  pv(liveData.rx_connected ? TFT_GREEN : TFT_RED, liveData.rx_connected ? "CONNECTED" : "LOST");
  sep();

  // Sensor
  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", liveData.accel_x, liveData.accel_y, liveData.accel_z); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.1f d/s", liveData.gyro_z); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.1fC", liveData.temperature); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%.0f Pa", liveData.pressure_pa); pv(TFT_WHITE, buf);
  sep();

  // System
  snprintf(buf, sizeof(buf), "%dK/%dK", ESP.getFreeHeap()/1024, ESP.getMaxAllocHeap()/1024); pv(TFT_WHITE, buf);
  snprintf(buf, sizeof(buf), "%lus m=%us", millis()/1000, liveData.megaUptimeSec); pv(TFT_WHITE, buf);

  disp2.endWrite();
}

void renderD2Placeholder(const char* title) {
  disp2.startWrite();
  disp2.fillScreen(TFT_BLACK);
  disp2.setTextColor(0x4208);
  disp2.setTextSize(2);
  int tw = strlen(title) * 12;
  disp2.setCursor((172 - tw) / 2, 150);
  disp2.print(title);
  disp2.endWrite();
}

// ─── Screen dispatchers ──────────────────────────────────────────────────────

void renderD1() {
  // Clear on screen change
  if (d1Screen != d1PrevScreen) {
    sprite1.fillScreen(TFT_BLACK);
    sprite1.pushSprite(0, 0);
    d1PrevScreen = d1Screen;
  }

  // Menu takes over D1 when active
  if (menuIsActive()) {
    renderD1Menu();
    return;
  }

  switch (d1Screen) {
    case D1_DASHBOARD:   renderNavInfo(); break;
    case D1_NAVIGATION:  renderD1Navigation(); break;
    case D1_MEDIA:       renderD1Media(); break;
    case D1_ABOUT:       renderD1About(); break;
    case D1_IMU:         renderD1IMU(); break;
    default: break;
  }
}

void renderD2() {
  // Clear on screen change
  if (d2Screen != d2PrevScreen) {
    disp2.startWrite();
    disp2.fillScreen(TFT_BLACK);
    disp2.endWrite();
    navBottomInit = false;
    battDrawn = false;
    sigBattDrawn = false;
    mediaDrawn = false;
    debugDrawn = false;
    d2PrevScreen = d2Screen;
  }

  switch (d2Screen) {
    case D2_NAVIGATION:  renderD2Navigation(); break;
    case D2_MAP:         renderMap(); break;
    case D2_BATTERY:     renderD2Battery(); break;
    case D2_MEDIA:       renderD2Media(); break;
    case D2_SIGNAL:      renderD2SignalBattery(); break;
    case D2_DEBUG:       renderD2Debug(); break;
    default: break;
  }
}

// ─── Web download processor (one tile per call) ─────────────────────────────

void processOneWebDownload() {
  snprintf(dlJob.current, sizeof(dlJob.current), "%d_%d_%d",
           dlJob.z, dlJob.curX, dlJob.curY);

  char path[48];
  snprintf(path, sizeof(path), "/tiles/%d_%d_%d.png", dlJob.z, dlJob.curX, dlJob.curY);

  sdAcquire();
  bool exists = SD.exists(path);
  sdRelease();

  if (exists) {
    Serial.printf("Web DL skip: %s\n", dlJob.current);
  } else {
    Serial.printf("Web DL: %s (heap=%d, largest=%d)\n", dlJob.current,
                   ESP.getFreeHeap(), ESP.getMaxAllocHeap());
    char url[128];
    snprintf(url, sizeof(url), "https://tile.openstreetmap.org/%d/%d/%d.png",
             dlJob.z, dlJob.curX, dlJob.curY);

    // Free sprite1 temporarily for SSL buffer space
    sprite1.deleteSprite();

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(10);

    HTTPClient http;
    http.begin(client, url);
    http.addHeader("User-Agent", "ESP32-NavDemo/1.0 (hobby)");
    http.setTimeout(10000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();

    if (code == 200) {
      sdAcquire();
      File f = SD.open(path, FILE_WRITE);
      if (f) {
        int total = http.getSize();
        WiFiClient* stream = http.getStreamPtr();
        uint8_t buf[512];
        int written = 0;
        unsigned long dlTimeout = millis() + 15000;
        while (http.connected() && millis() < dlTimeout) {
          int avail = stream->available();
          if (avail > 0) {
            int len = stream->readBytes(buf, min((int)sizeof(buf), avail));
            f.write(buf, len);
            written += len;
            if (total > 0 && written >= total) break;
          } else if (total > 0 && written >= total) break;
          else { webServer.handleClient(); delay(1); }
        }
        f.close();
        Serial.printf("  saved %d bytes\n", written);
      }
      sdRelease();
    } else {
      Serial.printf("  HTTP %d (heap=%d)\n", code, ESP.getFreeHeap());
      dlJob.errors++;
    }
    http.end();

    // Recreate sprite1 after SSL freed
    if (!sprite1.getBuffer()) {
      sprite1.setColorDepth(16);
      sprite1.createSprite(240, 135);
    }
    // OSM rate limit — keep web server responsive during the wait
    { unsigned long until = millis() + 600; while (millis() < until) { webServer.handleClient(); delay(10); } }
  }

  dlJob.done++;
  dlJob.curX++;
  if (dlJob.curX > dlJob.x1) {
    dlJob.curX = dlJob.x0;
    dlJob.curY++;
  }
  if (dlJob.curY > dlJob.y1 || dlJob.cancel) {
    dlJob.active = false;
    Serial.printf("Web DL done: %d/%d, %d errors\n", dlJob.done, dlJob.total, dlJob.errors);
    if (!dlJob.cancel) scanTileGrid();  // update grid bounds + save cache
  }
}

// ─── Boot animation ──────────────────────────────────────────────────────────

static const char* bootLines[] = {
  "[    0.000000] Linux version 6.1.0-rc-car-esp32 (gcc 12.2.0)",
  "[    0.000000] Machine: ESP32-NAV v1.1",
  "[    0.010000] Memory: 320K/320K available",
  "[    0.020000] SLUB: HWalign=64, Order=0-3, MinObjects=0",
  "[    0.030000] NR_IRQS: 32, nr_irqs: 32",
  "[    0.040000] sched_clock: 32 bits at 240MHz",
  "[    0.050000] Console: colour TFT0 135x240",
  "[    0.060000] Calibrating delay loop... 480.00 BogoMIPS",
  "[    0.100000] pid_max: default: 32768 minimum: 301",
  "[    0.110000] Mount-cache hash table entries: 512",
  "[    0.120000] CPU: ESP32-D0WDQ6-V3 rev3 @ 240MHz",
  "[    0.130000] spi spi0.0: ST7789V TFT 135x240 ready",
  "[    0.140000] spi spi1.0: ST7789V2 TFT 172x320 ready",
  "[    0.150000] mmc0: new SD card at address 0001",
  "[    0.160000] mmcblk0: mmc0:0001 SD 32.0 GiB",
  "[    0.170000]  mmcblk0: p1",
  "[    0.180000] EXT4-fs (mmcblk0p1): mounted filesystem",
  "[    0.200000] NET: Registered protocol family 2",
  "[    0.210000] wlan0: IEEE 802.11bgn ready",
  "[    0.230000] input: gpio-keys as /devices/gpio-keys",
  "[    0.250000] display: backlight0 at pwm channel 0",
  "[    0.260000] display: backlight1 at pwm channel 1",
  "[    0.280000] nav: tile cache at /mnt/sd/tiles/",
  "[    0.300000] nav: GPS simulation module loaded",
  "[    0.320000] nav: web interface on port 80",
  "[    0.350000] systemd[1]: Started GPS Navigation Service",
  "[    0.380000] systemd[1]: Reached target Multi-User System",
  "[  OK  ] Started GPS Navigation Daemon",
  "[  OK  ] Reached target graphical.target",
};
static const int bootLineCount = sizeof(bootLines) / sizeof(bootLines[0]);

void bootAnimation() {
  // Green-on-black terminal style for both displays
  unsigned long d1End = millis() + 4000 + random(3000);  // 4-7s
  unsigned long d2End = millis() + 4000 + random(3000);

  int d1Line = 0, d2Line = 0;
  int d1Y = 0, d2Y = 0;
  int lineH = 10;  // line height in pixels
  int d1MaxLines = 135 / lineH;  // ~13 lines on D1
  int d2MaxLines = 320 / lineH;  // ~32 lines on D2

  disp1.fillScreen(TFT_BLACK);
  disp2.startWrite();
  disp2.fillScreen(TFT_BLACK);
  disp2.endWrite();

  unsigned long lastD1 = 0, lastD2 = 0;
  bool d1Done = false, d2Done = false;

  while (!d1Done || !d2Done) {
    unsigned long now = millis();

    // D1: type out lines with random timing
    if (!d1Done && now - lastD1 > (80 + random(200))) {
      lastD1 = now;
      if (now >= d1End || d1Line >= bootLineCount) {
        d1Done = true;
      } else {
        if (d1Y >= 135 - lineH) {
          // Scroll: shift display up (clear and restart from top area)
          sprite1.fillScreen(TFT_BLACK);
          d1Y = 0;
        }
        sprite1.setTextColor(0x07E0);  // green
        sprite1.setTextSize(1);
        sprite1.setCursor(0, d1Y);
        // Truncate long lines for 240px width (40 chars)
        char buf[41];
        strncpy(buf, bootLines[d1Line % bootLineCount], 40);
        buf[40] = '\0';
        sprite1.print(buf);
        sprite1.pushSprite(0, 0);
        d1Y += lineH;
        d1Line++;
      }
    }

    // D2: same but direct to display
    if (!d2Done && now - lastD2 > (60 + random(180))) {
      lastD2 = now;
      if (now >= d2End || d2Line >= bootLineCount) {
        d2Done = true;
      } else {
        if (d2Y >= 320 - lineH) {
          disp2.startWrite();
          disp2.fillScreen(TFT_BLACK);
          disp2.endWrite();
          d2Y = 0;
        }
        disp2.startWrite();
        disp2.setTextColor(0x07E0);  // green
        disp2.setTextSize(1);
        disp2.setCursor(0, d2Y);
        char buf[30];  // 172px / 6 = ~28 chars
        strncpy(buf, bootLines[d2Line % bootLineCount], 28);
        buf[28] = '\0';
        disp2.print(buf);
        disp2.endWrite();
        d2Y += lineH;
        d2Line++;
      }
    }

    delay(5);
  }

  // Brief pause then clear
  delay(300);
  disp1.fillScreen(TFT_BLACK);
  disp2.startWrite();
  disp2.fillScreen(TFT_BLACK);
  disp2.endWrite();
}

// ─── Serial command processor (for testing) ─────────────────────────────────
// Commands:
//   mo        - open menu
//   mc        - close menu
//   mn        - menu next
//   mp        - menu prev
//   me        - menu enter/confirm
//   mb        - menu back
//   ns        - start/restart navigation
//   np        - preview selected track
//   d1 <n>    - switch D1 screen
//   d2 <n>    - switch D2 screen

void processSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("> %s\n", cmd.c_str());

  if (cmd == "mo") {
    menuOpen();
  } else if (cmd == "mc") {
    menuClose();
  } else if (cmd == "mn") {
    menuNext();
  } else if (cmd == "mp") {
    menuPrev();
  } else if (cmd == "me") {
    menuEnter();
  } else if (cmd == "mb") {
    menuBack();
  } else if (cmd == "ns") {
    startNavigation();
  } else if (cmd == "np") {
    if (menu.previewMode) stopPreview();
    else startPreview();
  } else if (cmd.startsWith("d1 ")) {
    int scr = cmd.substring(3).toInt();
    if (scr >= 0 && scr < D1_SCREEN_COUNT) {
      d1Screen = (D1Screen)scr;
      saveScreenState();
      Serial.printf("D1 -> %s\n", d1ScreenNames[scr]);
    }
  } else if (cmd.startsWith("d2 ")) {
    int scr = cmd.substring(3).toInt();
    if (scr >= 0 && scr < D2_SCREEN_COUNT) {
      d2Screen = (D2Screen)scr;
      saveScreenState();
      Serial.printf("D2 -> %s\n", d2ScreenNames[scr]);
    }
  } else if (cmd == "help") {
    Serial.println("Commands: mo mc mn mp me mb ns np d1<n> d2<n> help");
  } else {
    Serial.printf("Unknown: %s (type 'help')\n", cmd.c_str());
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println("GPS Nav Demo starting...");

  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(BTN_USER, INPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  hspi.begin(27, SD_MISO, 13, SD_CS);
  digitalWrite(SD_CS, HIGH);

  // Init displays
  disp1.init();
  disp1.setRotation(3);
  disp1.setBrightness(200);
  disp1.fillScreen(TFT_BLACK);

  disp2.init();
  disp2.setRotation(0);
  disp2.setBrightness(200);
  disp2.fillScreen(TFT_BLACK);

  sprite1.setColorDepth(16);
  if (!sprite1.createSprite(240, 135)) {
    Serial.println("sprite1 alloc failed!");
  }

  // Boot animation
  bootAnimation();

  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  disp2.endWrite();
  digitalWrite(15, HIGH);

  bool sdOK = false;
  for (int spd : {4000000, 10000000}) {
    if (SD.begin(SD_CS, hspi, spd)) {
      if (SD.cardType() != CARD_NONE) {
        sdOK = true;
        Serial.printf("SD OK at %dHz, %lluMB\n", spd, SD.cardSize() / 1048576);
        break;
      }
      SD.end();
    }
    SD.end();
    delay(100);
  }

  if (!sdOK) {
    showD1("SD CARD FAILED!", "", "Check wiring:", "MOSI=13 SCK=27", "MISO=21 CS=33");
    showD2("SD FAILED");
    while (1) delay(1000);
  }

  SD.mkdir("/tiles");
  SD.mkdir("/routes");
  loadScreenState();
  setupI2CSlave();
  servoSetup();
  loadActiveRoute();

  // Init simulation — load grid from cache, fall back to full scan
  if (!loadTileGridCache()) scanTileGrid();
  simX = MAP_PX_W / 2.0f;
  simY = MAP_PX_H / 2.0f;
  simHeading = 0.5f;
  simSpeed = 2.0f;
  randomSeed(analogRead(0));

  // ── Check BTN2 at boot: hold → web config mode ──
  delay(300);
  if (digitalRead(BTN_USER) == LOW) {
    Serial.println("BTN2 held — entering WEB CONFIG mode");
    showD1("WEB CONFIG MODE", "", "Starting AP+WiFi...");
    showD2("Web Config");

    // AP+STA: own access point + internet via home WiFi
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("GPSNav", "gpsnavconfig");  // AP: SSID=GPSNav, pass=gpsnavconfig
    delay(200);
    String apIP = WiFi.softAPIP().toString();
    Serial.printf("AP started: GPSNav @ %s\n", apIP.c_str());

    // Also connect to home WiFi for internet (tile downloads)
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    showD1("WEB CONFIG", "", ("AP: " + apIP).c_str(), "", "Connecting WiFi...");

    int tries = 0;
    bool staConnected = false;
    while (tries < 30) {
      delay(500);
      tries++;
      if (WiFi.status() == WL_CONNECTED) { staConnected = true; break; }
    }

    setupWebServer();
    webServerStarted = true;
    wifiActive = true;

    // Show both IPs
    char line1[40], line2[40], line3[40], line5[40];
    snprintf(line1, sizeof(line1), "AP: GPSNav");
    snprintf(line2, sizeof(line2), " -> %s", apIP.c_str());
    if (staConnected) {
      String staIP = WiFi.localIP().toString();
      snprintf(line3, sizeof(line3), "WiFi: %s", staIP.c_str());
      Serial.printf("STA: %s  AP: %s\n", staIP.c_str(), apIP.c_str());
    } else {
      snprintf(line3, sizeof(line3), "WiFi: not connected");
      Serial.println("STA failed, AP-only mode");
    }
    snprintf(line5, sizeof(line5), "Reset to navigate");
    showD1("WEB CONFIG", line1, line2, line3, "", line5);
    showD2("Web Config");
    appState = STATE_WEBSERVER;
    return;
  }

  // ── Normal boot: navigate ──
  Serial.println("Starting navigation. Hold BTN2 on boot for web config.");
  showD1("GPS Nav Demo", "", "Starting nav...");
  delay(1000);
  appState = STATE_NAVIGATE;
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
  switch (appState) {

  // ── Navigate ───────────────────────────────────────────────────────────────
  case STATE_NAVIGATE:
    if (!navInitDone) {
      Serial.printf("Nav init. Free heap: %d, largest: %d\n",
                     ESP.getFreeHeap(), ESP.getMaxAllocHeap());
      navInitDone = true;
    }
    checkScreenButton();
    parseI2CMessage();
    checkI2CTimeout();
    processSerialCommands();

    // Menu navigation via I2C steering/throttle
    if (menuIsActive() && liveData.masterPresent) {
      static unsigned long lastSteerAction = 0;
      static bool throttleWasHigh = false;
      // Steer left (< 80) / right (> 175) to browse
      if (millis() - lastSteerAction > 400) {
        if (liveData.steer_pos < 80) { menuPrev(); lastSteerAction = millis(); }
        else if (liveData.steer_pos > 175) { menuNext(); lastSteerAction = millis(); }
      }
      // Throttle to full (>90%) then back to 0 (<10%) to confirm/enter
      if (liveData.throttle_pct > 90) throttleWasHigh = true;
      if (throttleWasHigh && liveData.throttle_pct < 10) {
        throttleWasHigh = false;
        menuEnter();
      }
    }

    if (!navRoute.waitingForGPS) updateSim();
    updateMediaState();

    // Navigation update at 2Hz
    { static unsigned long lastNavUpdate = 0;
      if (millis() - lastNavUpdate > 500) {
        lastNavUpdate = millis();
        int32_t gpsLat = (int32_t)(liveData.latitude * 1e7);
        int32_t gpsLon = (int32_t)(liveData.longitude * 1e7);
        updateNavigation(gpsLat, gpsLon);
      }
    }

    // Populate LiveData: from I2C master if present, otherwise from simulation
    if (!liveData.masterPresent && !navRoute.waitingForGPS) {
      updateLiveDataFromSim();
    }

    // Preview overrides liveData position/heading AFTER sim (takes priority)
    if (menu.previewMode) {
      updatePreview();
    }

    servoUpdate();
    renderD2();
    renderD1();
    break;

  // ── Web config mode ────────────────────────────────────────────────────────
  case STATE_WEBSERVER:
    webServer.handleClient();

    // Process web-triggered tile downloads
    if (dlJob.active && !dlJob.cancel) {
      processOneWebDownload();
      // Update D1 with progress
      char prog[32];
      snprintf(prog, sizeof(prog), "DL %d/%d", dlJob.done, dlJob.total);
      String ip = WiFi.localIP().toString();
      char line2[32];
      snprintf(line2, sizeof(line2), "http://%s", ip.c_str());
      showD1("WEB CONFIG", "", line2, "", prog);
    }

    delay(5);
    break;

  default:
    break;
  }
}
