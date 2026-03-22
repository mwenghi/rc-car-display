#pragma once
#include <SD.h>

// Route file format
#define RTE_MAGIC "RTE"
#define RTE_VERSION 1
#define MAX_ROUTE_POINTS 500
#define MAX_TURNS 100
#define OFF_ROUTE_THRESHOLD_M 30.0f
#define TURN_ANGLE_THRESHOLD 25.0f

// Forward declarations
extern void sdAcquire();
extern void sdRelease();
extern LGFX_Waveshare disp2;

// ─── Route data ──────────────────────────────────────────────────────────────

struct RoutePoint {
  int32_t lat;  // lat * 1e7
  int32_t lon;  // lon * 1e7
};

struct TurnPoint {
  uint16_t idx;
  int16_t angle;  // degrees, -180..180 (negative=left, positive=right)
};

struct NavRoute {
  RoutePoint* pts;       // dynamically allocated when route loads
  uint16_t count;
  float* cumDistM;       // dynamically allocated when route loads
  TurnPoint turns[MAX_TURNS];
  uint8_t turnCount;

  // Navigation state
  uint16_t currentIdx;       // nearest segment index
  float projDistM;           // distance along route to projection point
  float totalDistM;
  float remainingDistM;
  float distToNextTurnM;
  int8_t nextTurnDir;        // -1=left, 0=straight, 1=right
  float nextTurnAngle;
  float bearingToNextDeg;    // desired bearing
  float offRouteDistM;       // perpendicular distance from route
  bool offRoute;
  bool active;
  bool finished;
  bool waitingForGPS;        // waiting at start point for GPS match
  float startBearingDeg;     // bearing from GPS to start point
  float startDistM;          // distance from GPS to start point
  char name[16];

  // Flat-earth constants (computed once at load)
  float cosLat;              // cos of average latitude
  float mPerDegLat;          // meters per 1e-7 degree lat
  float mPerDegLon;          // meters per 1e-7 degree lon
};

NavRoute navRoute = {0};

// ─── Coordinate math (flat-earth) ────────────────────────────────────────────

float routePointDist(const RoutePoint& a, const RoutePoint& b, float mLat, float mLon) {
  float dy = (b.lat - a.lat) * mLat;
  float dx = (b.lon - a.lon) * mLon;
  return sqrtf(dx * dx + dy * dy);
}

float bearingDeg(const RoutePoint& from, const RoutePoint& to, float mLat, float mLon) {
  float dy = (to.lat - from.lat) * mLat;
  float dx = (to.lon - from.lon) * mLon;
  float b = atan2f(dx, dy) * 180.0f / PI;
  if (b < 0) b += 360.0f;
  return b;
}

// Point-to-segment distance and projection parameter t (0..1)
float pointToSegmentDist(int32_t px, int32_t py,
                          int32_t ax, int32_t ay, int32_t bx, int32_t by,
                          float mLat, float mLon, float& t) {
  float dxAB = (bx - ax) * mLon;
  float dyAB = (by - ay) * mLat;
  float dxAP = (px - ax) * mLon;
  float dyAP = (py - ay) * mLat;
  float lenSq = dxAB * dxAB + dyAB * dyAB;
  if (lenSq < 0.01f) { t = 0; return sqrtf(dxAP * dxAP + dyAP * dyAP); }
  t = (dxAP * dxAB + dyAP * dyAB) / lenSq;
  t = constrain(t, 0.0f, 1.0f);
  float projX = dxAP - t * dxAB;
  float projY = dyAP - t * dyAB;
  return sqrtf(projX * projX + projY * projY);
}

// ─── Route file I/O ──────────────────────────────────────────────────────────

void freeRoute() {
  if (navRoute.pts) { free(navRoute.pts); navRoute.pts = nullptr; }
  if (navRoute.cumDistM) { free(navRoute.cumDistM); navRoute.cumDistM = nullptr; }
  navRoute.active = false;
  navRoute.count = 0;
}

bool loadRoute(const char* filename) {
  freeRoute();  // free previous route

  char path[48];
  snprintf(path, sizeof(path), "/routes/%s", filename);

  sdAcquire();
  File f = SD.open(path, FILE_READ);
  if (!f) { sdRelease(); return false; }

  // Read header
  char magic[4] = {0};
  f.read((uint8_t*)magic, 4);
  if (memcmp(magic, RTE_MAGIC, 3) != 0) { f.close(); sdRelease(); return false; }

  uint8_t ver = f.read();
  if (ver != RTE_VERSION) { f.close(); sdRelease(); return false; }

  uint8_t flags = f.read();
  uint16_t count;
  f.read((uint8_t*)&count, 2);
  if (count > MAX_ROUTE_POINTS) count = MAX_ROUTE_POINTS;

  uint32_t totalDist;
  f.read((uint8_t*)&totalDist, 4);

  f.read((uint8_t*)navRoute.name, 16);
  navRoute.name[15] = '\0';

  // Allocate point arrays
  navRoute.pts = (RoutePoint*)malloc(count * sizeof(RoutePoint));
  navRoute.cumDistM = (float*)malloc(count * sizeof(float));
  if (!navRoute.pts || !navRoute.cumDistM) {
    Serial.println("Route malloc failed!");
    freeRoute();
    f.close();
    sdRelease();
    return false;
  }

  // Read points
  for (int i = 0; i < count; i++) {
    f.read((uint8_t*)&navRoute.pts[i], 8);
  }
  f.close();
  sdRelease();

  navRoute.count = count;
  navRoute.totalDistM = totalDist;
  navRoute.active = true;
  navRoute.finished = false;
  navRoute.currentIdx = 0;
  navRoute.offRoute = false;

  Serial.printf("Route loaded: %s (%d points)\n", navRoute.name, count);

  // Compute flat-earth constants from average latitude
  if (count > 0) {
    float avgLat = 0;
    for (int i = 0; i < count; i++) avgLat += navRoute.pts[i].lat;
    avgLat /= count;
    float latRad = avgLat / 1e7f * PI / 180.0f;
    navRoute.cosLat = cosf(latRad);
    navRoute.mPerDegLat = 0.0111320f;  // meters per 1e-7 degree lat
    navRoute.mPerDegLon = 0.0111320f * navRoute.cosLat;
  }

  // Compute cumulative distances
  navRoute.cumDistM[0] = 0;
  for (int i = 1; i < count; i++) {
    navRoute.cumDistM[i] = navRoute.cumDistM[i - 1] +
      routePointDist(navRoute.pts[i - 1], navRoute.pts[i], navRoute.mPerDegLat, navRoute.mPerDegLon);
  }
  navRoute.totalDistM = navRoute.cumDistM[count - 1];

  // Detect turns
  navRoute.turnCount = 0;
  for (int i = 1; i < count - 1 && navRoute.turnCount < MAX_TURNS; i++) {
    float b1 = bearingDeg(navRoute.pts[i - 1], navRoute.pts[i], navRoute.mPerDegLat, navRoute.mPerDegLon);
    float b2 = bearingDeg(navRoute.pts[i], navRoute.pts[i + 1], navRoute.mPerDegLat, navRoute.mPerDegLon);
    float delta = b2 - b1;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    if (fabsf(delta) > TURN_ANGLE_THRESHOLD) {
      navRoute.turns[navRoute.turnCount].idx = i;
      navRoute.turns[navRoute.turnCount].angle = (int16_t)delta;
      navRoute.turnCount++;
    }
  }

  Serial.printf("Route: %.0fm, %d turns\n", navRoute.totalDistM, navRoute.turnCount);
  return true;
}

bool saveRoute(const char* filename, const char* name, const RoutePoint* pts, uint16_t count) {
  char path[48];
  snprintf(path, sizeof(path), "/routes/%s", filename);

  sdAcquire();
  File f = SD.open(path, FILE_WRITE);
  if (!f) { sdRelease(); return false; }

  // Header
  f.write((uint8_t*)RTE_MAGIC, 4);
  uint8_t ver = RTE_VERSION;
  f.write(ver);
  uint8_t flags = 0;
  f.write(flags);
  f.write((uint8_t*)&count, 2);

  // Total distance placeholder (rewritten after points)
  uint32_t totalDist = 0;
  long distPos = f.position();
  f.write((uint8_t*)&totalDist, 4);

  // Name (16 bytes padded)
  char nameBuf[16] = {0};
  strncpy(nameBuf, name, 15);
  f.write((uint8_t*)nameBuf, 16);

  // Points
  f.write((uint8_t*)pts, count * 8);

  // Compute and write total distance
  // (simplified: use flat-earth from first point)
  float avgLat = pts[0].lat / 1e7f * PI / 180.0f;
  float mLat = 0.0111320f;
  float mLon = 0.0111320f * cosf(avgLat);
  float dist = 0;
  for (int i = 1; i < count; i++) {
    dist += routePointDist(pts[i - 1], pts[i], mLat, mLon);
  }
  totalDist = (uint32_t)dist;
  f.seek(distPos);
  f.write((uint8_t*)&totalDist, 4);

  f.close();
  sdRelease();

  Serial.printf("Route saved: %s (%d pts, %dm)\n", filename, count, totalDist);
  return true;
}

// ─── Navigation update (call at 2Hz) ────────────────────────────────────────

void updateNavigation(int32_t gpsLat, int32_t gpsLon) {
  if (!navRoute.active || navRoute.count < 2) return;

  float mLat = navRoute.mPerDegLat;
  float mLon = navRoute.mPerDegLon;

  // ── Waiting for GPS at start point ──
  if (navRoute.waitingForGPS) {
    RoutePoint gps = {gpsLat, gpsLon};
    RoutePoint start = navRoute.pts[0];
    float dx = (gps.lon - start.lon) * mLon;
    float dy = (gps.lat - start.lat) * mLat;
    navRoute.startDistM = sqrtf(dx * dx + dy * dy);
    navRoute.startBearingDeg = bearingDeg(gps, start, mLat, mLon);
    // Start when within 15 meters of start point
    if (navRoute.startDistM < 15.0f) {
      navRoute.waitingForGPS = false;
      Serial.println("Nav: GPS reached start — GO!");
    }
    return;
  }

  // Find nearest segment (search window around current position)
  int searchStart = max(0, (int)navRoute.currentIdx - 5);
  int searchEnd = min((int)navRoute.count - 2, (int)navRoute.currentIdx + 30);
  float bestDist = 1e9f;
  int bestIdx = navRoute.currentIdx;
  float bestT = 0;

  for (int i = searchStart; i <= searchEnd; i++) {
    float t;
    float d = pointToSegmentDist(gpsLon, gpsLat,
      navRoute.pts[i].lon, navRoute.pts[i].lat,
      navRoute.pts[i + 1].lon, navRoute.pts[i + 1].lat,
      mLat, mLon, t);
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
      bestT = t;
    }
  }

  navRoute.currentIdx = bestIdx;
  navRoute.offRouteDistM = bestDist;
  navRoute.offRoute = (bestDist > OFF_ROUTE_THRESHOLD_M);

  // Distance along route to projection point
  float segLen = navRoute.cumDistM[bestIdx + 1] - navRoute.cumDistM[bestIdx];
  navRoute.projDistM = navRoute.cumDistM[bestIdx] + bestT * segLen;
  navRoute.remainingDistM = navRoute.totalDistM - navRoute.projDistM;

  // Bearing to next waypoint (~50m ahead or next point)
  int targetIdx = bestIdx + 1;
  while (targetIdx < navRoute.count - 1 &&
         navRoute.cumDistM[targetIdx] - navRoute.projDistM < 50.0f) {
    targetIdx++;
  }
  RoutePoint gps = {gpsLat, gpsLon};
  navRoute.bearingToNextDeg = bearingDeg(gps, navRoute.pts[targetIdx], mLat, mLon);

  // Find next turn
  navRoute.nextTurnDir = 0;
  navRoute.nextTurnAngle = 0;
  navRoute.distToNextTurnM = navRoute.remainingDistM;

  for (int t = 0; t < navRoute.turnCount; t++) {
    if (navRoute.turns[t].idx > bestIdx) {
      navRoute.distToNextTurnM = navRoute.cumDistM[navRoute.turns[t].idx] - navRoute.projDistM;
      navRoute.nextTurnAngle = navRoute.turns[t].angle;
      navRoute.nextTurnDir = (navRoute.turns[t].angle > 0) ? 1 : -1;
      break;
    }
  }

  // Check if finished (within 10m of last point)
  RoutePoint lastPt = navRoute.pts[navRoute.count - 1];
  float distToEnd = routePointDist(gps, lastPt, mLat, mLon);
  if (distToEnd < 10.0f && navRoute.projDistM > navRoute.totalDistM * 0.8f) {
    navRoute.finished = true;
  }
}

// ─── Load active route on boot ───────────────────────────────────────────────

void loadActiveRoute() {
  sdAcquire();
  if (SD.exists("/routes/active.cfg")) {
    File f = SD.open("/routes/active.cfg", FILE_READ);
    if (f) {
      char filename[32] = {0};
      int len = f.readBytesUntil('\n', filename, sizeof(filename) - 1);
      filename[len] = '\0';
      // Trim whitespace
      while (len > 0 && (filename[len-1] == '\r' || filename[len-1] == ' ')) filename[--len] = '\0';
      f.close();
      sdRelease();
      if (len > 0) {
        Serial.printf("Active route: %s\n", filename);
        loadRoute(filename);
      }
      return;
    }
  }
  sdRelease();
}

void setActiveRoute(const char* filename) {
  sdAcquire();
  File f = SD.open("/routes/active.cfg", FILE_WRITE);
  if (f) {
    f.print(filename);
    f.close();
  }
  sdRelease();
}

// ─── Track selection mode ────────────────────────────────────────────────────

#define MAX_TRACK_LIST 20

// ─── 3-level menu system ─────────────────────────────────────────────────────
// Level 1 (top):      Screen Select, Navigation, [Exit]
// Level 2 (sub):      Display 1 / Track Select / [Back]
// Level 3 (items):    screen list / track list / [Back]

#define MENU_MAX_ITEMS 10  // max visible items per level

enum MenuLevel { MENU_CLOSED = 0, MENU_L1, MENU_L2, MENU_L3 };

// Top-level menu entries
enum MenuL1Item { ML1_SCREEN_SEL = 0, ML1_NAVIGATION, ML1_VEHICLE, ML1_COUNT };
static const char* menuL1Names[] = {"Screen Select", "Navigation", "Vehicle"};

// Screen Select sub-items
enum MenuL2Screen { ML2S_DISPLAY1 = 0, ML2S_DISPLAY2, ML2S_COUNT };
static const char* menuL2ScreenNames[] = {"Display 1", "Display 2"};

// Navigation sub-items
enum MenuL2Nav { ML2N_TRACK_SEL = 0, ML2N_RESTART_NAV, ML2N_STOP_NAV, ML2N_PREVIEW, ML2N_COUNT };
static const char* menuL2NavNames[] = {"Track Select", "(Re)Start Nav", "Stop Nav", "Track Preview"};
static const char* menuL2NavPreviewStop = "Stop Preview";

struct Menu {
  MenuLevel level;
  int8_t l1Idx;          // selected at level 1
  int8_t l2Idx;          // selected at level 2
  int8_t l3Idx;          // selected at level 3
  uint8_t l1Parent;      // which L1 opened L2
  uint8_t l2Parent;      // which L2 opened L3
  // Track data (loaded when entering track select L3)
  char filenames[MAX_TRACK_LIST][24];
  char names[MAX_TRACK_LIST][16];
  uint16_t distances[MAX_TRACK_LIST];
  uint8_t trackCount;
  // Preview
  bool previewMode;
  float previewProgress;
};

Menu menu = {MENU_CLOSED};

// Legacy: trackSel.active = menu is in track selection L3

// Scan /routes/ for .rte files
void scanTracks() {
  menu.trackCount = 0;
  sdAcquire();
  File dir = SD.open("/routes");
  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f && menu.trackCount < MAX_TRACK_LIST) {
      String fn = f.name();
      if (fn.endsWith(".rte")) {
        String basename = fn;
        int sl = basename.lastIndexOf('/');
        if (sl >= 0) basename = basename.substring(sl + 1);
        strncpy(menu.filenames[menu.trackCount], basename.c_str(), 23);

        f.seek(0);
        char magic[4]; f.read((uint8_t*)magic, 4);
        f.read(); f.read();
        uint16_t cnt; f.read((uint8_t*)&cnt, 2);
        uint32_t dist; f.read((uint8_t*)&dist, 4);
        char name[16]; f.read((uint8_t*)name, 16); name[15] = '\0';
        strncpy(menu.names[menu.trackCount], name, 15);
        menu.distances[menu.trackCount] = (uint16_t)(dist < 65535 ? dist : 65535);
        menu.trackCount++;
      }
      f = dir.openNextFile();
    }
    dir.close();
  }
  sdRelease();
  Serial.printf("Track scan: %d routes found\n", menu.trackCount);
}

// ── Menu navigation ─────────────────────────────────────────────────────────

bool menuIsActive() { return menu.level != MENU_CLOSED; }

void menuBack();   // forward declaration
void menuEnter();  // forward declaration

void menuOpen() {
  if (menu.level != MENU_CLOSED) {
    menuEnter();  // already open — act as enter
    return;
  }
  menu.level = MENU_L1;
  menu.l1Idx = 0;
  Serial.println("Menu opened");
}

void menuClose() {
  menu.level = MENU_CLOSED;
  Serial.println("Menu closed");
}

// Get count of real items (not including Back/Exit button)
static int menuItemCount() {
  switch (menu.level) {
    case MENU_L1: return ML1_COUNT;
    case MENU_L2:
      if (menu.l1Parent == ML1_SCREEN_SEL) return ML2S_COUNT;
      if (menu.l1Parent == ML1_NAVIGATION) return ML2N_COUNT;
      if (menu.l1Parent == ML1_VEHICLE) return vconfRootCount();
      return 0;
    case MENU_L3:
      if (menu.l1Parent == ML1_SCREEN_SEL) {
        if (menu.l2Parent == ML2S_DISPLAY1) return D1_SCREEN_COUNT;
        if (menu.l2Parent == ML2S_DISPLAY2) return D2_SCREEN_COUNT;
      }
      if (menu.l1Parent == ML1_NAVIGATION && menu.l2Parent == ML2N_TRACK_SEL)
        return menu.trackCount;
      if (menu.l1Parent == ML1_VEHICLE) {
        // L3 = children of a submenu item
        VConfItem* parent = vconfRootItem(menu.l2Parent);
        if (parent && parent->type == VCONF_SUBMENU)
          return vconfChildCount(parent->id);
      }
      return 0;
    default: return 0;
  }
}

// Total items including Back/Exit
static int menuTotalCount() { return menuItemCount() + 1; }

static int8_t* menuIdx() {
  switch (menu.level) {
    case MENU_L1: return &menu.l1Idx;
    case MENU_L2: return &menu.l2Idx;
    case MENU_L3: return &menu.l3Idx;
    default: return &menu.l1Idx;
  }
}

void menuNext() {
  if (menu.level == MENU_CLOSED) return;
  // Number editing: adjust value instead of scrolling
  if (vconf.editing) { vconfItemAdjust(+1); return; }
  int total = menuTotalCount();
  if (total == 0) return;
  int8_t* idx = menuIdx();
  *idx = (*idx + 1) % total;
}

void menuPrev() {
  if (menu.level == MENU_CLOSED) return;
  if (vconf.editing) { vconfItemAdjust(-1); return; }
  int total = menuTotalCount();
  if (total == 0) return;
  int8_t* idx = menuIdx();
  *idx = (*idx - 1 + total) % total;
}

// Forward declarations
void startNavigation();
void stopNavigation();
void startPreview();
void stopPreview();

void menuEnter() {
  if (menu.level == MENU_CLOSED) return;

  // Check if on Back/Exit item (last item = count)
  int cnt = menuItemCount();
  int8_t* idx = menuIdx();
  if (*idx >= cnt) {
    menuBack();
    return;
  }

  if (menu.level == MENU_L1) {
    menu.l1Parent = menu.l1Idx;
    menu.l2Idx = 0;
    menu.level = MENU_L2;
    Serial.printf("Menu L2: %s\n", menuL1Names[menu.l1Parent]);
    return;
  }

  if (menu.level == MENU_L2) {
    // Action items (no L3) — execute and close
    if (menu.l1Parent == ML1_NAVIGATION && menu.l2Idx == ML2N_RESTART_NAV) {
      if (menu.previewMode) stopPreview();
      startNavigation();
      menuClose();
      return;
    }
    if (menu.l1Parent == ML1_NAVIGATION && menu.l2Idx == ML2N_STOP_NAV) {
      stopNavigation();
      menuClose();
      return;
    }
    if (menu.l1Parent == ML1_NAVIGATION && menu.l2Idx == ML2N_PREVIEW) {
      if (menu.previewMode) stopPreview(); else startPreview();
      menuClose();
      return;
    }
    // Vehicle config items — interact in-place at L2
    if (menu.l1Parent == ML1_VEHICLE) {
      VConfItem* item = vconfRootItem(menu.l2Idx);
      if (item) {
        if (item->type == VCONF_SUBMENU) {
          // Enter submenu → L3
          menu.l2Parent = menu.l2Idx;
          menu.l3Idx = 0;
          menu.level = MENU_L3;
          return;
        }
        vconfItemEnter(item);
      }
      return;
    }

    // Sub-menu items — enter L3
    menu.l2Parent = menu.l2Idx;
    menu.l3Idx = 0;
    if (menu.l1Parent == ML1_NAVIGATION && menu.l2Parent == ML2N_TRACK_SEL) {
      scanTracks();
    }
    menu.level = MENU_L3;
    return;
  }

  if (menu.level == MENU_L3) {
    if (menu.l1Parent == ML1_SCREEN_SEL && menu.l2Parent == ML2S_DISPLAY1) {
      extern D1Screen d1Screen;
      extern void saveScreenState();
      extern const char* d1ScreenNames[];
      if (menu.l3Idx >= 0 && menu.l3Idx < D1_SCREEN_COUNT) {
        d1Screen = (D1Screen)menu.l3Idx;
        saveScreenState();
        Serial.printf("D1 -> %s\n", d1ScreenNames[menu.l3Idx]);
        menuClose();
      }
    } else if (menu.l1Parent == ML1_SCREEN_SEL && menu.l2Parent == ML2S_DISPLAY2) {
      extern D2Screen d2Screen;
      extern void saveScreenState();
      extern const char* d2ScreenNames[];
      if (menu.l3Idx >= 0 && menu.l3Idx < D2_SCREEN_COUNT) {
        d2Screen = (D2Screen)menu.l3Idx;
        saveScreenState();
        Serial.printf("D2 -> %s\n", d2ScreenNames[menu.l3Idx]);
        menuClose();
      }
    } else if (menu.l1Parent == ML1_NAVIGATION && menu.l2Parent == ML2N_TRACK_SEL) {
      if (menu.trackCount > 0 && menu.l3Idx < menu.trackCount) {
        const char* fname = menu.filenames[menu.l3Idx];
        Serial.printf("Track selected: %s\n", fname);
        setActiveRoute(fname);
        loadRoute(fname);
        menuClose();
      }
    } else if (menu.l1Parent == ML1_VEHICLE) {
      // L3 = children of a submenu — interact in-place
      VConfItem* parent = vconfRootItem(menu.l2Parent);
      if (parent && parent->type == VCONF_SUBMENU) {
        VConfItem* child = vconfChildItem(parent->id, menu.l3Idx);
        if (child) vconfItemEnter(child);
      }
    }
  }
}

void menuBack() {
  if (vconf.editing) { vconfCancelEdit(); return; }
  if (menu.level == MENU_L3) {
    menu.level = MENU_L2;
  } else if (menu.level == MENU_L2) {
    menu.level = MENU_L1;
  } else if (menu.level == MENU_L1) {
    menuClose();
  }
}

// Legacy wrappers for I2C/serial compatibility
void enterTrackSelection() { menuOpen(); }
void exitTrackSelection()  { menuClose(); }
void trackSelectNext()     { menuNext(); }
void trackSelectPrev()     { menuPrev(); }
void trackConfirmSelection() { menuEnter(); }

void startNavigation() {
  if (!navRoute.active || navRoute.count < 2) {
    Serial.println("No route loaded");
    return;
  }
  navRoute.currentIdx = 0;
  navRoute.finished = false;
  navRoute.projDistM = 0;
  navRoute.waitingForGPS = true;
  navRoute.startDistM = 9999;
  navRoute.startBearingDeg = 0;

  // Center map on first route point
  extern float simX, simY, simHeading;
  int n = 1 << TILE_ZOOM;
  float startLat = navRoute.pts[0].lat / 1e7;
  float startLon = navRoute.pts[0].lon / 1e7;
  float tileX = (startLon + 180.0f) / 360.0f * n;
  float latRad = startLat * PI / 180.0f;
  float tileY = (1.0f - logf(tanf(latRad) + 1.0f / cosf(latRad)) / PI) / 2.0f * n;
  simX = (tileX - GRID_X0) * TPX;
  simY = (tileY - GRID_Y0) * TPX;

  // Heading toward second point
  simHeading = bearingDeg(navRoute.pts[0], navRoute.pts[1],
                           navRoute.mPerDegLat, navRoute.mPerDegLon) * PI / 180.0f;

  Serial.printf("Nav started: waiting for GPS near start (%s)\n", navRoute.name);
}

void stopNavigation() {
  navRoute.waitingForGPS = false;
  navRoute.currentIdx = 0;
  navRoute.finished = true;
  navRoute.projDistM = 0;
  navRoute.remainingDistM = 0;
  Serial.println("Navigation stopped");
}

void startPreview() {
  if (menu.trackCount == 0 && !navRoute.active) {
    Serial.println("No track to preview");
    return;
  }
  // Load track if in menu track selection
  bool inTrackSel = (menu.level == MENU_L3 && menu.l1Parent == ML1_NAVIGATION
                     && menu.l2Parent == ML2N_TRACK_SEL);
  if (inTrackSel && menu.trackCount > 0) {
    const char* fname = menu.filenames[menu.l3Idx];
    loadRoute(fname);
  }
  if (!navRoute.active || navRoute.count < 2) {
    Serial.println("Route not loaded");
    return;
  }
  menu.previewMode = true;
  menu.previewProgress = 0;
  navRoute.currentIdx = 0;
  navRoute.finished = false;
  Serial.printf("Preview: %s (%d pts)\n", navRoute.name, navRoute.count);
}

void stopPreview() {
  menu.previewMode = false;
  Serial.println("Preview stopped");
}

void updatePreview() {
  if (!menu.previewMode || !navRoute.active || navRoute.count < 2) return;

  // Advance along route (faster for preview)
  menu.previewProgress += 0.008f;
  if (menu.previewProgress >= 1.0f) {
    menu.previewProgress = 0;  // loop
  }

  // Interpolate position between route points
  float fIdx = menu.previewProgress * (navRoute.count - 1);
  int idx = (int)fIdx;
  float frac = fIdx - idx;
  idx = constrain(idx, 0, navRoute.count - 2);

  int32_t lat = navRoute.pts[idx].lat + (int32_t)((navRoute.pts[idx+1].lat - navRoute.pts[idx].lat) * frac);
  int32_t lon = navRoute.pts[idx].lon + (int32_t)((navRoute.pts[idx+1].lon - navRoute.pts[idx].lon) * frac);

  // Override liveData with preview position
  extern LiveData liveData;
  liveData.latitude = lat / 1e7;
  liveData.longitude = lon / 1e7;

  // Compute heading from route direction
  if (idx < navRoute.count - 1) {
    liveData.heading_deg = bearingDeg(navRoute.pts[idx], navRoute.pts[idx+1],
                                       navRoute.mPerDegLat, navRoute.mPerDegLon);
  }

  // Simulated walking speed
  liveData.speed_kmh = 4.5f;

  // Steering from lookahead: compare current heading to bearing toward
  // a point ~2 segments ahead — like a driver looking at the road ahead
  {
    RoutePoint cur = {lat, lon};
    // Lookahead: 2 points ahead (or last point)
    int laIdx = constrain(idx + 2, 0, navRoute.count - 1);
    float bearAhead = bearingDeg(cur, navRoute.pts[laIdx],
                                  navRoute.mPerDegLat, navRoute.mPerDegLon);
    float hdg = liveData.heading_deg;
    float err = bearAhead - hdg;
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    // ±45° deviation = full lock
    float steer = constrain(err / 45.0f, -1.0f, 1.0f);
    liveData.steer_pos = (uint8_t)(128.0f + steer * 127.0f);
  }

  // Update simX/simY pixel coords so map viewport follows preview
  // Inverse of pixelToLatLon: convert lat/lon back to pixel position in tile grid
  extern float simX, simY, simHeading;
  int n = 1 << TILE_ZOOM;
  float tileX = (liveData.longitude + 180.0f) / 360.0f * n;
  float latRad = liveData.latitude * PI / 180.0f;
  float tileY = (1.0f - logf(tanf(latRad) + 1.0f / cosf(latRad)) / PI) / 2.0f * n;
  simX = (tileX - GRID_X0) * TPX;
  simY = (tileY - GRID_Y0) * TPX;
  simHeading = liveData.heading_deg * PI / 180.0f;
}

// ─── Menu renderer for D1 ────────────────────────────────────────────────────

extern LGFX_Sprite sprite1;

// Flags for which L2 items are actions (no L3 submenu)
static bool menuL2IsAction(int l1, int l2) {
  return (l1 == ML1_NAVIGATION && (l2 == ML2N_RESTART_NAV || l2 == ML2N_STOP_NAV || l2 == ML2N_PREVIEW));
}

// Helper: draw a menu list (2x font, 240x135 landscape)
// itemH=24px, title=18px, bottom bar=14px → 4 visible items
static void drawMenuList(const char* title, const char** items, int count,
                         int selIdx, const char* breadcrumb) {
  sprite1.fillScreen(TFT_BLACK);

  // Title bar (size 1 for breadcrumb + title)
  sprite1.fillRect(0, 0, 240, 16, 0x0841);
  sprite1.setTextSize(1);
  if (breadcrumb) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(4, 4);
    sprite1.print(breadcrumb);
    sprite1.setTextColor(TFT_CYAN);
    sprite1.print(" > ");
  } else {
    sprite1.setTextColor(TFT_CYAN);
    sprite1.setCursor(4, 4);
  }
  sprite1.print(title);

  // Preview indicator
  if (menu.previewMode) {
    if ((millis() / 400) % 2) sprite1.fillCircle(230, 8, 4, TFT_RED);
  }

  // Items area: y=18..120 (102px), item height=24, max 4 visible
  int totalItems = count + 1;  // +1 for Back/Exit
  sprite1.setTextSize(2);  // 12x16 chars

  if (count == 0 && menu.level == MENU_L3) {
    sprite1.setTextColor(0x8410);
    sprite1.setCursor(30, 50);
    sprite1.print("(empty)");
  }

  int maxVis = 4;
  int visCount = min(maxVis, totalItems);
  int startIdx = selIdx - visCount / 2;
  if (startIdx < 0) startIdx = 0;
  if (startIdx + visCount > totalItems) startIdx = totalItems - visCount;

  for (int i = 0; i < visCount; i++) {
    int idx = startIdx + i;
    int y = 18 + i * 24;
    bool selected = (idx == selIdx);

    if (selected) {
      sprite1.fillRect(2, y, 236, 22, 0x0841);
      sprite1.drawRect(2, y, 236, 22, TFT_CYAN);
      sprite1.setTextColor(TFT_WHITE);
    } else {
      sprite1.setTextColor(0x8410);
    }

    sprite1.setCursor(8, y + 3);
    if (idx < count) {
      sprite1.print(items[idx]);
      // Arrow ">" for submenu items, no arrow for actions
      bool hasSubmenu = (menu.level < MENU_L3) && !menuL2IsAction(menu.l1Parent, idx);
      if (hasSubmenu && menu.level != MENU_L1) {
        sprite1.setTextColor(selected ? 0x6B6D : 0x4208);
        sprite1.setCursor(224, y + 3);
        sprite1.print(">");
      }
      if (menu.level == MENU_L1) {
        sprite1.setTextColor(selected ? 0x6B6D : 0x4208);
        sprite1.setCursor(224, y + 3);
        sprite1.print(">");
      }
    } else {
      sprite1.setTextColor(selected ? TFT_ORANGE : 0x6B6D);
      sprite1.print(menu.level == MENU_L1 ? "< Exit" : "< Back");
    }
  }

  // Scroll indicators
  sprite1.setTextSize(1);
  if (startIdx > 0) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(116, 17);
    sprite1.print("^");
  }
  if (startIdx + visCount < totalItems) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(116, 115);
    sprite1.print("v");
  }

  // Bottom hint bar
  sprite1.fillRect(0, 122, 240, 13, 0x0841);
  sprite1.setTextColor(0x4A69);
  sprite1.setTextSize(1);
  sprite1.setCursor(20, 125);
  sprite1.print("UP/DN:browse  OK:select  BACK:up");

  sprite1.pushSprite(0, 0);
}

// Helper for L3 vehicle submenu child lookup
static uint8_t _vconfChildGroupId = 0;
static VConfItem* _vconfChildItemWrapper(int idx) {
  return vconfChildItem(_vconfChildGroupId, idx);
}

// Draw vehicle config item list (name + value on right)
// items come from vconfRootItem(idx) or vconfChildItem(groupId, idx)
static void drawVehicleList(const char* title, const char* breadcrumb,
                            int count, int selIdx,
                            VConfItem* (*getItem)(int)) {
  sprite1.fillScreen(TFT_BLACK);

  // Title bar
  sprite1.fillRect(0, 0, 240, 16, 0x0841);
  sprite1.setTextSize(1);
  if (breadcrumb) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(4, 4);
    sprite1.print(breadcrumb);
    sprite1.setTextColor(TFT_CYAN);
    sprite1.print(" > ");
  } else {
    sprite1.setTextColor(TFT_CYAN);
    sprite1.setCursor(4, 4);
  }
  sprite1.print(title);

  int totalItems = count + 1;  // +1 for Back
  int maxVis = 4;
  int visCount = min(maxVis, totalItems);
  int startIdx = selIdx - visCount / 2;
  if (startIdx < 0) startIdx = 0;
  if (startIdx + visCount > totalItems) startIdx = totalItems - visCount;

  if (count == 0) {
    sprite1.setTextSize(2);
    sprite1.setTextColor(0x8410);
    sprite1.setCursor(18, 50);
    sprite1.print("No items");
  }

  for (int i = 0; i < visCount; i++) {
    int idx = startIdx + i;
    int y = 18 + i * 24;
    bool selected = (idx == selIdx);

    if (selected) {
      sprite1.fillRect(2, y, 236, 22, 0x0841);
      sprite1.drawRect(2, y, 236, 22, TFT_CYAN);
    }

    sprite1.setTextSize(2);
    sprite1.setCursor(8, y + 3);

    if (idx < count) {
      VConfItem* item = getItem(idx);
      if (!item) continue;

      sprite1.setTextColor(selected ? TFT_WHITE : 0x8410);
      sprite1.print(item->name);

      // Value on right side
      sprite1.setTextSize(1);
      bool isEditing = (vconf.editing && vconf.editSlot == item->id);

      switch (item->type) {
        case VCONF_TOGGLE: {
          bool on = item->value != 0;
          sprite1.setTextColor(on ? TFT_GREEN : TFT_RED);
          sprite1.setCursor(200, y + 7);
          sprite1.print(on ? "ON" : "OFF");
          break;
        }
        case VCONF_NUMBER: {
          int16_t dispVal = isEditing ? vconf.editValue : item->value;
          sprite1.setTextColor(isEditing ? TFT_YELLOW : (selected ? 0xC618 : 0x6B6D));
          sprite1.setCursor(188, y + 7);
          if (isEditing) sprite1.printf("[%d]", dispVal);
          else sprite1.printf("%d", dispVal);
          break;
        }
        case VCONF_OPTIONS: {
          sprite1.setTextColor(selected ? TFT_CYAN : 0x6B6D);
          sprite1.setCursor(160, y + 7);
          int oi = constrain(item->value, 0, item->optionCount - 1);
          if (item->optionCount > 0)
            sprite1.print(item->optionLabels[oi]);
          break;
        }
        case VCONF_ACTION: {
          sprite1.setTextColor(selected ? TFT_YELLOW : 0x4208);
          sprite1.setCursor(224, y + 7);
          sprite1.print("!");
          break;
        }
        case VCONF_SUBMENU: {
          sprite1.setTextColor(selected ? 0x6B6D : 0x4208);
          sprite1.setCursor(224, y + 3);
          sprite1.setTextSize(2);
          sprite1.print(">");
          break;
        }
      }
    } else {
      sprite1.setTextColor(selected ? TFT_ORANGE : 0x6B6D);
      sprite1.print("< Back");
    }
  }

  // Scroll indicators
  sprite1.setTextSize(1);
  if (startIdx > 0) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(116, 17);
    sprite1.print("^");
  }
  if (startIdx + visCount < totalItems) {
    sprite1.setTextColor(0x4A69);
    sprite1.setCursor(116, 115);
    sprite1.print("v");
  }

  // Bottom hint bar
  sprite1.fillRect(0, 122, 240, 13, 0x0841);
  sprite1.setTextColor(0x4A69);
  sprite1.setTextSize(1);
  sprite1.setCursor(20, 125);
  if (vconf.editing)
    sprite1.print("UP/DN:adjust  OK:confirm  BACK:cancel");
  else
    sprite1.print("UP/DN:browse  OK:select  BACK:up");

  sprite1.pushSprite(0, 0);
}

void renderD1Menu() {
  extern const char* d1ScreenNames[];
  extern const char* d2ScreenNames[];

  if (menu.level == MENU_L1) {
    drawMenuList("MENU", menuL1Names, ML1_COUNT, menu.l1Idx, NULL);
    return;
  }

  if (menu.level == MENU_L2) {
    if (menu.l1Parent == ML1_SCREEN_SEL) {
      drawMenuList("SCREENS", menuL2ScreenNames, ML2S_COUNT,
                   menu.l2Idx, "Screen Sel");
    } else if (menu.l1Parent == ML1_NAVIGATION) {
      // Dynamic label for preview item
      const char* navNames[ML2N_COUNT];
      for (int i = 0; i < ML2N_COUNT; i++) navNames[i] = menuL2NavNames[i];
      if (menu.previewMode) navNames[ML2N_PREVIEW] = menuL2NavPreviewStop;
      drawMenuList("NAVIGATION", navNames, ML2N_COUNT,
                   menu.l2Idx, "Nav");
    } else if (menu.l1Parent == ML1_VEHICLE) {
      drawVehicleList("VEHICLE", "Vehicle", vconfRootCount(),
                      menu.l2Idx, vconfRootItem);
    }
    return;
  }

  if (menu.level == MENU_L3) {
    if (menu.l1Parent == ML1_SCREEN_SEL && menu.l2Parent == ML2S_DISPLAY1) {
      drawMenuList("DISPLAY 1", d1ScreenNames, D1_SCREEN_COUNT,
                   menu.l3Idx, "Screens");
    } else if (menu.l1Parent == ML1_SCREEN_SEL && menu.l2Parent == ML2S_DISPLAY2) {
      drawMenuList("DISPLAY 2", d2ScreenNames, D2_SCREEN_COUNT,
                   menu.l3Idx, "Screens");
    } else if (menu.l1Parent == ML1_NAVIGATION && menu.l2Parent == ML2N_TRACK_SEL) {
      // Track list with distances — custom rendering
      sprite1.fillScreen(TFT_BLACK);
      sprite1.fillRect(0, 0, 240, 16, 0x0841);
      sprite1.setTextSize(1);
      sprite1.setTextColor(0x4A69);
      sprite1.setCursor(4, 4);
      sprite1.print("Nav > ");
      sprite1.setTextColor(TFT_CYAN);
      sprite1.print("TRACKS");

      if (menu.previewMode) {
        if ((millis() / 400) % 2) sprite1.fillCircle(230, 8, 4, TFT_RED);
      }

      int totalItems = menu.trackCount + 1;
      sprite1.setTextSize(2);

      if (menu.trackCount == 0) {
        sprite1.setTextColor(0x8410);
        sprite1.setCursor(18, 45);
        sprite1.print("No routes");
        sprite1.setCursor(18, 68);
        sprite1.setTextSize(1);
        sprite1.print("Use web UI to upload");
      } else {
        int maxVis = 4;
        int visCount = min(maxVis, totalItems);
        int startIdx = menu.l3Idx - visCount / 2;
        if (startIdx < 0) startIdx = 0;
        if (startIdx + visCount > totalItems) startIdx = totalItems - visCount;

        for (int i = 0; i < visCount; i++) {
          int idx = startIdx + i;
          int y = 18 + i * 24;
          bool selected = (idx == menu.l3Idx);
          if (selected) {
            sprite1.fillRect(2, y, 236, 22, 0x0841);
            sprite1.drawRect(2, y, 236, 22, TFT_CYAN);
            sprite1.setTextColor(TFT_WHITE);
          } else {
            sprite1.setTextColor(0x8410);
          }
          sprite1.setTextSize(2);
          sprite1.setCursor(8, y + 3);
          if (idx < menu.trackCount) {
            sprite1.print(menu.names[idx]);
            // Distance in small font on right
            int dist = menu.distances[idx];
            sprite1.setTextSize(1);
            sprite1.setTextColor(selected ? 0x6B6D : 0x4208);
            sprite1.setCursor(190, y + 7);
            if (dist < 1000) sprite1.printf("%dm", dist);
            else sprite1.printf("%.1fkm", dist / 1000.0f);
          } else {
            sprite1.setTextColor(selected ? TFT_ORANGE : 0x6B6D);
            sprite1.print("< Back");
          }
        }

        // Scroll indicators
        sprite1.setTextSize(1);
        if (startIdx > 0) {
          sprite1.setTextColor(0x4A69);
          sprite1.setCursor(116, 17);
          sprite1.print("^");
        }
        if (startIdx + visCount < totalItems) {
          sprite1.setTextColor(0x4A69);
          sprite1.setCursor(116, 115);
          sprite1.print("v");
        }
      }

      sprite1.fillRect(0, 122, 240, 13, 0x0841);
      sprite1.setTextColor(0x4A69);
      sprite1.setTextSize(1);
      sprite1.setCursor(20, 125);
      sprite1.print("UP/DN:browse  OK:select  BACK:up");
      sprite1.pushSprite(0, 0);
    } else if (menu.l1Parent == ML1_VEHICLE) {
      // L3 = children of a vehicle submenu
      VConfItem* parent = vconfRootItem(menu.l2Parent);
      if (parent && parent->type == VCONF_SUBMENU) {
        _vconfChildGroupId = parent->id;
        drawVehicleList(parent->name, "Vehicle",
          vconfChildCount(parent->id), menu.l3Idx,
          _vconfChildItemWrapper);
      }
    }
  }
}
