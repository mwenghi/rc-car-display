#pragma once
#include <Wire.h>

// I2C config
#define I2C_SDA_PIN  26
#define I2C_SCL_PIN  25
#define I2C_ADDR     0x42

// Message types
#define MSG_DRIVE     0x01
#define MSG_LINK      0x02
#define MSG_BATTERY   0x03
#define MSG_GPS       0x04
#define MSG_NAV       0x05
#define MSG_SENSOR    0x06
#define MSG_VEHICLE   0x07
#define MSG_HEAD_TRACKER 0x08
#define MSG_CMD_SCREEN     0x10
#define MSG_CMD_BRIGHTNESS 0x11
#define MSG_CMD_MEDIA      0x12
#define MSG_CMD_MENU       0x13  // menu navigation control
#define MSG_MEDIA_STATE    0x14  // media playback state from vehicle
#define MSG_VCONF_DEFINE   0x20  // define custom menu item
#define MSG_VCONF_OPTION   0x21  // option label for options-type item
#define MSG_VCONF_UPDATE   0x22  // update item value
#define MSG_VCONF_CLEAR    0x23  // clear item(s)
#define MSG_VCONF_GROUP    0x24  // assign item to submenu group
#define MSG_HEARTBEAT 0xFE

// Timeout for master presence detection
#define I2C_TIMEOUT_MS 2000

// ─── Live data struct ────────────────────────────────────────────────────────

struct LiveData {
  // Drive (0x01)
  float speed_kmh;
  float throttle_pct;
  uint16_t rpm;
  uint8_t motor_state;
  uint8_t steer_pos;
  int8_t accel_trend;

  // Link (0x02)
  int8_t rssi_dbm;
  uint8_t link_quality;
  bool rx_connected;
  uint8_t signal_bars;

  // Battery (0x03)
  float voltage;
  uint8_t batt_pct;
  bool charging;
  bool low_warning;
  bool critical;
  float current_a;

  // GPS (0x04)
  double latitude;
  double longitude;
  float altitude_m;
  uint8_t satellites;
  uint8_t fix_type;
  float gps_speed_kmh;

  // Nav (0x05)
  float heading_deg;
  float course_deg;
  float heading_mag_deg;
  uint32_t distance_home_m;

  // Sensor (0x06)
  float accel_x, accel_y, accel_z;
  float gyro_z;
  float temperature;
  float pressure_pa;

  // Vehicle (0x07)
  uint8_t vehicle_state;  // 0=OFF 1=STANDBY 2=ARMED
  uint8_t light_mode;
  uint8_t blinker_state;
  bool forward;
  bool horn_active;
  uint8_t video_camera;

  // Head tracker (from CRSF channels)
  float ht_heading;   // degrees 0-360
  float ht_pitch;     // degrees ±180
  float ht_roll;      // degrees ±180
  float ht_yaw;       // degrees ±180

  // Media state (0x14)
  uint8_t media_state;    // 0=stopped, 1=playing, 2=paused
  uint8_t media_song;     // current song index 0-19
  uint8_t media_flags;    // bit0=system_sound_active
  uint8_t media_volume;   // 0-30

  // Meta
  bool masterPresent;
  uint32_t lastReceiveMs;
  uint8_t lastSeq;
  uint16_t megaUptimeSec;
};

LiveData liveData = {0};

// ─── I2C receive buffer (ISR-safe) ──────────────────────────────────────────

#define I2C_BUF_SIZE 34
volatile uint8_t i2cRxBuf[I2C_BUF_SIZE];
volatile uint8_t i2cRxLen = 0;
volatile bool i2cNewData = false;

// Forward declarations for screen switching
extern D1Screen d1Screen;
extern D2Screen d2Screen;
extern void saveScreenState();
extern LGFX_TDisplay disp1;
extern LGFX_Waveshare disp2;

// ─── I2C receive ISR ─────────────────────────────────────────────────────────

void onI2CReceive(int numBytes) {
  uint8_t len = 0;
  while (Wire1.available() && len < I2C_BUF_SIZE) {
    i2cRxBuf[len++] = Wire1.read();
  }
  i2cRxLen = len;
  i2cNewData = true;
}

// ─── Parse received message ──────────────────────────────────────────────────

void parseI2CMessage() {
  if (!i2cNewData) return;
  i2cNewData = false;

  uint8_t len = i2cRxLen;
  if (len < 4) return;  // minimum: type + seq + 1 payload + checksum

  // Copy from volatile buffer
  uint8_t buf[I2C_BUF_SIZE];
  for (int i = 0; i < len; i++) buf[i] = i2cRxBuf[i];

  // Verify checksum (XOR of all bytes except last)
  uint8_t xorSum = 0;
  for (int i = 0; i < len - 1; i++) xorSum ^= buf[i];
  if (xorSum != buf[len - 1]) {
    Serial.printf("I2C checksum fail: got 0x%02X expected 0x%02X\n", buf[len-1], xorSum);
    return;
  }

  uint8_t msgType = buf[0];
  uint8_t seq = buf[1];
  uint8_t* p = &buf[2];  // payload start

  liveData.lastReceiveMs = millis();
  liveData.lastSeq = seq;
  liveData.masterPresent = true;

  switch (msgType) {
    case MSG_DRIVE:
      if (len >= 11) {
        { // raw 0-255, center 127: above = forward throttle, below = brake
          int raw = (int)p[0] - 127;
          if (raw > 0) liveData.throttle_pct = raw * 100.0f / 128.0f;
          else         liveData.throttle_pct = raw * 100.0f / 127.0f;
        }
        liveData.speed_kmh = ((uint16_t)p[1] << 8 | p[2]) / 10.0f;
        liveData.rpm = ((uint16_t)p[3] << 8 | p[4]) * 10;
        liveData.motor_state = p[5];
        liveData.steer_pos = p[6];
        liveData.accel_trend = (int8_t)p[7];
      }
      break;

    case MSG_LINK:
      if (len >= 8) {
        liveData.rssi_dbm = (int8_t)p[0];
        liveData.link_quality = p[1];
        liveData.rx_connected = p[2];
        liveData.signal_bars = p[3];
      }
      break;

    case MSG_BATTERY:
      if (len >= 9) {
        liveData.voltage = ((uint16_t)p[0] << 8 | p[1]) / 100.0f;
        liveData.batt_pct = p[2];
        liveData.current_a = ((uint16_t)p[3] << 8 | p[4]) / 10.0f;
        liveData.charging = p[5] & 0x01;
        liveData.low_warning = p[5] & 0x02;
        liveData.critical = p[5] & 0x04;
      }
      break;

    case MSG_GPS:
      if (len >= 20) {
        int32_t lat, lon;
        memcpy(&lat, p, 4);
        memcpy(&lon, p + 4, 4);
        liveData.latitude = lat / 1e7;
        liveData.longitude = lon / 1e7;
        int16_t alt;
        memcpy(&alt, p + 8, 2);
        liveData.altitude_m = alt / 10.0f;
        liveData.satellites = p[10];
        liveData.fix_type = p[11];
        uint16_t spd;
        memcpy(&spd, p + 14, 2);
        liveData.gps_speed_kmh = spd / 10.0f;
      }
      break;

    case MSG_NAV:
      if (len >= 13) {
        uint16_t hdg, crs, mag;
        memcpy(&hdg, p, 2);
        memcpy(&crs, p + 2, 2);
        memcpy(&liveData.distance_home_m, p + 4, 4);
        memcpy(&mag, p + 8, 2);
        liveData.heading_deg = hdg / 10.0f;
        liveData.course_deg = crs / 10.0f;
        liveData.heading_mag_deg = mag / 10.0f;
      }
      break;

    case MSG_SENSOR:
      if (len >= 17) {
        int16_t ax, ay, az, gz, temp;
        uint32_t press;
        memcpy(&ax, p, 2); memcpy(&ay, p+2, 2); memcpy(&az, p+4, 2);
        memcpy(&gz, p+6, 2); memcpy(&temp, p+8, 2); memcpy(&press, p+10, 4);
        liveData.accel_x = ax / 100.0f;
        liveData.accel_y = ay / 100.0f;
        liveData.accel_z = az / 100.0f;
        liveData.gyro_z = gz / 10.0f;
        liveData.temperature = temp / 10.0f;
        liveData.pressure_pa = press;
      }
      break;

    case MSG_VEHICLE:
      if (len >= 9) {
        liveData.vehicle_state = p[0];
        liveData.light_mode = p[1];
        liveData.blinker_state = p[2];
        liveData.forward = (p[3] == 0);
        liveData.horn_active = p[4];
        liveData.video_camera = p[5];
      }
      break;

    case MSG_HEAD_TRACKER:
      if (len >= 11) {
        // 4x int16: heading_x10, pitch_x10, roll_x10, yaw_x10
        int16_t hd, pt, rl, yw;
        memcpy(&hd, p+0, 2);
        memcpy(&pt, p+2, 2);
        memcpy(&rl, p+4, 2);
        memcpy(&yw, p+6, 2);
        liveData.ht_heading = hd / 10.0f;
        liveData.ht_pitch   = pt / 10.0f;
        liveData.ht_roll    = rl / 10.0f;
        liveData.ht_yaw     = yw / 10.0f;
      }
      break;

    case MSG_MEDIA_STATE:
      if (len >= 6) {
        liveData.media_state = p[0];
        liveData.media_song = p[1];
        liveData.media_flags = p[2];
        if (len >= 7) liveData.media_volume = p[3];
      }
      break;

    case MSG_CMD_SCREEN:
      if (len >= 6) {
        uint8_t disp = p[0];
        uint8_t scr = p[1];
        if (disp == 1 && scr < D1_SCREEN_COUNT) {
          d1Screen = (D1Screen)scr;
          saveScreenState();
          Serial.printf("I2C: D1 -> screen %d\n", scr);
        } else if (disp == 2 && scr < D2_SCREEN_COUNT) {
          d2Screen = (D2Screen)scr;
          saveScreenState();
          Serial.printf("I2C: D2 -> screen %d\n", scr);
        }
      }
      break;

    case MSG_CMD_MENU:
      // Menu navigation control
      // p[0] = command: 0=open, 1=close, 2=next, 3=prev,
      //                 4=enter/confirm, 5=back, 6=start_nav, 7=preview
      // p[1] = param (unused for now)
      if (len >= 5) {
        extern void menuOpen();
        extern void menuClose();
        extern void menuNext();
        extern void menuPrev();
        extern void menuEnter();
        extern void menuBack();
        extern void startNavigation();
        extern void startPreview();
        switch (p[0]) {
          case 0: menuOpen(); break;
          case 1: menuClose(); break;
          case 2: menuNext(); break;
          case 3: menuPrev(); break;
          case 4: menuEnter(); break;
          case 5: menuBack(); break;
          case 6: startNavigation(); break;
          case 7: startPreview(); break;
        }
      }
      break;

    case MSG_CMD_BRIGHTNESS:
      if (len >= 6) {
        uint8_t disp = p[0];
        uint8_t brt = p[1];
        if (disp == 0 || disp == 1) disp1.setBrightness(brt);
        if (disp == 0 || disp == 2) disp2.setBrightness(brt);
        Serial.printf("I2C: brightness D%d = %d\n", disp, brt);
      }
      break;

    case MSG_HEARTBEAT:
      if (len >= 5) {
        memcpy(&liveData.megaUptimeSec, p, 2);
      }
      break;

    case MSG_VCONF_DEFINE:
      vconfParseDefine(p, len - 3);
      break;
    case MSG_VCONF_OPTION:
      vconfParseOptionLabel(p, len - 3);
      break;
    case MSG_VCONF_UPDATE:
      vconfParseUpdateValue(p, len - 3);
      break;
    case MSG_VCONF_CLEAR:
      vconfParseClear(p, len - 3);
      break;
    case MSG_VCONF_GROUP:
      vconfParseSetGroup(p, len - 3);
      break;

    default:
      Serial.printf("I2C: unknown msg 0x%02X\n", msgType);
      break;
  }
}

// ─── Simulation fallback ────────────────────────────────────────────────────

extern float simX, simY, simHeading, simSpeed, simKmh, simThrottle;

void updateLiveDataFromSim() {
  liveData.speed_kmh = simKmh;
  liveData.throttle_pct = simThrottle;  // sim already 0-100 (forward only)
  liveData.rpm = (uint16_t)(simThrottle / 100.0f * 0.7f + simKmh / 200.0f * 0.3f) * 8000;
  liveData.heading_deg = fmodf(simHeading * 180.0f / PI + 360.0f, 360.0f);

  // Simulated battery
  float t = millis() * 0.00005f;
  liveData.batt_pct = (uint8_t)constrain(50.0f + 45.0f * sinf(t), 0, 100);
  liveData.voltage = 3.2f + liveData.batt_pct / 100.0f * 0.95f;
  liveData.charging = (sinf(millis() * 0.0002f) > 0);

  // Simulated signal
  liveData.rssi_dbm = -50 + (int8_t)(30.0f * sinf(millis() * 0.00008f));
  liveData.link_quality = constrain(100 + liveData.rssi_dbm + 20 + (int)(10 * sinf(millis() * 0.00013f)), 0, 100);
  liveData.signal_bars = (liveData.rssi_dbm > -40) ? 5 : (liveData.rssi_dbm > -55) ? 4 : (liveData.rssi_dbm > -65) ? 3 : (liveData.rssi_dbm > -75) ? 2 : 1;
  liveData.rx_connected = true;

  // GPS from sim position
  pixelToLatLon(simX, simY, (float*)&liveData.latitude, (float*)&liveData.longitude);

  // Simulated vehicle state
  liveData.vehicle_state = 2;  // ARMED
  liveData.forward = true;
  liveData.light_mode = 2;     // normal headlights
  // Simulate blinkers: cycle through off/left/right/hazard
  int blinkerCycle = (millis() / 5000) % 4;
  liveData.blinker_state = (blinkerCycle == 1) ? 0x01 : (blinkerCycle == 2) ? 0x02 : (blinkerCycle == 3) ? 0x04 : 0x00;
  // Simulate braking when decelerating
  liveData.motor_state = (liveData.accel_trend < 0) ? 0x0B : 0x07;  // bit3=braking
  liveData.horn_active = false;

  liveData.steer_pos = (uint8_t)(128.0f + 100.0f * sinf(millis() * 0.0004f));

  // Simulated IMU — gentle roll/pitch oscillation like driving on rough terrain
  float imuT = millis() * 0.001f;
  liveData.accel_x = 0.3f * sinf(imuT * 1.1f) + 0.15f * sinf(imuT * 2.7f);  // roll
  liveData.accel_y = 0.2f * sinf(imuT * 0.8f) + 0.1f * cosf(imuT * 1.9f);   // pitch
  liveData.accel_z = 1.0f + 0.05f * sinf(imuT * 3.2f);                        // ~1g vertical
  liveData.gyro_z = 15.0f * sinf(imuT * 0.6f);                                // yaw rate
  liveData.temperature = 28.0f + 3.0f * sinf(imuT * 0.02f);

  liveData.masterPresent = false;
}

void onI2CRequest();  // forward declaration for reinit

// ─── Check master timeout ────────────────────────────────────────────────────

void checkI2CTimeout() {
  if (liveData.masterPresent && millis() - liveData.lastReceiveMs > I2C_TIMEOUT_MS) {
    liveData.masterPresent = false;
    // Reinitialize I2C slave — a bus glitch may have corrupted its state
    Wire1.end();
    delay(10);
    Wire1.begin(I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000);
    Wire1.onReceive(onI2CReceive);
    Wire1.onRequest(onI2CRequest);
    Serial.println("I2C: master timeout, slave reinit");
  }
}

// ─── I2C request callback (MEGA polls for menu events) ──────────────────────

void onI2CRequest() {
  uint8_t resp[4];
  vconfDequeueEvent(resp);
  Wire1.write(resp, 4);
}

// ─── Init I2C slave ──────────────────────────────────────────────────────────

void setupI2CSlave() {
  Wire1.begin(I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire1.onReceive(onI2CReceive);
  Wire1.onRequest(onI2CRequest);
  Serial.printf("I2C slave on 0x%02X (SDA=%d SCL=%d)\n", I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}
