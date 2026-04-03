#pragma once
#include <Wire.h>

// ─── SD2405 RTC via Wire (master mode on dedicated bus) ─────────────────────

#define RTC_ADDR        0x32
#define RTC_SDA_PIN     12
#define RTC_SCL_PIN     32

struct RtcTime {
  uint8_t second;   // 0-59
  uint8_t minute;   // 0-59
  uint8_t hour;     // 0-23
  uint8_t day;      // 1-31
  uint8_t month;    // 1-12
  uint16_t year;    // 2000-2099
  uint8_t weekday;  // 0-6 (Sunday=0)
  bool valid;       // true if RTC was read successfully
};

static RtcTime rtcTime = {};

// ─── BCD helpers ────────────────────────────────────────────────────────────

static uint8_t bcd2dec(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }
static uint8_t dec2bcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }

// ─── Init ───────────────────────────────────────────────────────────────────

bool rtcInit() {
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN, 100000);

  // Probe: read seconds register
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("[RTC] SD2405 not found at 0x32");
    return false;
  }
  Serial.println("[RTC] SD2405 found");
  return true;
}

// ─── Read time ──────────────────────────────────────────────────────────────

bool rtcRead() {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);  // start at register 0
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7) != 7) return false;

  uint8_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = Wire.read();

  rtcTime.second  = bcd2dec(raw[0] & 0x7F);
  rtcTime.minute  = bcd2dec(raw[1] & 0x7F);
  // SD2405 hour register: mask off control bits, BCD decode, wrap to 0-23
  rtcTime.hour    = bcd2dec(raw[2] & 0x3F) % 24;
  rtcTime.weekday = bcd2dec(raw[3] & 0x07);
  rtcTime.day     = bcd2dec(raw[4] & 0x3F);
  rtcTime.month   = bcd2dec(raw[5] & 0x1F);
  rtcTime.year    = 2000 + bcd2dec(raw[6]);
  rtcTime.valid   = true;
  return true;
}

// ─── Write time ─────────────────────────────────────────────────────────────

bool rtcWrite(uint16_t year, uint8_t month, uint8_t day,
              uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday) {
  // Disable write protection
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x10);
  Wire.write(0x80);  // set WRTC1 bit
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x0F);
  Wire.write(0x84);  // set WRTC2 + WRTC3 bits
  if (Wire.endTransmission() != 0) return false;

  // Write time registers 0x00-0x06
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(dec2bcd(second));
  Wire.write(dec2bcd(minute));
  Wire.write(dec2bcd(hour));       // 24h mode BCD
  Wire.write(dec2bcd(weekday));
  Wire.write(dec2bcd(day));
  Wire.write(dec2bcd(month));
  Wire.write(dec2bcd(year - 2000));
  if (Wire.endTransmission() != 0) return false;

  // Re-enable write protection
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x0F);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.printf("[RTC] Set: %04d-%02d-%02d %02d:%02d:%02d\n",
                year, month, day, hour, minute, second);
  return true;
}

// ─── Write from struct ──────────────────────────────────────────────────────

bool rtcWriteTime(const RtcTime& t) {
  return rtcWrite(t.year, t.month, t.day, t.hour, t.minute, t.second, t.weekday);
}

// ─── System clock sync from RTC ─────────────────────────────────────────────

#include <WiFi.h>
#include "time.h"
#include <sys/time.h>

// Set ESP32 system clock from RTC values (call after rtcRead)
void rtcSyncSystemClock() {
  if (!rtcTime.valid) return;
  struct tm t = {};
  t.tm_year = rtcTime.year - 1900;
  t.tm_mon  = rtcTime.month - 1;
  t.tm_mday = rtcTime.day;
  t.tm_hour = rtcTime.hour;
  t.tm_min  = rtcTime.minute;
  t.tm_sec  = rtcTime.second;
  time_t epoch = mktime(&t);
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  Serial.printf("[RTC] System clock set: %04d-%02d-%02d %02d:%02d:%02d\n",
                rtcTime.year, rtcTime.month, rtcTime.day,
                rtcTime.hour, rtcTime.minute, rtcTime.second);
}

// Read current time from ESP32 system clock (fast, no I2C)
void rtcGetSystemTime() {
  struct tm t;
  if (getLocalTime(&t, 0)) {
    rtcTime.second  = t.tm_sec;
    rtcTime.minute  = t.tm_min;
    rtcTime.hour    = t.tm_hour;
    rtcTime.day     = t.tm_mday;
    rtcTime.month   = t.tm_mon + 1;
    rtcTime.year    = t.tm_year + 1900;
    rtcTime.weekday = t.tm_wday;
    rtcTime.valid   = true;
  }
}

// ─── NTP sync ───────────────────────────────────────────────────────────────

static bool ntpSynced = false;
static unsigned long lastNtpAttemptMs = 0;

bool rtcSyncFromNTP() {
  if (WiFi.status() != WL_CONNECTED) return false;

  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

  struct tm t;
  if (!getLocalTime(&t, 5000)) {
    Serial.println("[RTC] NTP: failed to get time");
    return false;
  }

  uint16_t year = t.tm_year + 1900;
  uint8_t month = t.tm_mon + 1;
  uint8_t wday  = t.tm_wday;

  if (rtcWrite(year, month, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, wday)) {
    ntpSynced = true;
    Serial.printf("[RTC] NTP sync OK: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                  year, month, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    return true;
  }
  return false;
}

// ─── Periodic NTP check (call from loop, non-blocking) ─────────────────────

void rtcNtpTick() {
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  // First sync: try every 10s until success. Then every 6 hours.
  unsigned long interval = ntpSynced ? 6UL * 3600 * 1000 : 10000;
  if (now - lastNtpAttemptMs < interval) return;
  lastNtpAttemptMs = now;

  rtcSyncFromNTP();
}
