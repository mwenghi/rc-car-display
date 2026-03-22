# I2C Protocol: MEGA 2560 (Master) -> ESP32 Display (Slave)

## Wiring

```
MEGA 2560 PRO Mini                     ESP32 (LilyGo T-Display v1.1)
Pin 20 (SDA) ---- [Level Shifter] ---- GPIO 26 (SDA)
Pin 21 (SCL) ---- [Level Shifter] ---- GPIO 32 (SCL)
GND          ---- [common ground] ---- GND

Level Shifter: BSS138-based, HV=5V, LV=3.3V
I2C speed: 400kHz (Fast Mode)
ESP32 Slave Address: 0x42
```

## Packet Format

Every I2C write from MEGA to ESP32:
```
Byte 0:    MSG_TYPE     (uint8_t)  - message type ID
Byte 1:    SEQ          (uint8_t)  - sequence counter (0-255, wraps)
Byte 2..N: PAYLOAD      (variable) - type-specific data
Byte N+1:  CHECKSUM     (uint8_t)  - XOR of bytes 0..N
```

All multi-byte values are **little-endian**.
Max payload: 29 bytes (32 byte I2C buffer - 3 bytes overhead).

## Message Types

### 0x01 TELEMETRY_DRIVE (8 bytes payload, 20 Hz)
```
[0] throttle_pct     uint8    0-100 (%)
[1] speed_kmh_x10_H  uint8    high byte of speed*10
[2] speed_kmh_x10_L  uint8    low byte (0-2000 = 0.0-200.0 km/h)
[3] rpm_x10_H        uint8    high byte of RPM/10
[4] rpm_x10_L        uint8    low byte (0-65530)
[5] motor_state      uint8    bits: [0]=armed [1]=running [2]=forward [3]=braking
[6] steer_pos        uint8    0-255 (left..center..right)
[7] accel_trend      int8     speed delta indicator
```

### 0x02 TELEMETRY_LINK (5 bytes payload, 5 Hz)
```
[0] rssi_dbm         int8     -128..0 dBm
[1] link_quality     uint8    0-100 (%)
[2] rx_connected     uint8    0=lost, 1=connected
[3] signal_bars      uint8    1-5
[4] tx_power_dbm     uint8    0-30
```

### 0x03 TELEMETRY_BATTERY (6 bytes payload, 2 Hz)
```
[0] voltage_x100_H   uint8    high byte of voltage*100
[1] voltage_x100_L   uint8    low byte (e.g. 1184 = 11.84V)
[2] percentage        uint8    0-100 (%)
[3] current_x10_H    uint8    high byte of current*10 (amps)
[4] current_x10_L    uint8    low byte
[5] flags            uint8    bits: [0]=charging [1]=low_warning [2]=critical
```

### 0x04 TELEMETRY_GPS (17 bytes payload, 5 Hz)
```
[0..3]  latitude       int32    lat * 1e7
[4..7]  longitude      int32    lon * 1e7
[8..9]  altitude_dm    int16    altitude in decimeters
[10]    satellites     uint8    sat count
[11]    fix_type       uint8    0=none 1=2D 2=3D
[12..13] hdop_x100    uint16   HDOP*100
[14..15] speed_x10    uint16   ground speed * 10 km/h
[16]    _reserved      uint8
```

### 0x05 TELEMETRY_NAV (10 bytes payload, 10 Hz)
```
[0..1]  heading_x10     uint16   heading * 10 (0-3599)
[2..3]  course_x10      uint16   GPS course * 10
[4..7]  distance_home_m uint32   distance to home in meters
[8..9]  heading_mag_x10 uint16   mag heading * 10
```

### 0x06 TELEMETRY_SENSOR (14 bytes payload, 10 Hz)
```
[0..1]  accel_x_x100   int16    m/s^2 * 100
[2..3]  accel_y_x100   int16
[4..5]  accel_z_x100   int16
[6..7]  gyro_z_x10     int16    deg/s * 10
[8..9]  temperature_x10 int16   C * 10
[10..13] pressure_pa   uint32   Pa
```

### 0x07 TELEMETRY_VEHICLE (6 bytes payload, 5 Hz)
```
[0] state            uint8    0=OFF 1=STANDBY 2=ARMED
[1] light_mode       uint8    0=off 1=parking 2=normal 3=distance
[2] blinker_state    uint8    bits: [0]=left [1]=right [2]=alert
[3] direction        uint8    0=forward 1=reverse
[4] horn_active      uint8    0/1
[5] video_camera     uint8    0=front 1=rear
```

### 0x10 CMD_SCREEN (3 bytes payload, on event)
```
[0] display          uint8    1=D1, 2=D2
[1] screen_idx       uint8    screen index
[2] _reserved        uint8
```

### 0x11 CMD_BRIGHTNESS (3 bytes payload, on event)
```
[0] display          uint8    1=D1, 2=D2, 0=both
[1] brightness       uint8    0-255
[2] _reserved        uint8
```

### 0x12 CMD_MEDIA (2 bytes payload, on event)
```
[0] command          uint8    0=play 1=pause 2=next 3=prev 4=stop
[1] _reserved        uint8
```

### 0x13 CMD_MENU (2 bytes payload, on event)
```
[0] command          uint8    0=open 1=close 2=next 3=prev
                              4=enter/confirm 5=back 6=start_nav 7=preview
[1] _reserved        uint8
```

**Menu structure (3-level):**
- L1: Screen Select, Navigation
- L2 (Screen Select): Display 1
- L2 (Navigation): Track Select
- L3 (Display 1): Dashboard, Navigation, Media, IMU, About
- L3 (Track Select): list of .rte files from SD

**Menu control via steering/throttle (when menu is active):**
- Steer left (pos < 80): previous item
- Steer right (pos > 175): next item
- Throttle to full (>90%) then back to 0 (<10%): enter/confirm

### 0xFE HEARTBEAT (2 bytes payload, 2 Hz)
```
[0..1] uptime_sec    uint16   MEGA uptime seconds (wraps at 65535)
```

## Screen Indices

### D1 (Main Display)
0=Dashboard, 1=Navigation, 2=Media, 3=IMU, 4=About

### D2 (Second Display)
0=Navigation, 1=Map, 2=Battery, 3=Signal, 4=Media, 5=Radio, 6=Debug

## MEGA Transmission Schedule

| Message | Interval | Bytes/sec |
|---------|----------|-----------|
| DRIVE   | 50ms     | 220       |
| NAV     | 100ms    | 130       |
| SENSOR  | 100ms    | 170       |
| GPS     | 200ms    | 100       |
| LINK    | 200ms    | 40        |
| VEHICLE | 200ms    | 45        |
| BATTERY | 500ms    | 18        |
| HEARTBEAT| 500ms   | 10        |
| **Total** |        | **~730 bytes/sec** (< 10% of 400kHz bus) |

## Presence Detection

### ESP32 (Slave)
- If no valid message received for 2 seconds -> `masterPresent = false` -> use simulation
- On valid message received -> `masterPresent = true` -> use live data
- Auto-recovers when MEGA reconnects

### MEGA (Master)
- On boot, send HEARTBEAT to 0x42
- If Wire.endTransmission() returns 0 (ACK) -> display connected
- If non-zero (NACK) -> display not connected, skip I2C sends
- Retry every 5 seconds to detect hot-plug
