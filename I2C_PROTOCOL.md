# I2C Protocol: MEGA 2560 (Master) -> ESP32 Display (Slave)

## Wiring

```
MEGA 2560 PRO Mini                     ESP32 (LilyGo T-Display v1.1)
Pin 20 (SDA) ---- [Level Shifter] ---- GPIO 26 (SDA)
Pin 21 (SCL) ---- [Level Shifter] ---- GPIO 25 (SCL)
GND          ---- [common ground] ---- GND

Level Shifter: BSS138-based, HV=5V, LV=3.3V
I2C speed: 100kHz
ESP32 Slave Address: 0x42

SD2405 RTC (on ESP32 Wire bus):
GPIO 12 (SDA) ---- SD2405 SDA
GPIO 32 (SCL) ---- SD2405 SCL
RTC I2C Address: 0x32
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

### 0x01 TELEMETRY_DRIVE (8 bytes payload, 10 Hz)
```
[0] throttle_raw     uint8    0-255 (center=127, >127=fwd, <127=brake)
[1] speed_kmh_x10_H  uint8    high byte of speed*10
[2] speed_kmh_x10_L  uint8    low byte (0-2000 = 0.0-200.0 km/h)
[3] rpm_x10_H        uint8    high byte of RPM/10
[4] rpm_x10_L        uint8    low byte (0-65530)
[5] motor_state      uint8    bits: [0]=armed [1]=running [2]=forward [3]=braking
[6] steer_pos        uint8    0-255 (left..center..right)
[7] accel_trend      int8     speed delta indicator
```

### 0x02 TELEMETRY_LINK (5 bytes payload, 2 Hz)
```
[0] rssi_dbm         int8     -128..0 dBm (best of 2 antennas, from CRSF 0x14)
[1] link_quality     uint8    0-100 (%) (from CRSF link stats)
[2] rx_connected     uint8    0=lost, 1=connected
[3] signal_bars      uint8    0-5 (derived from LQ: >80%=5, >60%=4, >40%=3, >20%=2, else 1)
[4] tx_power_dbm     uint8    0-30 (converted from CRSF TX power index)
```

### 0x03 TELEMETRY_BATTERY (6 bytes payload, 1 Hz)
```
[0] voltage_x100_H   uint8    high byte of voltage*100
[1] voltage_x100_L   uint8    low byte (e.g. 1184 = 11.84V)
[2] percentage        uint8    0-100 (%)
[3] current_x10_H    uint8    high byte of current*10 (amps)
[4] current_x10_L    uint8    low byte
[5] flags            uint8    bits: [0]=charging [1]=low_warning [2]=critical
```

### 0x04 TELEMETRY_GPS (17 bytes payload, 2 Hz)
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

### 0x05 TELEMETRY_NAV (10 bytes payload, 5 Hz)
```
[0..1]  heading_x10     uint16   heading * 10 (0-3599)
[2..3]  course_x10      uint16   GPS course * 10
[4..7]  distance_home_m uint32   distance to home in meters
[8..9]  heading_mag_x10 uint16   mag heading * 10
```

### 0x06 TELEMETRY_SENSOR (14 bytes payload, 5 Hz)
```
[0..1]  accel_x_x100   int16    m/s^2 * 100
[2..3]  accel_y_x100   int16
[4..5]  accel_z_x100   int16
[6..7]  gyro_z_x10     int16    deg/s * 10
[8..9]  temperature_x10 int16   C * 10
[10..13] pressure_pa   uint32   Pa
```

### 0x07 TELEMETRY_VEHICLE (6 bytes payload, 2 Hz)
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
- L1: Screen Select, Navigation, Vehicle, Restart
- L2 (Screen Select): Display 1, Display 2
- L2 (Navigation): Track Select, (Re)Start Nav, Stop Nav, Track Preview, Stop Preview, Show/Hide Path
- L2 (Vehicle): dynamic items defined by MEGA (see 0x20-0x24)
- L2 (Restart): Restart Dashboard, Restart Vehicle
- L3 (Display 1): Dashboard, Navigation, Media, IMU, About
- L3 (Display 2): Navigation, Map, Battery, Signal, Media, Debug
- L3 (Track Select): horizontal thumbnail carousel of .rte files from SD
- L3 (Vehicle submenu): children of a submenu-type item

**Menu open/close:**
- CH2 switch toggle (any direction) opens menu (1s debounce)
- Menu close only via selecting Exit/Back items
- Opening menu disarms motor and centers steering servo on Vehicle

**Menu control via steering/throttle (when menu is active):**
- Steer left (pos < 76): next item (~40% deflection)
- Steer right (pos > 178): previous item (~40% deflection)
- Throttle forward ~26% then back to ~0: enter/confirm
- Vehicle steering servo and display servo disabled while menu is open

### 0x20 VCONF_DEFINE_ITEM (13-21 bytes payload, on event)
Define or redefine a custom vehicle configuration menu item.
```
[0]     item_id        uint8    0-15 (slot index)
[1]     item_type      uint8    0=toggle, 1=number, 2=options, 3=action, 4=submenu
[2]     flags          uint8    bit0=readonly, bit1=hidden
[3..12] name           char[10] null-padded display label

Type-specific data after name:
  toggle:  [13] value          uint8   0=off, 1=on
  number:  [13..14] value      int16   current value
           [15..16] min        int16   minimum
           [17..18] max        int16   maximum
           [19..20] step       int16   increment step
  options: [13] current_idx    uint8   selected option (0-based)
           [14] option_count   uint8   total options (send labels via 0x21)
  action:  (no extra data)
  submenu: [13] parent_id      uint8   0xFF=root (usually 0xFF for the submenu itself)
```

### 0x21 VCONF_OPTION_LABEL (12 bytes payload, on event)
Send one option label for an options-type item. Send after DEFINE.
```
[0]     item_id        uint8    which item
[1]     option_idx     uint8    0-5
[2..11] label          char[10] null-padded
```

### 0x22 VCONF_UPDATE_VALUE (3 bytes payload, on event)
Update current value of an existing item (e.g. MEGA confirms change).
```
[0]     item_id        uint8
[1..2]  value          int16    new value (little-endian)
```

### 0x23 VCONF_CLEAR (1 byte payload, on event)
```
[0]     item_id        uint8    0xFF=clear all, else clear specific item
```

### 0x24 VCONF_SET_GROUP (2 bytes payload, on event)
Assign an item to a submenu group (for nesting under submenu-type items).
```
[0]     item_id        uint8    item to assign
[1]     group_id       uint8    item_id of parent submenu (0xFF=root)
```

### Event Polling (ESP32 → MEGA)
MEGA polls for user changes via `Wire.requestFrom(0x42, 4)`.
ESP32 responds with 4 bytes:
```
[0]     item_id        uint8    0xFF=no event, else item that changed
[1]     value_lo       uint8    new value low byte
[2]     value_hi       uint8    new value high byte
[3]     checksum       uint8    XOR of bytes 0..2
```
Events: toggle (0/1), number (confirmed value), options (new index),
action (value=1 when triggered). Queue holds up to 8 events.

**Reserved event IDs (ESP32 → MEGA system events):**
```
0xFE  Menu state     value: 1=menu opened, 0=menu closed
0xFD  Time (HH:MM)   value: high byte=hour (0-23), low byte=minute (0-59)
0xFC  Date (MM.DD)   value: high byte=month (1-12), low byte=day (1-31)
0xFB  Year           value: year (e.g. 2025)
0xFA  Restart cmd    value: 1 = restart vehicle (MEGA resets)
```
Time events are sent every 10s from ESP32 RTC (SD2405).
NTP sync: ESP32 syncs RTC via NTP when WiFi connected (TZ: CET/CEST Bratislava).

**Item interaction on display:**
- Toggle: Enter cycles on/off
- Number: Enter starts edit → Up/Down adjust by step → Enter confirms
- Options: Enter cycles to next option
- Action: Enter triggers immediately
- Submenu: Enter opens child list

### 0xFE HEARTBEAT (2 bytes payload, 1 Hz)
```
[0..1] uptime_sec    uint16   MEGA uptime seconds (wraps at 65535)
```

## Screen Indices

### D1 (Main Display)
0=Dashboard, 1=Navigation, 2=Media, 3=IMU, 4=About

### D2 (Second Display)
0=Navigation, 1=Map, 2=Battery, 3=Signal, 4=Media, 5=Radio, 6=Debug

## MEGA Transmission Schedule

| Message   | Interval | Rate   |
|-----------|----------|--------|
| DRIVE     | 100ms    | 10 Hz  |
| NAV       | 200ms    | 5 Hz   |
| SENSOR    | 200ms    | 5 Hz   |
| GPS       | 500ms    | 2 Hz   |
| LINK      | 500ms    | 2 Hz   |
| VEHICLE   | 500ms    | 2 Hz   |
| BATTERY   | 1000ms   | 1 Hz   |
| HEARTBEAT | 1000ms   | 1 Hz   |
| POLL      | 200ms    | 5 Hz   |

I2C timeout: 5ms (Wire.setWireTimeout). Bus recovery on timeout.

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
