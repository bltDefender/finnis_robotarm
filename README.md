# finnis_robotarm

ESP32/PlatformIO project for controlling Finnis' robot arm via MQTT and HTTP.

## Features

- Control a 4-servo robot arm with an ESP32
- MQTT command interface
- HTTP API for status and commands
- Relative motion commands
- Inverse kinematics command for 2D target positions
- Configurable axis limits stored in Preferences
- Servo release / re-enable workflow
- `set_zero_here` workflow for defining a new logical zero position
- WiFi and MQTT secrets moved to `include/config.h`

## Project structure

- `src/main.cpp` - main firmware
- `include/config.h` - local WiFi / MQTT configuration

## Configuration

Create or edit `include/config.h`:

````cpp
#pragma once

// WLAN
static const char* WIFI_SSID = "YOUR_WIFI";
static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// MQTT
static const char* MQTT_HOST = "192.168.188.220";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USER = "YOUR_MQTT_USER";
static const char* MQTT_PASS = "YOUR_MQTT_PASSWORD";
static const char* MQTT_CLIENT_ID = "finnis_robotarm_esp32";
````

## MQTT topics

- Command input: `finnis/robotarm/cmd`
- State output: `finnis/robotarm/state` (retained)
- Command response: `finnis/robotarm/response`
- Availability: `finnis/robotarm/availability`

## Command examples

### Status

```json
{"mode":"status"}
```

### Start engines

```json
{"mode":"start_engines"}
```

### Release engines

```json
{"mode":"release_engines"}
```

### Set new logical zero

Use this workflow if you manually move the arm while the servos are released:

1. Send `release_engines`
2. Move the arm by hand
3. Send `set_zero_here`
4. Send `start_engines`

Command:

```json
{"mode":"set_zero_here"}
```

### Move to relative angles

```json
{"mode":"move_rel","body":0,"shoulder":20,"elbow":-10,"gripper":0}
```

### Jog relative

```json
{"mode":"jog_rel","shoulder":5}
```

### Inverse kinematics

```json
{"mode":"ik","x":120,"y":80,"body":0,"gripper":0}
```

Optional:

```json
{"mode":"ik","x":120,"y":80,"elbowUp":true}
```

### Set positive limit

```json
{"mode":"set_limit_pos","axis":"shoulder","value":70}
```

### Set negative limit

```json
{"mode":"set_limit_neg","axis":"shoulder","value":-70}
```

### Save limits

```json
{"mode":"save_limits"}
```

## HTTP API

- `GET /` - API overview
- `GET /api/scan` - device discovery info
- `GET /api/state` - current state JSON
- `POST /api/cmd` - send a JSON command body

Example:

```bash
curl -X POST http://<device-ip>/api/cmd \
  -H "Content-Type: application/json" \
  -d '{"mode":"status"}'
```

## Notes

- `status` works even when engines are disabled.
- Most motion commands require `start_engines` first.
- `set_zero_here` currently sets the logical zero based on the firmware's current internal pose model.
- Without external position feedback, manually moving released servos cannot be measured by the ESP32.
