# finnis_robotarm

ESP32/PlatformIO project for controlling Finnis' robot arm via browser, HTTP, and MQTT.

## Features

- Control a 4-servo robot arm with an ESP32
- Built-in web UI served directly by the ESP32 on port 80
- Absolute position control from the browser
- Persistent named macros stored on the ESP32
- Save a macro by entered values or capture the current pose
- MQTT command interface
- HTTP API for state, commands, and macro handling
- Relative motion commands
- Inverse kinematics command for 2D target positions
- Configurable axis limits stored in Preferences
- Safe sequential startup homing sequence
- WiFi and MQTT secrets moved to `src/config.h`

## Project structure

- `src/main.cpp` - main firmware
- `src/config.h` - local WiFi / MQTT configuration
- `platformio.ini` - PlatformIO environment and library setup

## Configuration

Create or edit `src/config.h`:

```cpp
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
```

## Browser UI

After flashing and connecting the ESP32 to WiFi, open the device IP in a browser:

```text
http://<device-ip>/
```

The ESP32 itself serves the web page on port 80.

The web UI provides:

- current status display
- absolute position input for all four servos
- button to move to the entered absolute position
- button to copy the current position into the input fields
- macro save by entered values
- macro save from the current live position
- list of stored macros
- macro execution button
- macro delete button

## Startup behavior

On boot the servos are moved in a fixed safe sequence:

1. Elbow to `90`
2. Shoulder to `90`
3. Body to `0`
4. Gripper to `90`

These values are currently hardcoded in `src/main.cpp`:

```cpp
static const int STARTUP_ELBOW    = 90;
static const int STARTUP_SHOULDER = 90;
static const int STARTUP_BODY     = 0;
static const int STARTUP_GRIPPER  = 90;
```

If your mechanics require different safe startup angles, change these constants.

## Servo limits

Current absolute servo limits in the firmware:

- Body: `0 .. 180`
- Shoulder: `5 .. 175`
- Elbow: `5 .. 175`
- Gripper: `10 .. 170`

## Macros

Macros are stored as **absolute servo positions** and persist across restarts using ESP32 Preferences.

A macro contains:

- `name`
- `body`
- `shoulder`
- `elbow`
- `gripper`

Example:

```json
{
  "name": "park",
  "body": 0,
  "shoulder": 90,
  "elbow": 90,
  "gripper": 90
}
```

## MQTT topics

- Command input: `finnis/robotarm/cmd`
- State output: `finnis/robotarm/state` (retained)
- Command response: `finnis/robotarm/response`
- Availability: `finnis/robotarm/availability`

## HTTP API

### UI and general endpoints

- `GET /` - web UI
- `GET /api/scan` - device discovery info
- `GET /api/state` - current state JSON
- `POST /api/cmd` - send a JSON command body

### Browser and macro endpoints

- `POST /api/move_abs` - move to an absolute position
- `GET /api/macros` - list stored macros
- `POST /api/macros/save` - save or overwrite a macro from absolute values
- `POST /api/macros/capture` - save current live position as a macro
- `POST /api/macros/run` - execute a macro by name
- `POST /api/macros/delete` - delete a macro by name

## Command examples

### Status

```json
{"mode":"status"}
```

### Move to relative angles

```json
{"mode":"move_rel","body":0,"shoulder":20,"elbow":-10,"gripper":0}
```

### Jog relative

```json
{"mode":"jog_rel","shoulder":5}
```

### Move to absolute angles

```json
{"mode":"move_abs","body":0,"shoulder":90,"elbow":90,"gripper":90}
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

## Macro API examples

### Save macro from absolute values

```json
{
  "name": "pickup",
  "body": 10,
  "shoulder": 110,
  "elbow": 130,
  "gripper": 70
}
```

### Run macro

```json
{
  "name": "pickup"
}
```

### Delete macro

```json
{
  "name": "pickup"
}
```

## Build and upload

```bash
pio run
pio run -t upload
pio device monitor
```

## PlatformIO libraries

The current firmware uses these libraries from `platformio.ini`:

- `knolleary/PubSubClient`
- `madhephaestus/ESP32Servo`

No additional external library is required for the web UI or macro storage.
The browser UI uses the built-in `WebServer` from the ESP32 Arduino framework, and persistent storage uses the built-in `Preferences` API.

## Notes

- The web UI runs directly on the ESP32.
- Macros are stored locally on the chip and survive restarts.
- Browser actions use HTTP requests internally; you do not need to enter JSON manually in normal use.
- MQTT control remains available in parallel to the browser UI.
- If startup movement is mechanically unsafe, adjust the `STARTUP_*` constants in `src/main.cpp`.
