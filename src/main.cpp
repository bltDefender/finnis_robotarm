#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WebServer.h>
#include <math.h>
#include <config.h>

// =====================================================
// MQTT Topics / HTTP
// =====================================================
static const char* TOPIC_CMD      = "finnis/robotarm/cmd";
static const char* TOPIC_STATE    = "finnis/robotarm/state";
static const char* TOPIC_RESPONSE = "finnis/robotarm/response";
static const char* TOPIC_AVAIL    = "finnis/robotarm/availability";

static const char* DEVICE_NAME = "finnis-robotarm";
WebServer http(80);

// =====================================================
// Pins
// =====================================================
static const int PIN_GRIPPER  = 14;
static const int PIN_ELBOW    = 27;
static const int PIN_SHOULDER = 33;
static const int PIN_BODY     = 32;

// =====================================================
// Servo limits / mechanics
// =====================================================
static float L1 = 110.0f;
static float L2 = 110.0f;

static int BODY_MIN = 0, BODY_MAX = 180;
static int SHOULDER_MIN = 5, SHOULDER_MAX = 175;
static int ELBOW_MIN = 5, ELBOW_MAX = 175;
static int GRIPPER_MIN = 10, GRIPPER_MAX = 170;

// Safe startup fallback sequence targets
static const int STARTUP_ELBOW    = 90;
static const int STARTUP_SHOULDER = 90;
static const int STARTUP_BODY     = 0;
static const int STARTUP_GRIPPER  = 90;

// Fine offsets for relative moves only
static int BODY_OFFSET = 0;
static int SHOULDER_OFFSET = 0;
static int ELBOW_OFFSET = 0;
static int GRIPPER_OFFSET = 0;

static const uint32_t SERVO_TICK_MS = 20;
static const int SERVO_STEP_DEG = 1;
static const int SERVO_DEADBAND_DEG = 0;
static const uint32_t START_MOVE_SETTLE_MS = 400;
static const uint32_t MQTT_RETRY_MS = 5000;
static const uint32_t POSE_SAVE_MS = 2000;
static const int POSE_SAVE_DELTA_DEG = 1;

uint32_t lastMqttAttemptMs = 0;
uint32_t lastPoseSaveMs = 0;

// Macro storage
static const int MAX_MACROS = 12;

struct MacroPose {
  String name;
  int body;
  int shoulder;
  int elbow;
  int gripper;
  bool used;
};

MacroPose macros[MAX_MACROS];

WiFiClient espClient;
PubSubClient mqtt(espClient);

Servo servoBody;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

Preferences prefs;

bool servosAttached = false;
bool resumePoseValid = false;

int curBody = STARTUP_BODY;
int curShoulder = STARTUP_SHOULDER;
int curElbow = STARTUP_ELBOW;
int curGripper = STARTUP_GRIPPER;

int tgtAbsBody = STARTUP_BODY;
int tgtAbsShoulder = STARTUP_SHOULDER;
int tgtAbsElbow = STARTUP_ELBOW;
int tgtAbsGripper = STARTUP_GRIPPER;

int zeroAbsBody = STARTUP_BODY;
int zeroAbsShoulder = STARTUP_SHOULDER;
int zeroAbsElbow = STARTUP_ELBOW;
int zeroAbsGripper = STARTUP_GRIPPER;

int relBody = 0;
int relShoulder = 0;
int relElbow = 0;
int relGripper = 0;

int tgtRelBody = 0;
int tgtRelShoulder = 0;
int tgtRelElbow = 0;
int tgtRelGripper = 0;

int limNegBody = -90, limPosBody = 90;
int limNegShoulder = -85, limPosShoulder = 85;
int limNegElbow = -85, limPosElbow = 85;
int limNegGripper = -80, limPosGripper = 80;

int lastSavedBody = STARTUP_BODY;
int lastSavedShoulder = STARTUP_SHOULDER;
int lastSavedElbow = STARTUP_ELBOW;
int lastSavedGripper = STARTUP_GRIPPER;

uint32_t lastServoTick = 0;

String lastStatus = "ready";
String lastDetail = "booted";

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampRel(int v, int n, int p) {
  if (v < n) return n;
  if (v > p) return p;
  return v;
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else {
      out += c;
    }
  }
  return out;
}

String buildMacrosJsonArray() {
  String out = "[";
  bool first = true;
  for (int i = 0; i < MAX_MACROS; i++) {
    if (!macros[i].used) continue;
    if (!first) out += ",";
    first = false;
    out += "{";
    out += "\"name\":\"" + jsonEscape(macros[i].name) + "\",";
    out += "\"body\":" + String(macros[i].body) + ",";
    out += "\"shoulder\":" + String(macros[i].shoulder) + ",";
    out += "\"elbow\":" + String(macros[i].elbow) + ",";
    out += "\"gripper\":" + String(macros[i].gripper);
    out += "}";
  }
  out += "]";
  return out;
}

String buildStateJson() {
  String s = "{";
  s += "\"status\":\"" + jsonEscape(lastStatus) + "\",";
  s += "\"detail\":\"" + jsonEscape(lastDetail) + "\",";
  s += "\"device\":\"" + String(DEVICE_NAME) + "\",";
  s += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  s += "\"mqttConnected\":" + String(mqtt.connected() ? "true" : "false") + ",";
  s += "\"attached\":" + String(servosAttached ? "true" : "false") + ",";
  s += "\"resumePoseValid\":" + String(resumePoseValid ? "true" : "false") + ",";
  s += "\"startupAbs\":{";
  s += "\"body\":" + String(STARTUP_BODY) + ",";
  s += "\"shoulder\":" + String(STARTUP_SHOULDER) + ",";
  s += "\"elbow\":" + String(STARTUP_ELBOW) + ",";
  s += "\"gripper\":" + String(STARTUP_GRIPPER) + "},";
  s += "\"curAbs\":{";
  s += "\"body\":" + String(curBody) + ",";
  s += "\"shoulder\":" + String(curShoulder) + ",";
  s += "\"elbow\":" + String(curElbow) + ",";
  s += "\"gripper\":" + String(curGripper) + "},";
  s += "\"tgtAbs\":{";
  s += "\"body\":" + String(tgtAbsBody) + ",";
  s += "\"shoulder\":" + String(tgtAbsShoulder) + ",";
  s += "\"elbow\":" + String(tgtAbsElbow) + ",";
  s += "\"gripper\":" + String(tgtAbsGripper) + "},";
  s += "\"rel\":{";
  s += "\"body\":" + String(relBody) + ",";
  s += "\"shoulder\":" + String(relShoulder) + ",";
  s += "\"elbow\":" + String(relElbow) + ",";
  s += "\"gripper\":" + String(relGripper) + "},";
  s += "\"tgtRel\":{";
  s += "\"body\":" + String(tgtRelBody) + ",";
  s += "\"shoulder\":" + String(tgtRelShoulder) + ",";
  s += "\"elbow\":" + String(tgtRelElbow) + ",";
  s += "\"gripper\":" + String(tgtRelGripper) + "},";
  s += "\"refAbs\":{";
  s += "\"body\":" + String(zeroAbsBody) + ",";
  s += "\"shoulder\":" + String(zeroAbsShoulder) + ",";
  s += "\"elbow\":" + String(zeroAbsElbow) + ",";
  s += "\"gripper\":" + String(zeroAbsGripper) + "},";
  s += "\"macros\":" + buildMacrosJsonArray();
  s += "}";
  return s;
}

void publishState(const char* status, const char* detail = "") {
  lastStatus = status;
  lastDetail = detail;

  String s = buildStateJson();
  if (mqtt.connected()) {
    mqtt.publish(TOPIC_STATE, s.c_str(), true);
    mqtt.publish(TOPIC_RESPONSE, s.c_str(), false);
  }
  Serial.println(s);
}

void resetRelativeModel() {
  relBody = 0;
  relShoulder = 0;
  relElbow = 0;
  relGripper = 0;
  tgtRelBody = 0;
  tgtRelShoulder = 0;
  tgtRelElbow = 0;
  tgtRelGripper = 0;
}

void loadLimits() {
  prefs.begin("armcfg", true);
  limNegBody     = prefs.getInt("bNeg", -90);
  limPosBody     = prefs.getInt("bPos",  90);
  limNegShoulder = prefs.getInt("sNeg", -85);
  limPosShoulder = prefs.getInt("sPos",  85);
  limNegElbow    = prefs.getInt("eNeg", -85);
  limPosElbow    = prefs.getInt("ePos",  85);
  limNegGripper  = prefs.getInt("gNeg", -80);
  limPosGripper  = prefs.getInt("gPos",  80);
  prefs.end();
}

void saveLimits() {
  prefs.begin("armcfg", false);
  prefs.putInt("bNeg", limNegBody);     prefs.putInt("bPos", limPosBody);
  prefs.putInt("sNeg", limNegShoulder); prefs.putInt("sPos", limPosShoulder);
  prefs.putInt("eNeg", limNegElbow);    prefs.putInt("ePos", limPosElbow);
  prefs.putInt("gNeg", limNegGripper);  prefs.putInt("gPos", limPosGripper);
  prefs.end();
}

void loadResumePose() {
  prefs.begin("armpose", true);
  resumePoseValid = prefs.getBool("valid", false);
  if (resumePoseValid) {
    curBody = clampInt(prefs.getInt("body", STARTUP_BODY), BODY_MIN, BODY_MAX);
    curShoulder = clampInt(prefs.getInt("shoulder", STARTUP_SHOULDER), SHOULDER_MIN, SHOULDER_MAX);
    curElbow = clampInt(prefs.getInt("elbow", STARTUP_ELBOW), ELBOW_MIN, ELBOW_MAX);
    curGripper = clampInt(prefs.getInt("gripper", STARTUP_GRIPPER), GRIPPER_MIN, GRIPPER_MAX);
  } else {
    curBody = STARTUP_BODY;
    curShoulder = STARTUP_SHOULDER;
    curElbow = STARTUP_ELBOW;
    curGripper = STARTUP_GRIPPER;
  }
  prefs.end();

  tgtAbsBody = curBody;
  tgtAbsShoulder = curShoulder;
  tgtAbsElbow = curElbow;
  tgtAbsGripper = curGripper;

  zeroAbsBody = curBody;
  zeroAbsShoulder = curShoulder;
  zeroAbsElbow = curElbow;
  zeroAbsGripper = curGripper;

  lastSavedBody = curBody;
  lastSavedShoulder = curShoulder;
  lastSavedElbow = curElbow;
  lastSavedGripper = curGripper;
}

void saveResumePoseNow() {
  prefs.begin("armpose", false);
  prefs.putBool("valid", true);
  prefs.putInt("body", curBody);
  prefs.putInt("shoulder", curShoulder);
  prefs.putInt("elbow", curElbow);
  prefs.putInt("gripper", curGripper);
  prefs.end();

  lastSavedBody = curBody;
  lastSavedShoulder = curShoulder;
  lastSavedElbow = curElbow;
  lastSavedGripper = curGripper;
  lastPoseSaveMs = millis();
}

void maybeSaveResumePose() {
  uint32_t now = millis();
  if (now - lastPoseSaveMs < POSE_SAVE_MS) return;

  bool changed =
    abs(curBody - lastSavedBody) >= POSE_SAVE_DELTA_DEG ||
    abs(curShoulder - lastSavedShoulder) >= POSE_SAVE_DELTA_DEG ||
    abs(curElbow - lastSavedElbow) >= POSE_SAVE_DELTA_DEG ||
    abs(curGripper - lastSavedGripper) >= POSE_SAVE_DELTA_DEG;

  if (!changed) return;
  saveResumePoseNow();
}

void clearMacroSlots() {
  for (int i = 0; i < MAX_MACROS; i++) {
    macros[i].used = false;
    macros[i].name = "";
    macros[i].body = STARTUP_BODY;
    macros[i].shoulder = STARTUP_SHOULDER;
    macros[i].elbow = STARTUP_ELBOW;
    macros[i].gripper = STARTUP_GRIPPER;
  }
}

void loadMacros() {
  clearMacroSlots();
  prefs.begin("macros", true);
  int count = prefs.getInt("count", 0);
  count = clampInt(count, 0, MAX_MACROS);
  for (int i = 0; i < count; i++) {
    String base = "m" + String(i) + "_";
    String name = prefs.getString((base + "name").c_str(), "");
    if (name.length() == 0) continue;
    macros[i].used = true;
    macros[i].name = name;
    macros[i].body = prefs.getInt((base + "body").c_str(), STARTUP_BODY);
    macros[i].shoulder = prefs.getInt((base + "shoulder").c_str(), STARTUP_SHOULDER);
    macros[i].elbow = prefs.getInt((base + "elbow").c_str(), STARTUP_ELBOW);
    macros[i].gripper = prefs.getInt((base + "gripper").c_str(), STARTUP_GRIPPER);
  }
  prefs.end();
}

void saveMacros() {
  prefs.begin("macros", false);
  int writeIdx = 0;
  for (int i = 0; i < MAX_MACROS; i++) {
    if (!macros[i].used) continue;
    String base = "m" + String(writeIdx) + "_";
    prefs.putString((base + "name").c_str(), macros[i].name);
    prefs.putInt((base + "body").c_str(), macros[i].body);
    prefs.putInt((base + "shoulder").c_str(), macros[i].shoulder);
    prefs.putInt((base + "elbow").c_str(), macros[i].elbow);
    prefs.putInt((base + "gripper").c_str(), macros[i].gripper);
    writeIdx++;
  }
  prefs.putInt("count", writeIdx);
  for (int i = writeIdx; i < MAX_MACROS; i++) {
    String base = "m" + String(i) + "_";
    prefs.remove((base + "name").c_str());
    prefs.remove((base + "body").c_str());
    prefs.remove((base + "shoulder").c_str());
    prefs.remove((base + "elbow").c_str());
    prefs.remove((base + "gripper").c_str());
  }
  prefs.end();
}

int findMacroByName(const String& name) {
  for (int i = 0; i < MAX_MACROS; i++) {
    if (macros[i].used && macros[i].name == name) return i;
  }
  return -1;
}

int findFreeMacroSlot() {
  for (int i = 0; i < MAX_MACROS; i++) {
    if (!macros[i].used) return i;
  }
  return -1;
}

bool upsertMacro(const String& name, int body, int shoulder, int elbow, int gripper, String& detail) {
  if (name.length() == 0) {
    detail = "macro name required";
    return false;
  }
  int idx = findMacroByName(name);
  if (idx < 0) idx = findFreeMacroSlot();
  if (idx < 0) {
    detail = "macro storage full";
    return false;
  }
  macros[idx].used = true;
  macros[idx].name = name;
  macros[idx].body = clampInt(body, BODY_MIN, BODY_MAX);
  macros[idx].shoulder = clampInt(shoulder, SHOULDER_MIN, SHOULDER_MAX);
  macros[idx].elbow = clampInt(elbow, ELBOW_MIN, ELBOW_MAX);
  macros[idx].gripper = clampInt(gripper, GRIPPER_MIN, GRIPPER_MAX);
  saveMacros();
  detail = "macro saved";
  return true;
}

bool deleteMacroByName(const String& name, String& detail) {
  int idx = findMacroByName(name);
  if (idx < 0) {
    detail = "macro not found";
    return false;
  }
  macros[idx].used = false;
  macros[idx].name = "";
  saveMacros();
  detail = "macro deleted";
  return true;
}

void attachServosIfNeeded() {
  if (servosAttached) return;
  servoBody.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);
  servoBody.attach(PIN_BODY, 544, 2400);
  servoShoulder.attach(PIN_SHOULDER, 544, 2400);
  servoElbow.attach(PIN_ELBOW, 544, 2400);
  servoGripper.attach(PIN_GRIPPER, 544, 2400);
  servosAttached = true;
}

void waitWithBackground(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (mqtt.connected()) mqtt.loop();
    http.handleClient();
    delay(5);
  }
}

void moveSingleServoBlocking(Servo& servo, int& cur, int target, int minVal, int maxVal) {
  target = clampInt(target, minVal, maxVal);
  while (cur != target) {
    int diff = target - cur;
    if (diff > 0) cur += min(SERVO_STEP_DEG, diff);
    else cur -= min(SERVO_STEP_DEG, -diff);
    cur = clampInt(cur, minVal, maxVal);
    servo.write(cur);
    if (mqtt.connected()) mqtt.loop();
    http.handleClient();
    delay(SERVO_TICK_MS);
  }
  waitWithBackground(START_MOVE_SETTLE_MS);
}

void runStartupSequence() {
  attachServosIfNeeded();

  if (resumePoseValid) {
    servoBody.write(curBody);
    servoShoulder.write(curShoulder);
    servoElbow.write(curElbow);
    servoGripper.write(curGripper);
    waitWithBackground(START_MOVE_SETTLE_MS);
  } else {
    moveSingleServoBlocking(servoElbow,    curElbow,    STARTUP_ELBOW,    ELBOW_MIN,    ELBOW_MAX);
    moveSingleServoBlocking(servoShoulder, curShoulder, STARTUP_SHOULDER, SHOULDER_MIN, SHOULDER_MAX);
    moveSingleServoBlocking(servoBody,     curBody,     STARTUP_BODY,     BODY_MIN,     BODY_MAX);
    moveSingleServoBlocking(servoGripper,  curGripper,  STARTUP_GRIPPER,  GRIPPER_MIN,  GRIPPER_MAX);
  }

  tgtAbsBody = curBody;
  tgtAbsShoulder = curShoulder;
  tgtAbsElbow = curElbow;
  tgtAbsGripper = curGripper;

  zeroAbsBody = curBody;
  zeroAbsShoulder = curShoulder;
  zeroAbsElbow = curElbow;
  zeroAbsGripper = curGripper;

  resetRelativeModel();
  lastServoTick = millis();
  saveResumePoseNow();
}

void stepServoToward(Servo& s, int& cur, int tgt) {
  int diff = tgt - cur;
  if (abs(diff) <= SERVO_DEADBAND_DEG) return;
  if (diff > 0) cur += min(SERVO_STEP_DEG, diff);
  else cur -= min(SERVO_STEP_DEG, -diff);
  cur = clampInt(cur, 0, 180);
  s.write(cur);
}

void serviceServos() {
  if (!servosAttached) return;
  uint32_t now = millis();
  if (now - lastServoTick < SERVO_TICK_MS) return;
  lastServoTick = now;

  stepServoToward(servoBody, curBody, tgtAbsBody);
  stepServoToward(servoShoulder, curShoulder, tgtAbsShoulder);
  stepServoToward(servoElbow, curElbow, tgtAbsElbow);
  stepServoToward(servoGripper, curGripper, tgtAbsGripper);

  relBody     = curBody - zeroAbsBody;
  relShoulder = curShoulder - zeroAbsShoulder;
  relElbow    = curElbow - zeroAbsElbow;
  relGripper  = curGripper - zeroAbsGripper;
}

void setTargetsRelative(int b, int s, int e, int g) {
  tgtRelBody     = clampRel(b, limNegBody, limPosBody);
  tgtRelShoulder = clampRel(s, limNegShoulder, limPosShoulder);
  tgtRelElbow    = clampRel(e, limNegElbow, limPosElbow);
  tgtRelGripper  = clampRel(g, limNegGripper, limPosGripper);

  tgtAbsBody     = clampInt(zeroAbsBody + tgtRelBody + BODY_OFFSET, BODY_MIN, BODY_MAX);
  tgtAbsShoulder = clampInt(zeroAbsShoulder + tgtRelShoulder + SHOULDER_OFFSET, SHOULDER_MIN, SHOULDER_MAX);
  tgtAbsElbow    = clampInt(zeroAbsElbow + tgtRelElbow + ELBOW_OFFSET, ELBOW_MIN, ELBOW_MAX);
  tgtAbsGripper  = clampInt(zeroAbsGripper + tgtRelGripper + GRIPPER_OFFSET, GRIPPER_MIN, GRIPPER_MAX);
}

void setTargetsAbsolute(int body, int shoulder, int elbow, int gripper) {
  tgtAbsBody = clampInt(body, BODY_MIN, BODY_MAX);
  tgtAbsShoulder = clampInt(shoulder, SHOULDER_MIN, SHOULDER_MAX);
  tgtAbsElbow = clampInt(elbow, ELBOW_MIN, ELBOW_MAX);
  tgtAbsGripper = clampInt(gripper, GRIPPER_MIN, GRIPPER_MAX);
}

bool solveIK2D(float x, float y, float& shoulderRelDeg, float& elbowRelDeg, bool elbowUp = false) {
  float r2 = x * x + y * y;
  float r = sqrtf(r2);
  float minReach = fabsf(L1 - L2);
  float maxReach = L1 + L2;
  if (r < minReach || r > maxReach) return false;
  float c2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  c2 = clampFloat(c2, -1.0f, 1.0f);
  float s2 = sqrtf(fmaxf(0.0f, 1.0f - c2 * c2));
  if (elbowUp) s2 = -s2;
  float theta2 = atan2f(s2, c2);
  float k1 = L1 + L2 * c2;
  float k2 = L2 * s2;
  float theta1 = atan2f(y, x) - atan2f(k2, k1);
  shoulderRelDeg = theta1 * 180.0f / PI;
  elbowRelDeg = theta2 * 180.0f / PI;
  return true;
}

bool extractNumber(const String& s, const char* key, float& outVal) {
  String k = "\"" + String(key) + "\"";
  int i = s.indexOf(k);
  if (i < 0) return false;
  int colon = s.indexOf(':', i + k.length());
  if (colon < 0) return false;
  int start = colon + 1;
  while (start < (int)s.length() && (s[start] == ' ' || s[start] == '"')) start++;
  int end = start;
  while (end < (int)s.length()) {
    char c = s[end];
    if (!(isdigit(c) || c == '-' || c == '+' || c == '.')) break;
    end++;
  }
  if (end <= start) return false;
  outVal = s.substring(start, end).toFloat();
  return true;
}

bool extractString(const String& s, const char* key, String& outVal) {
  String k = "\"" + String(key) + "\"";
  int i = s.indexOf(k);
  if (i < 0) return false;
  int colon = s.indexOf(':', i + k.length());
  if (colon < 0) return false;
  int q1 = s.indexOf('"', colon + 1);
  if (q1 < 0) return false;
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  outVal = s.substring(q1 + 1, q2);
  return true;
}

bool extractBool(const String& s, const char* key, bool& outVal) {
  String k = "\"" + String(key) + "\"";
  int i = s.indexOf(k);
  if (i < 0) return false;
  int colon = s.indexOf(':', i + k.length());
  if (colon < 0) return false;
  String tail = s.substring(colon + 1);
  tail.trim();
  if (tail.startsWith("true")) { outVal = true; return true; }
  if (tail.startsWith("false")) { outVal = false; return true; }
  return false;
}

bool setAxisLimit(const String& axis, bool positive, int value) {
  if (axis == "body") {
    if (positive) limPosBody = value; else limNegBody = value;
    return limNegBody <= limPosBody;
  }
  if (axis == "shoulder") {
    if (positive) limPosShoulder = value; else limNegShoulder = value;
    return limNegShoulder <= limPosShoulder;
  }
  if (axis == "elbow") {
    if (positive) limPosElbow = value; else limNegElbow = value;
    return limNegElbow <= limPosElbow;
  }
  if (axis == "gripper") {
    if (positive) limPosGripper = value; else limNegGripper = value;
    return limNegGripper <= limPosGripper;
  }
  return false;
}

void handleCommandJson(const String& msg) {
  String mode;
  if (!extractString(msg, "mode", mode)) {
    publishState("error", "missing mode");
    return;
  }

  if (mode == "status") {
    publishState("ok", "status");
    return;
  }

  if (mode == "move_abs") {
    float b, s, e, g;
    if (!extractNumber(msg, "body", b) || !extractNumber(msg, "shoulder", s) || !extractNumber(msg, "elbow", e) || !extractNumber(msg, "gripper", g)) {
      publishState("error", "move_abs requires body/shoulder/elbow/gripper");
      return;
    }
    setTargetsAbsolute((int)roundf(b), (int)roundf(s), (int)roundf(e), (int)roundf(g));
    publishState("ok", "move_abs target set");
    return;
  }

  if (mode == "move_rel") {
    float b = tgtRelBody, s = tgtRelShoulder, e = tgtRelElbow, g = tgtRelGripper;
    extractNumber(msg, "body", b);
    extractNumber(msg, "shoulder", s);
    extractNumber(msg, "elbow", e);
    extractNumber(msg, "gripper", g);
    setTargetsRelative((int)roundf(b), (int)roundf(s), (int)roundf(e), (int)roundf(g));
    publishState("ok", "move_rel target set");
    return;
  }

  if (mode == "jog_rel") {
    float db = 0, ds = 0, de = 0, dg = 0;
    extractNumber(msg, "body", db);
    extractNumber(msg, "shoulder", ds);
    extractNumber(msg, "elbow", de);
    extractNumber(msg, "gripper", dg);
    setTargetsRelative(
      tgtRelBody + (int)roundf(db),
      tgtRelShoulder + (int)roundf(ds),
      tgtRelElbow + (int)roundf(de),
      tgtRelGripper + (int)roundf(dg)
    );
    publishState("ok", "jog_rel target set");
    return;
  }

  if (mode == "ik") {
    float x, y;
    if (!extractNumber(msg, "x", x) || !extractNumber(msg, "y", y)) {
      publishState("error", "ik missing x/y");
      return;
    }
    bool elbowUp = false;
    extractBool(msg, "elbowUp", elbowUp);
    float b = tgtRelBody, g = tgtRelGripper;
    extractNumber(msg, "body", b);
    extractNumber(msg, "gripper", g);
    float sRel, eRel;
    if (!solveIK2D(x, y, sRel, eRel, elbowUp)) {
      publishState("error", "ik target unreachable");
      return;
    }
    setTargetsRelative((int)roundf(b), (int)roundf(sRel), (int)roundf(eRel), (int)roundf(g));
    publishState("ok", "ik target set");
    return;
  }

  if (mode == "set_limit_pos" || mode == "set_limit_neg") {
    String axis;
    float v;
    if (!extractString(msg, "axis", axis) || !extractNumber(msg, "value", v)) {
      publishState("error", "set_limit missing axis/value");
      return;
    }
    bool positive = (mode == "set_limit_pos");
    if (!setAxisLimit(axis, positive, (int)roundf(v))) {
      publishState("error", "invalid axis or neg>pos");
      return;
    }
    publishState("ok", "limit updated");
    return;
  }

  if (mode == "save_limits") {
    saveLimits();
    publishState("ok", "limits saved");
    return;
  }

  publishState("error", "unknown mode");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT RX [");
  Serial.print(topicStr);
  Serial.print("]: ");
  Serial.println(msg);
  if (topicStr != TOPIC_CMD) {
    publishState("error", "message on unexpected topic");
    return;
  }
  handleCommandJson(msg);
}

void addCors() {
  http.sendHeader("Access-Control-Allow-Origin", "*");
  http.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  http.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendJson(int code, const String& body) {
  addCors();
  http.send(code, "application/json", body);
}

void handleOptions() {
  addCors();
  http.send(204);
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Finnis Robotarm</title>
  <style>
    body{font-family:system-ui,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:20px}
    h1,h2{margin:0 0 12px}
    .grid{display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
    .card{background:#1b1b1b;border:1px solid #333;border-radius:12px;padding:16px}
    label{display:block;margin:8px 0 4px}
    input[type=number],input[type=text]{width:100%;padding:10px;border-radius:8px;border:1px solid #444;background:#0f0f0f;color:#fff;box-sizing:border-box}
    button{padding:10px 14px;border-radius:8px;border:0;background:#2d6cdf;color:#fff;cursor:pointer;margin:6px 6px 0 0}
    button.danger{background:#a33}
    button.secondary{background:#444}
    .mono{font-family:ui-monospace,SFMono-Regular,monospace;white-space:pre-wrap}
    .macro{border-top:1px solid #333;padding-top:10px;margin-top:10px}
    .row{display:flex;gap:8px;flex-wrap:wrap}
    .sliders-card{grid-column:1/-1}
    .u-wrap{display:grid;grid-template-columns:1fr 1fr;gap:28px;align-items:end;justify-items:center;margin:8px 0 18px}
    .vertical-wrap{display:flex;flex-direction:column;align-items:center;gap:10px}
    .vertical{appearance:slider-vertical;-webkit-appearance:slider-vertical;width:28px;height:260px;background:transparent}
    input[type=range].vertical{writing-mode:bt-lr}
    .horizontal-wrap{display:flex;flex-direction:column;align-items:center;gap:10px}
    .horizontal{width:min(520px,100%)}
    .compact .row2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
    .small{font-size:12px;color:#bbb}
    @supports not (-webkit-appearance: slider-vertical) {
      .vertical{width:260px;height:28px;transform:rotate(-90deg)}
      .vertical-wrap{padding:120px 0}
    }
  </style>
</head>
<body>
  <h1>Finnis Robotarm</h1>
  <div class="grid">
    <div class="card">
      <h2>Status</h2>
      <div id="status" class="mono">lädt…</div>
      <button onclick="refreshState()">Aktualisieren</button>
    </div>

    <div class="card compact">
      <h2>Makro speichern</h2>
      <label>Name</label><input id="macroName" type="text" placeholder="z. B. park">
      <div class="row">
        <button onclick="saveMacroFromInputs()">Eingegebene Position speichern</button>
        <button onclick="captureCurrentMacro()">Aktuelle Position speichern</button>
      </div>
    </div>

    <div class="card sliders-card">
      <h2>Fließende Steuerung</h2>
      <div class="small">U-Form: links Elbow, rechts Shoulder, unten Body. Änderungen fahren direkt los.</div>
      <div class="u-wrap">
        <div class="vertical-wrap">
          <div>Elbow: <span id="elbowVal">90</span></div>
          <input id="elbowSlider" class="vertical" type="range" min="5" max="175" value="90" oninput="sliderChanged()">
        </div>
        <div class="vertical-wrap">
          <div>Shoulder: <span id="shoulderVal">90</span></div>
          <input id="shoulderSlider" class="vertical" type="range" min="5" max="175" value="90" oninput="sliderChanged()">
        </div>
      </div>
      <div class="horizontal-wrap">
        <div>Body: <span id="bodyVal">0</span></div>
        <input id="bodySlider" class="horizontal" type="range" min="0" max="180" value="0" oninput="sliderChanged()">
      </div>
    </div>

    <div class="card compact">
      <h2>Absolute Position fahren</h2>
      <div class="row2">
        <div><label>Body</label><input id="body" type="number" min="0" max="180" value="0"></div>
        <div><label>Shoulder</label><input id="shoulder" type="number" min="5" max="175" value="90"></div>
        <div><label>Elbow</label><input id="elbow" type="number" min="5" max="175" value="90"></div>
        <div><label>Gripper</label><input id="gripper" type="number" min="10" max="170" value="90"></div>
      </div>
      <div class="row">
        <button onclick="moveAbs()">Position fahren</button>
        <button class="secondary" onclick="fillCurrent()">Aktuelle Position übernehmen</button>
      </div>
    </div>

    <div class="card">
      <h2>Makros</h2>
      <div id="macros">lädt…</div>
    </div>
  </div>

<script>
let moveTimer = null;
let suppressSliderSync = false;

async function api(path, options={}) {
  const res = await fetch(path, Object.assign({headers:{'Content-Type':'application/json'}}, options));
  return res.json();
}

function setInputsFromPose(p){
  document.getElementById('body').value = p.body;
  document.getElementById('shoulder').value = p.shoulder;
  document.getElementById('elbow').value = p.elbow;
  document.getElementById('gripper').value = p.gripper;
}

function setSliderPose(p){
  suppressSliderSync = true;
  document.getElementById('bodySlider').value = p.body;
  document.getElementById('shoulderSlider').value = p.shoulder;
  document.getElementById('elbowSlider').value = p.elbow;
  document.getElementById('bodyVal').textContent = p.body;
  document.getElementById('shoulderVal').textContent = p.shoulder;
  document.getElementById('elbowVal').textContent = p.elbow;
  suppressSliderSync = false;
}

async function refreshState(){
  const s = await api('/api/state');
  document.getElementById('status').textContent = JSON.stringify(s, null, 2);
  renderMacros(s.macros || []);
  setSliderPose(s.curAbs);
}

function renderMacros(macros){
  const root = document.getElementById('macros');
  if (!macros.length) {
    root.innerHTML = '<div>Keine Makros gespeichert.</div>';
    return;
  }
  root.innerHTML = macros.map(m => `
    <div class="macro">
      <div><strong>${m.name}</strong></div>
      <div class="mono">body=${m.body}, shoulder=${m.shoulder}, elbow=${m.elbow}, gripper=${m.gripper}</div>
      <div class="row">
        <button onclick='runMacro(${JSON.stringify(JSON.stringify(m.name))})'>Ausführen</button>
        <button class="secondary" onclick='loadMacroIntoInputs(${JSON.stringify(JSON.stringify(m))})'>In Felder laden</button>
        <button class="danger" onclick='deleteMacro(${JSON.stringify(JSON.stringify(m.name))})'>Löschen</button>
      </div>
    </div>
  `).join('');
}

function getInputPose(){
  return {
    body: Number(document.getElementById('body').value),
    shoulder: Number(document.getElementById('shoulder').value),
    elbow: Number(document.getElementById('elbow').value),
    gripper: Number(document.getElementById('gripper').value)
  };
}

function getSliderPose(){
  return {
    body: Number(document.getElementById('bodySlider').value),
    shoulder: Number(document.getElementById('shoulderSlider').value),
    elbow: Number(document.getElementById('elbowSlider').value),
    gripper: Number(document.getElementById('gripper').value)
  };
}

async function moveAbs(){
  const p = getInputPose();
  await api('/api/move_abs', {method:'POST', body: JSON.stringify(p)});
  setSliderPose(p);
  refreshState();
}

function sliderChanged(){
  document.getElementById('bodyVal').textContent = document.getElementById('bodySlider').value;
  document.getElementById('shoulderVal').textContent = document.getElementById('shoulderSlider').value;
  document.getElementById('elbowVal').textContent = document.getElementById('elbowSlider').value;
  if (suppressSliderSync) return;
  const p = getSliderPose();
  document.getElementById('body').value = p.body;
  document.getElementById('shoulder').value = p.shoulder;
  document.getElementById('elbow').value = p.elbow;
  if (moveTimer) clearTimeout(moveTimer);
  moveTimer = setTimeout(async () => {
    await api('/api/move_abs', {method:'POST', body: JSON.stringify(p)});
    refreshState();
  }, 80);
}

async function saveMacroFromInputs(){
  const name = document.getElementById('macroName').value.trim();
  if (!name) return alert('Bitte Makronamen eingeben');
  const p = getInputPose();
  await api('/api/macros/save', {method:'POST', body: JSON.stringify({name, ...p})});
  refreshState();
}

async function captureCurrentMacro(){
  const name = document.getElementById('macroName').value.trim();
  if (!name) return alert('Bitte Makronamen eingeben');
  await api('/api/macros/capture', {method:'POST', body: JSON.stringify({name})});
  refreshState();
}

async function runMacro(nameJson){
  const name = JSON.parse(nameJson);
  await api('/api/macros/run', {method:'POST', body: JSON.stringify({name})});
  refreshState();
}

async function deleteMacro(nameJson){
  const name = JSON.parse(nameJson);
  await api('/api/macros/delete', {method:'POST', body: JSON.stringify({name})});
  refreshState();
}

function loadMacroIntoInputs(macroJson){
  const m = JSON.parse(macroJson);
  setInputsFromPose(m);
  setSliderPose(m);
  document.getElementById('macroName').value = m.name;
}

async function fillCurrent(){
  const s = await api('/api/state');
  setInputsFromPose(s.curAbs);
  setSliderPose(s.curAbs);
}

refreshState();
setInterval(refreshState, 2000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  addCors();
  http.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleScan() {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"name\":\"%s\",\"ip\":\"%s\",\"api\":\"http://%s/api\"}",
           DEVICE_NAME,
           WiFi.localIP().toString().c_str(),
           WiFi.localIP().toString().c_str());
  sendJson(200, String(buf));
}

void handleStateHttp() {
  sendJson(200, buildStateJson());
}

void handleCmdHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  handleCommandJson(body);
  sendJson(200, buildStateJson());
}

void handleMoveAbsHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  float b, s, e, g;
  if (!extractNumber(body, "body", b) || !extractNumber(body, "shoulder", s) || !extractNumber(body, "elbow", e) || !extractNumber(body, "gripper", g)) {
    sendJson(400, "{\"error\":\"body/shoulder/elbow/gripper required\"}");
    return;
  }
  setTargetsAbsolute((int)roundf(b), (int)roundf(s), (int)roundf(e), (int)roundf(g));
  publishState("ok", "move_abs target set via browser");
  sendJson(200, buildStateJson());
}

void handleMacrosHttp() {
  sendJson(200, "{\"macros\":" + buildMacrosJsonArray() + "}");
}

void handleMacroSaveHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  String name;
  float b, s, e, g;
  if (!extractString(body, "name", name) || !extractNumber(body, "body", b) || !extractNumber(body, "shoulder", s) || !extractNumber(body, "elbow", e) || !extractNumber(body, "gripper", g)) {
    sendJson(400, "{\"error\":\"name/body/shoulder/elbow/gripper required\"}");
    return;
  }
  String detail;
  if (!upsertMacro(name, (int)roundf(b), (int)roundf(s), (int)roundf(e), (int)roundf(g), detail)) {
    sendJson(400, "{\"error\":\"" + jsonEscape(detail) + "\"}");
    return;
  }
  publishState("ok", detail.c_str());
  sendJson(200, "{\"ok\":true,\"detail\":\"" + jsonEscape(detail) + "\",\"macros\":" + buildMacrosJsonArray() + "}");
}

void handleMacroCaptureHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  String name;
  if (!extractString(body, "name", name)) {
    sendJson(400, "{\"error\":\"name required\"}");
    return;
  }
  String detail;
  if (!upsertMacro(name, curBody, curShoulder, curElbow, curGripper, detail)) {
    sendJson(400, "{\"error\":\"" + jsonEscape(detail) + "\"}");
    return;
  }
  publishState("ok", "macro captured from current position");
  sendJson(200, "{\"ok\":true,\"macros\":" + buildMacrosJsonArray() + "}");
}

void handleMacroRunHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  String name;
  if (!extractString(body, "name", name)) {
    sendJson(400, "{\"error\":\"name required\"}");
    return;
  }
  int idx = findMacroByName(name);
  if (idx < 0) {
    sendJson(404, "{\"error\":\"macro not found\"}");
    return;
  }
  setTargetsAbsolute(macros[idx].body, macros[idx].shoulder, macros[idx].elbow, macros[idx].gripper);
  publishState("ok", "macro target set");
  sendJson(200, buildStateJson());
}

void handleMacroDeleteHttp() {
  if (!http.hasArg("plain")) {
    sendJson(400, "{\"error\":\"missing json body\"}");
    return;
  }
  String body = http.arg("plain");
  String name;
  if (!extractString(body, "name", name)) {
    sendJson(400, "{\"error\":\"name required\"}");
    return;
  }
  String detail;
  if (!deleteMacroByName(name, detail)) {
    sendJson(404, "{\"error\":\"" + jsonEscape(detail) + "\"}");
    return;
  }
  publishState("ok", detail.c_str());
  sendJson(200, "{\"ok\":true,\"macros\":" + buildMacrosJsonArray() + "}");
}

void setupHttp() {
  http.on("/", HTTP_GET, handleRoot);
  http.on("/api/scan", HTTP_GET, handleScan);
  http.on("/api/state", HTTP_GET, handleStateHttp);
  http.on("/api/cmd", HTTP_POST, handleCmdHttp);
  http.on("/api/move_abs", HTTP_POST, handleMoveAbsHttp);
  http.on("/api/macros", HTTP_GET, handleMacrosHttp);
  http.on("/api/macros/save", HTTP_POST, handleMacroSaveHttp);
  http.on("/api/macros/capture", HTTP_POST, handleMacroCaptureHttp);
  http.on("/api/macros/run", HTTP_POST, handleMacroRunHttp);
  http.on("/api/macros/delete", HTTP_POST, handleMacroDeleteHttp);

  http.on("/", HTTP_OPTIONS, handleOptions);
  http.on("/api/scan", HTTP_OPTIONS, handleOptions);
  http.on("/api/state", HTTP_OPTIONS, handleOptions);
  http.on("/api/cmd", HTTP_OPTIONS, handleOptions);
  http.on("/api/move_abs", HTTP_OPTIONS, handleOptions);
  http.on("/api/macros", HTTP_OPTIONS, handleOptions);
  http.on("/api/macros/save", HTTP_OPTIONS, handleOptions);
  http.on("/api/macros/capture", HTTP_OPTIONS, handleOptions);
  http.on("/api/macros/run", HTTP_OPTIONS, handleOptions);
  http.on("/api/macros/delete", HTTP_OPTIONS, handleOptions);

  http.onNotFound([]() {
    sendJson(404, "{\"error\":\"not found\"}");
  });

  http.begin();
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(250);
  }
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;

  uint32_t now = millis();
  if (now - lastMqttAttemptMs < MQTT_RETRY_MS) return;
  lastMqttAttemptMs = now;

  bool ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, TOPIC_AVAIL, 1, true, "offline");
  if (ok) {
    mqtt.publish(TOPIC_AVAIL, "online", true);
    mqtt.subscribe(TOPIC_CMD);
    publishState("ready", "mqtt connected");
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  servosAttached = false;

  clearMacroSlots();
  loadLimits();
  loadMacros();
  loadResumePose();
  ensureWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  setupHttp();
  runStartupSequence();

  publishState("ready", resumePoseValid ? "booted, resumed pose, ui ready" : "booted, fallback startup, ui ready");
}

void loop() {
  ensureWiFi();
  ensureMqtt();
  if (mqtt.connected()) mqtt.loop();
  http.handleClient();
  serviceServos();
  maybeSaveResumePose();
}
