/*
  4-DOF Wi-Fi Web Control Motion Repeater Robotic Arm (ESP32)

  Fixed / hardened version:
   - All robot state is owned by loop(); the async WebSocket callback only queues
     commands (FreeRTOS queue) -> no data races / use-after-free on the recording.
   - Fully non-blocking playback state machine (no delay()) -> Stop is instant.
   - Per-joint angle limits + strict input parsing -> bad input can't slam servos.
   - Speed-limited (smoothed) servo motion for live control, homing and playback.
   - Bounded recording (MAX_STEPS) stored compactly and persisted in NVS flash.
   - PIN authentication per WebSocket client, failed-attempt lockout, client cap.
   - Unique AP SSID per board; state broadcast to every connected client.
   - Same command set also accepted on the Serial port (115200 baud).

  Serial / WebSocket protocol (client -> board), one command per line/frame:
     Auth,<pin>          Move,<Joint>,<angle>     Record,<0|1>
     Play,<0|1>          Home,1                   Status
  Board -> client:
     Auth,REQUIRED|OK|FAIL       Joint,<name>,<min>,<max>,<pos>
     Pos,<name>,<pos>            State,Record|Play,ON|OFF
     Steps,<count>               Info,<text>

  Libraries (Arduino Library Manager):
     "ESP Async WebServer" + "Async TCP" (ESP32Async), "ESP32Servo" (Kevin Harrington)
*/

// 1 = normal (Wi-Fi hotspot + web page). 0 = Serial-only test mode, e.g. for the Wokwi
// simulator, whose Wi-Fi model does not support hotspot (AP) mode.
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1
#endif

#include <Arduino.h>
#if ENABLE_WIFI
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif
#include <ESP32Servo.h>
#include <Preferences.h>
#include <vector>
#include <algorithm>

// ============================ USER CONFIGURATION ============================
// CHANGE THESE before deploying. Wi-Fi password must be at least 8 characters.
#define AP_SSID_PREFIX   "RobotArm-"      // MAC suffix is appended -> unique per board
#define AP_PASSWORD      "ChangeMe-Arm42" // WPA2 password for the hotspot
#define CONTROL_PIN      "4321"           // PIN needed to control the arm ("" = no PIN)

static const uint8_t  MAX_WS_CLIENTS      = 4;      // simultaneous browser connections
static const uint8_t  MAX_AUTH_FAILS      = 5;      // wrong PINs before disconnect
static const size_t   MAX_STEPS           = 1000;   // recording capacity (4 bytes/step)
static const uint16_t MAX_STEP_DELAY_MS   = 60000;  // longer pauses are shortened to this
static const float    LIVE_SPEED_DEG_S    = 180.0f; // max speed for slider control/playback
static const float    HOMING_SPEED_DEG_S  = 20.0f;  // slow speed when returning to start pose
static const uint32_t PLAY_START_PAUSE_MS = 2000;   // pause after homing, before steps
static const uint32_t MOTION_TICK_MS      = 10;     // servo update period
static const uint32_t POS_BROADCAST_MS    = 60;     // position updates to browsers

struct Joint
{
  const char* name;
  uint8_t     pin;
  uint8_t     minAngle;   // mechanical safety limits - calibrate for YOUR arm!
  uint8_t     maxAngle;
  uint8_t     homeAngle;
  Servo       servo;
  float       current;    // smoothed position actually written
  uint8_t     target;     // where we want to go
  int16_t     lastWritten;
  int16_t     lastSent;
};

Joint joints[] =
{
  // name        pin  min  max  home
  { "Base",      27,    0, 180,  90 },
  { "Shoulder",  26,   15, 165,  90 },
  { "Elbow",     25,   15, 165,  90 },
  { "Gripper",   33,   30, 150,  90 },
};
static const uint8_t NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);
// ===========================================================================

struct __attribute__((packed)) RecordedStep
{
  uint8_t  joint;
  uint8_t  angle;
  uint16_t delayMs;   // time since previous step
};

std::vector<RecordedStep> recordedSteps;
uint8_t  recordedStartPose[NUM_JOINTS];
uint32_t lastRecordTimeMs = 0;

enum class Mode : uint8_t { Idle, Recording, PlayHoming, PlayPause, PlaySteps };
Mode     mode = Mode::Idle;
size_t   playIndex = 0;
uint32_t playTimerMs = 0;
float    currentSpeed = LIVE_SPEED_DEG_S;

// ---- command queue: async callbacks -> loop() ----
enum class CmdType : uint8_t { Text, Connect, Disconnect };
struct Command
{
  CmdType  type;
  uint32_t clientId;          // 0 = Serial
  char     text[64];
};
QueueHandle_t commandQueue;

struct ClientInfo { uint32_t id; bool authed; uint8_t fails; };
ClientInfo clients[MAX_WS_CLIENTS];

Preferences prefs;

#if ENABLE_WIFI
AsyncWebServer server(80);
AsyncWebSocket ws("/RobotArmInput");

const char htmlHomePage[] PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Arm Control</title>
<style>
  *{box-sizing:border-box}
  body{font-family:system-ui,Arial,sans-serif;margin:0;padding:16px;background:#fff;color:#123;
       -webkit-user-select:none;user-select:none;touch-action:manipulation}
  main{max-width:460px;margin:auto}
  h1{color:teal;text-align:center;font-size:1.5em;margin:.2em 0}
  h2{color:teal;text-align:center;font-size:1.1em;margin:.2em 0 1em}
  #status{text-align:center;padding:6px;border-radius:8px;margin-bottom:12px;background:#eee}
  #status.ok{background:#d9f2e3}#status.bad{background:#f8d7da}
  .row{display:flex;align-items:center;gap:10px;margin:14px 0}
  .row label{width:95px;font-weight:bold;font-size:1.1em}
  .row output{width:42px;text-align:right;font-variant-numeric:tabular-nums}
  input[type=range]{flex:1;-webkit-appearance:none;appearance:none;height:20px;border-radius:5px;background:#d3d3d3}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:36px;height:36px;border-radius:50%;background:red;cursor:pointer}
  input[type=range]::-moz-range-thumb{width:36px;height:36px;border-radius:50%;background:red;cursor:pointer}
  input:disabled{opacity:.4}
  .btns{display:flex;gap:10px;margin-top:20px}
  button{flex:1;height:48px;border:0;border-radius:24px;font-size:1.05em;color:#fff;background:#c62828;cursor:pointer}
  button.on{background:#2e7d32} button.plain{background:#455a64} button:disabled{opacity:.4;cursor:default}
  #pinBox{display:none;text-align:center;margin:12px 0}
  #pinBox input{font-size:1.2em;width:8em;padding:6px;text-align:center}
  #pinBox button{flex:none;width:7em;margin-left:6px}
  #info{text-align:center;min-height:1.4em;color:#555;margin-top:12px}
</style>
</head>
<body>
<main>
  <h1>Hash Include Electronics</h1>
  <h2>Robot Arm Control</h2>
  <div id="status">Connecting...</div>
  <div id="pinBox"><input id="pin" type="password" inputmode="numeric" placeholder="PIN"><button class="plain" id="pinBtn">Unlock</button></div>
  <div id="joints"></div>
  <div class="btns">
    <button id="Record">Record: OFF</button>
    <button id="Play">Play: OFF</button>
    <button id="Home" class="plain">Home</button>
  </div>
  <div id="steps" style="text-align:center;margin-top:8px;color:#555"></div>
  <div id="info"></div>
</main>
<script>
var ws, authed = false, recording = false, playing = false;
var sliders = {}, lastUserInput = {}, sendTimers = {}, lastSend = {};
var $ = function(id){ return document.getElementById(id); };

function setStatus(text, cls){ var s=$("status"); s.textContent=text; s.className=cls||""; }
function send(msg){ if (ws && ws.readyState === WebSocket.OPEN) ws.send(msg); }
function savedPin(){ try { return sessionStorage.getItem("armPin") || ""; } catch(e){ return ""; } }
function storePin(p){ try { sessionStorage.setItem("armPin", p); } catch(e){} }

function connect(){
  ws = new WebSocket("ws://" + location.host + "/RobotArmInput");
  ws.onopen = function(){ setStatus("Connected", "ok"); };
  ws.onclose = function(){ authed=false; setStatus("Disconnected - retrying...", "bad"); refresh(); setTimeout(connect, 2000); };
  ws.onerror = function(){ try{ ws.close(); }catch(e){} };
  ws.onmessage = function(ev){ handle(String(ev.data).split(",")); };
}

function handle(p){
  switch(p[0]){
    case "Auth":
      if (p[1] === "OK"){ authed = true; $("pinBox").style.display="none"; setStatus("Connected - control unlocked","ok"); }
      else {
        authed = false; $("pinBox").style.display="block";
        if (p[1] === "FAIL"){ storePin(""); setStatus("Wrong PIN","bad"); }
        else if (savedPin()) send("Auth," + savedPin());
      }
      refresh(); break;
    case "Joint": makeSlider(p[1], +p[2], +p[3], +p[4]); break;
    case "Pos": updateSlider(p[1], +p[2]); break;
    case "State":
      if (p[1] === "Record") recording = (p[2] === "ON");
      if (p[1] === "Play")   playing   = (p[2] === "ON");
      refresh(); break;
    case "Steps": $("steps").textContent = p[1] + " recorded steps"; break;
    case "Info": $("info").textContent = p.slice(1).join(","); break;
  }
}

function makeSlider(name, min, max, val){
  if (sliders[name]) { sliders[name].min=min; sliders[name].max=max; updateSlider(name,val); return; }
  var row = document.createElement("div"); row.className = "row";
  row.innerHTML = '<label></label><input type="range"><output></output>';
  row.children[0].textContent = name + ":";
  var s = row.children[1]; s.min=min; s.max=max; s.value=val; s.id = "j_" + name;
  row.children[2].textContent = val;
  s.addEventListener("input", function(){ row.children[2].textContent = s.value; lastUserInput[name] = Date.now(); throttledMove(name, s.value); });
  sliders[name] = s; $("joints").appendChild(row); refresh();
}

function updateSlider(name, val){
  var s = sliders[name]; if (!s) return;
  if (Date.now() - (lastUserInput[name] || 0) < 400) return;   // don't fight the user's finger
  s.value = val; s.nextElementSibling.textContent = val;
}

function throttledMove(name, val){
  var now = Date.now(), wait = 40 - (now - (lastSend[name] || 0));
  clearTimeout(sendTimers[name]);
  if (wait <= 0){ lastSend[name] = now; send("Move," + name + "," + val); }
  else sendTimers[name] = setTimeout(function(){ lastSend[name] = Date.now(); send("Move," + name + "," + sliders[name].value); }, wait);
}

function refresh(){
  var r=$("Record"), pl=$("Play"), h=$("Home");
  r.textContent = "Record: " + (recording ? "ON" : "OFF"); r.classList.toggle("on", recording);
  pl.textContent = "Play: " + (playing ? "ON" : "OFF");     pl.classList.toggle("on", playing);
  r.disabled  = !authed || playing;
  pl.disabled = !authed || recording;
  h.disabled  = !authed || playing || recording;
  for (var n in sliders) sliders[n].disabled = !authed || playing;
}

$("Record").addEventListener("click", function(){ send("Record," + (recording ? 0 : 1)); });
$("Play").addEventListener("click",   function(){ send("Play,"   + (playing   ? 0 : 1)); });
$("Home").addEventListener("click",   function(){ send("Home,1"); });
$("pinBtn").addEventListener("click", function(){ var p=$("pin").value; storePin(p); send("Auth," + p); });
$("pin").addEventListener("keydown", function(e){ if (e.key === "Enter") $("pinBtn").click(); });
refresh();
connect();
</script>
</body>
</html>
)HTMLHOMEPAGE";
#else
// Serial-only mode: no-op stand-ins so the rest of the code stays identical.
struct AsyncWebSocketClient { void close() {} };
struct NullWebSocket
{
  void   text(uint32_t, const String&) {}
  void   textAll(const String&) {}
  size_t count() const { return 0; }
  void   cleanupClients(uint16_t = 0) {}
  AsyncWebSocketClient* client(uint32_t) { return nullptr; }
};
NullWebSocket ws;
#endif

// =============================== helpers ===================================

int findJoint(const char* name)
{
  for (uint8_t i = 0; i < NUM_JOINTS; i++)
    if (strcasecmp(name, joints[i].name) == 0) return i;
  return -1;
}

bool parseLong(const char* s, long& out)
{
  if (s == nullptr || *s == '\0') return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

uint8_t clampAngle(uint8_t j, long angle)
{
  if (angle < joints[j].minAngle) return joints[j].minAngle;
  if (angle > joints[j].maxAngle) return joints[j].maxAngle;
  return (uint8_t)angle;
}

void reply(uint32_t clientId, const String& msg)
{
  if (clientId == 0) Serial.println(msg);
  else ws.text(clientId, msg);
}

void broadcast(const String& msg)
{
  ws.textAll(msg);
  Serial.println(msg);
}

ClientInfo* clientInfo(uint32_t id)
{
  for (auto& c : clients) if (c.id == id) return &c;
  return nullptr;
}

bool isAuthorized(uint32_t clientId)
{
  if (clientId == 0) return true;                     // Serial = physical access
  ClientInfo* c = clientInfo(clientId);
  return c != nullptr && c->authed;
}

const char* modeName(Mode m)
{
  switch (m)
  {
    case Mode::Idle:       return "Idle";
    case Mode::Recording:  return "Recording";
    case Mode::PlayHoming: return "PlayHoming";
    case Mode::PlayPause:  return "PlayPause";
    case Mode::PlaySteps:  return "PlaySteps";
  }
  return "?";
}

bool isPlaying() { return mode == Mode::PlayHoming || mode == Mode::PlayPause || mode == Mode::PlaySteps; }

void broadcastState()
{
  broadcast(String("State,Record,") + (mode == Mode::Recording ? "ON" : "OFF"));
  broadcast(String("State,Play,")   + (isPlaying() ? "ON" : "OFF"));
  broadcast(String("Steps,") + recordedSteps.size());
}

void sendFullState(uint32_t clientId)
{
  for (uint8_t i = 0; i < NUM_JOINTS; i++)
  {
    reply(clientId, String("Joint,") + joints[i].name + "," + joints[i].minAngle + "," +
                    joints[i].maxAngle + "," + (int)lroundf(joints[i].current));
  }
  reply(clientId, String("State,Record,") + (mode == Mode::Recording ? "ON" : "OFF"));
  reply(clientId, String("State,Play,")   + (isPlaying() ? "ON" : "OFF"));
  reply(clientId, String("Steps,") + recordedSteps.size());
}

// ============================== persistence ================================

void saveRecording()
{
  prefs.begin("robotarm", false);
  prefs.putBytes("start", recordedStartPose, sizeof(recordedStartPose));
  prefs.putUInt("count", recordedSteps.size());
  if (!recordedSteps.empty())
    prefs.putBytes("steps", recordedSteps.data(), recordedSteps.size() * sizeof(RecordedStep));
  else
    prefs.remove("steps");
  prefs.end();
}

void loadRecording()
{
  prefs.begin("robotarm", true);
  size_t count = prefs.getUInt("count", 0);
  if (count > 0 && count <= MAX_STEPS &&
      prefs.getBytes("start", recordedStartPose, sizeof(recordedStartPose)) == sizeof(recordedStartPose))
  {
    recordedSteps.resize(count);
    size_t got = prefs.getBytes("steps", recordedSteps.data(), count * sizeof(RecordedStep));
    bool valid = (got == count * sizeof(RecordedStep));
    for (size_t i = 0; valid && i < count; i++)
      valid = recordedSteps[i].joint < NUM_JOINTS;
    for (uint8_t j = 0; valid && j < NUM_JOINTS; j++)
      valid = recordedStartPose[j] >= joints[j].minAngle && recordedStartPose[j] <= joints[j].maxAngle;
    if (!valid) recordedSteps.clear();
    // re-clamp in case limits were tightened since recording
    for (auto& s : recordedSteps) s.angle = clampAngle(s.joint, s.angle);
  }
  prefs.end();
  Serial.printf("Loaded %u recorded steps from flash\n", (unsigned)recordedSteps.size());
}

// ============================ mode transitions =============================

void setTargetsFromCurrent()
{
  for (auto& j : joints) j.target = (uint8_t)lroundf(j.current);
}

void startRecording()
{
  recordedSteps.clear();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) recordedStartPose[i] = joints[i].target;
  lastRecordTimeMs = millis();
  mode = Mode::Recording;
}

void stopRecording()
{
  mode = Mode::Idle;
  saveRecording();
  broadcast(String("Info,Saved ") + recordedSteps.size() + " steps");
}

bool startPlayback()
{
  if (recordedSteps.empty()) return false;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) joints[i].target = recordedStartPose[i];
  currentSpeed = HOMING_SPEED_DEG_S;
  mode = Mode::PlayHoming;
  return true;
}

void stopPlayback()
{
  mode = Mode::Idle;
  currentSpeed = LIVE_SPEED_DEG_S;
  setTargetsFromCurrent();   // freeze where we are, instantly
}

void moveJoint(uint8_t j, long angle)
{
  uint8_t a = clampAngle(j, angle);
  if (mode == Mode::Recording && a != joints[j].target)
  {
    if (recordedSteps.size() >= MAX_STEPS)
    {
      stopRecording();
      broadcast("Info,Recording full - stopped");
      broadcastState();
    }
    else
    {
      uint32_t now = millis();
      uint32_t d = now - lastRecordTimeMs;
      recordedSteps.push_back({ j, a, (uint16_t)std::min<uint32_t>(d, MAX_STEP_DELAY_MS) });
      lastRecordTimeMs = now;
    }
  }
  joints[j].target = a;
}

// ============================ command handling =============================

void handleText(uint32_t clientId, char* text)
{
  // trim trailing whitespace / CR
  size_t n = strlen(text);
  while (n && isspace((unsigned char)text[n - 1])) text[--n] = '\0';
  if (n == 0) return;

  char* save = nullptr;
  char* cmd = strtok_r(text, ",", &save);
  char* a1  = strtok_r(nullptr, ",", &save);
  char* a2  = strtok_r(nullptr, ",", &save);
  if (cmd == nullptr) return;

  if (strcasecmp(cmd, "Auth") == 0)
  {
    ClientInfo* c = clientInfo(clientId);
    if (clientId == 0) { reply(0, "Auth,OK"); return; }
    if (c == nullptr) return;
    if (strcmp(a1 ? a1 : "", CONTROL_PIN) == 0)
    {
      c->authed = true; c->fails = 0;
      reply(clientId, "Auth,OK");
    }
    else
    {
      c->authed = false;
      reply(clientId, "Auth,FAIL");
      if (++c->fails >= MAX_AUTH_FAILS)
      {
        AsyncWebSocketClient* wc = ws.client(clientId);
        if (wc) wc->close();
      }
    }
    return;
  }

  if (strcasecmp(cmd, "Status") == 0)
  {
    sendFullState(clientId);
    reply(clientId, String("Info,Mode ") + modeName(mode));
    return;
  }

  if (!isAuthorized(clientId)) { reply(clientId, "Auth,REQUIRED"); return; }

  long v = 0;
  if (strcasecmp(cmd, "Move") == 0)
  {
    int j = a1 ? findJoint(a1) : -1;
    if (j < 0 || !parseLong(a2, v)) { reply(clientId, "Info,Bad Move command"); return; }
    if (isPlaying()) { reply(clientId, "Info,Stop playback first"); return; }
    moveJoint((uint8_t)j, v);
  }
  else if (strcasecmp(cmd, "Record") == 0)
  {
    if (!parseLong(a1, v)) { reply(clientId, "Info,Bad Record command"); return; }
    if (v && isPlaying())            { reply(clientId, "Info,Stop playback first"); }
    else if (v && mode == Mode::Idle) { startRecording(); broadcast("Info,Recording..."); }
    else if (!v && mode == Mode::Recording) stopRecording();
    broadcastState();
  }
  else if (strcasecmp(cmd, "Play") == 0)
  {
    if (!parseLong(a1, v)) { reply(clientId, "Info,Bad Play command"); return; }
    if (v && mode == Mode::Recording) { reply(clientId, "Info,Stop recording first"); }
    else if (v && mode == Mode::Idle)
    {
      if (startPlayback()) broadcast("Info,Playing (returning to start pose)");
      else reply(clientId, "Info,Nothing recorded");
    }
    else if (!v && isPlaying()) { stopPlayback(); broadcast("Info,Playback stopped"); }
    broadcastState();
  }
  else if (strcasecmp(cmd, "Home") == 0)
  {
    if (mode != Mode::Idle) { reply(clientId, "Info,Busy"); return; }
    for (auto& j : joints) j.target = j.homeAngle;
    reply(clientId, "Info,Homing");
  }
  else
  {
    reply(clientId, "Info,Unknown command");
  }
}

void processCommands()
{
  Command c;
  while (xQueueReceive(commandQueue, &c, 0) == pdTRUE)
  {
    switch (c.type)
    {
      case CmdType::Connect:
      {
        ClientInfo* slot = clientInfo(0);       // free slot has id 0
        if (slot == nullptr)
        {
          AsyncWebSocketClient* wc = ws.client(c.clientId);
          if (wc) wc->close();
          break;
        }
        *slot = { c.clientId, strlen(CONTROL_PIN) == 0, 0 };
        sendFullState(c.clientId);
        reply(c.clientId, slot->authed ? "Auth,OK" : "Auth,REQUIRED");
        break;
      }
      case CmdType::Disconnect:
      {
        ClientInfo* ci = clientInfo(c.clientId);
        if (ci) *ci = { 0, false, 0 };
        break;
      }
      case CmdType::Text:
        handleText(c.clientId, c.text);
        break;
    }
  }
}

#if ENABLE_WIFI
// ======================= async callbacks (other task!) ======================
// Only copy data into the queue here - never touch robot state.

void queueCommand(CmdType type, uint32_t clientId, const uint8_t* data = nullptr, size_t len = 0)
{
  Command c;
  c.type = type;
  c.clientId = clientId;
  len = std::min(len, sizeof(c.text) - 1);
  if (data) memcpy(c.text, data, len);
  c.text[len] = '\0';
  xQueueSend(commandQueue, &c, 0);   // drop if full rather than block the network task
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len)
{
  switch (type)
  {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      queueCommand(CmdType::Connect, client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      queueCommand(CmdType::Disconnect, client->id());
      break;
    case WS_EVT_DATA:
    {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT && len < 64)
        queueCommand(CmdType::Text, client->id(), data, len);
      break;
    }
    default:
      break;
  }
}

#endif // ENABLE_WIFI

void pollSerial()
{
  static char buf[64];
  static size_t n = 0;
  while (Serial.available())
  {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r')
    {
      if (n) { buf[n] = '\0'; handleText(0, buf); n = 0; }
    }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

// ============================ motion / playback ============================

bool allAtTarget()
{
  for (auto& j : joints) if (lroundf(j.current) != j.target) return false;
  return true;
}

void updateMotion()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  uint32_t dt = now - last;
  if (dt < MOTION_TICK_MS) return;
  last = now;

  float maxStep = currentSpeed * dt / 1000.0f;
  for (auto& j : joints)
  {
    float diff = j.target - j.current;
    if (fabsf(diff) <= maxStep) j.current = j.target;
    else j.current += (diff > 0 ? maxStep : -maxStep);

    int a = (int)lroundf(j.current);
    if (a != j.lastWritten) { j.servo.write(a); j.lastWritten = a; }
  }
}

void updatePlayback()
{
  uint32_t now = millis();
  switch (mode)
  {
    case Mode::PlayHoming:
      if (allAtTarget()) { mode = Mode::PlayPause; playTimerMs = now; }
      break;
    case Mode::PlayPause:
      if (now - playTimerMs >= PLAY_START_PAUSE_MS)
      {
        currentSpeed = LIVE_SPEED_DEG_S;
        mode = Mode::PlaySteps; playIndex = 0; playTimerMs = now;
      }
      break;
    case Mode::PlaySteps:
      while (playIndex < recordedSteps.size() && now - playTimerMs >= recordedSteps[playIndex].delayMs)
      {
        playTimerMs += recordedSteps[playIndex].delayMs;
        const RecordedStep& s = recordedSteps[playIndex++];
        joints[s.joint].target = s.angle;
      }
      if (playIndex >= recordedSteps.size() && allAtTarget())
        startPlayback();              // loop the motion until Play is switched off
      break;
    default:
      break;
  }
}

void broadcastPositions()
{
  static uint32_t last = 0;
  if (millis() - last < POS_BROADCAST_MS) return;
  last = millis();
  bool anyone = ws.count() > 0;
  for (auto& j : joints)
  {
    int a = (int)lroundf(j.current);
    if (a != j.lastSent)
    {
      j.lastSent = a;
      if (anyone) ws.textAll(String("Pos,") + j.name + "," + a);
    }
  }
}

// ================================ setup/loop ===============================

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\nRobot Arm starting");

  commandQueue = xQueueCreate(32, sizeof(Command));
  for (auto& c : clients) c = { 0, false, 0 };
  recordedSteps.reserve(MAX_STEPS);
  for (auto& j : joints) j.homeAngle = constrain(j.homeAngle, j.minAngle, j.maxAngle);
  for (uint8_t i = 0; i < NUM_JOINTS; i++) recordedStartPose[i] = joints[i].homeAngle;
  loadRecording();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (auto& j : joints)
  {
    j.current = j.target = j.homeAngle;
    j.lastWritten = j.lastSent = j.homeAngle;
    j.servo.setPeriodHertz(50);
    j.servo.attach(j.pin, 500, 2400);
    j.servo.write(j.homeAngle);
  }

#if ENABLE_WIFI
  WiFi.mode(WIFI_AP);
  String mac = WiFi.softAPmacAddress();          // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  String ssid = String(AP_SSID_PREFIX) + mac.substring(mac.length() - 4);
  if (!WiFi.softAP(ssid.c_str(), AP_PASSWORD))
    Serial.println("ERROR: softAP failed (password must be 8-63 chars)");
  Serial.printf("AP SSID: %s  IP: %s\n", ssid.c_str(), WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", htmlHomePage);
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "File Not Found");
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");
#else
  Serial.println("Wi-Fi disabled (ENABLE_WIFI 0): Serial-only mode");
#endif
  Serial.println("Ready. Serial commands: Status, Move,Base,90, Record,1, Play,1, Home,1");
}

void loop()
{
  ws.cleanupClients(MAX_WS_CLIENTS);
  pollSerial();
  processCommands();
  updatePlayback();
  updateMotion();
  broadcastPositions();
}
