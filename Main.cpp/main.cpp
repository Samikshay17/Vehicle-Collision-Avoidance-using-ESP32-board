#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ================= MOTOR =================
#define IN1 5
#define IN2 17
#define IN3 25
#define IN4 13

#define PWM_A 0
#define PWM_B 1
#define PWM_FREQ 1000
#define PWM_RES 8

// ================= ALERT =================
#define LED_PIN 2
#define BUZZER_PIN 16

// ================= ULTRASONIC =================
#define TRIG_F 32
#define ECHO_F 33

// ================= WIFI =================
const char* ssid     = "NEXDRIVE";
const char* password = "12345687";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= STATE =================
String currentDirection = "S";
String lastDirection    = "S";

int  currentSpeed   = 200;
bool movingForward  = false;
bool movingBackward = false;
bool alertActive    = false;

float rawDist      = 200;
float smoothDist   = 200;
float currentDistF = 200;

int stopCounter = 0;
const int stopThreshold = 2;

unsigned long motorStopTime  = 0;
const int     motorSettleMs  = 200;

float odometerCm = 0;
unsigned long lastOdomUpdate = 0;

unsigned long lastSensorRead = 0;
const int sensorInterval     = 60;

unsigned long lastTelemetry = 0;
const int telemetryInterval = 150;

// ================= MOTOR FUNCTIONS =================
void stopMotors() {
  movingForward  = false;
  movingBackward = false;
  motorStopTime  = millis();

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}

void moveForward(int spd) {
  if (currentDistF <= 30) return;
  currentSpeed   = spd;
  movingForward  = true;
  movingBackward = false;
  motorStopTime  = millis();

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  delayMicroseconds(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);

  ledcWrite(PWM_A, spd);
  ledcWrite(PWM_B, spd);
}

void moveBackward(int spd) {
  currentSpeed   = spd;
  movingForward  = false;
  movingBackward = true;
  motorStopTime  = millis();

  // 1. Kill PWM first
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  delayMicroseconds(500);

  // 2. Set direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);

  // 3. Apply PWM
  ledcWrite(PWM_A, spd);
  ledcWrite(PWM_B, spd);
}

void turnLeft(int spd) {
  currentSpeed   = spd;
  movingForward  = false;
  movingBackward = false;
  motorStopTime  = millis();

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  delayMicroseconds(500);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, spd);
}

void turnRight(int spd) {
  currentSpeed   = spd;
  movingForward  = false;
  movingBackward = false;
  motorStopTime  = millis();

  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  delayMicroseconds(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);

  ledcWrite(PWM_A, spd);
  ledcWrite(PWM_B, 0);
}

// ================= ULTRASONIC =================
float readDistance() {
  float r[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_F, LOW);
    delayMicroseconds(4);
    digitalWrite(TRIG_F, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_F, LOW);

    long dur = pulseIn(ECHO_F, HIGH, 20000);
    if (dur == 0) r[i] = 200;
    else {
      float d = dur * 0.0343 / 2.0;
      r[i] = (d < 2 || d > 200) ? 200 : d;
    }
    delayMicroseconds(300);
  }

  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (r[j] < r[i]) { float t = r[i]; r[i] = r[j]; r[j] = t; }

  return r[1];
}

// ================= ALERT =================
void activateAlert() {
  alertActive = true;
  digitalWrite(LED_PIN,    HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
}

void deactivateAlert() {
  alertActive = false;
  digitalWrite(LED_PIN,    LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

// ================= TELEMETRY =================
void pushTelemetry() {
  int   spdCms = (movingForward || movingBackward) ? (int)((currentSpeed / 255.0) * 40.0) : 0;
  float odomM  = odometerCm / 100.0;

  String json = "{";
  json += "\"distF\":"     + String(currentDistF, 1) + ",";
  json += "\"speed\":"     + String(currentSpeed) + ",";
  json += "\"spdCms\":"    + String(spdCms) + ",";
  json += "\"dir\":\""     + currentDirection + "\",";
  json += "\"lastDir\":\"" + lastDirection + "\",";
  json += "\"odom\":"      + String(odomM, 2) + ",";
  json += "\"fwd\":"       + String(movingForward ? "true" : "false");
  json += "}";

  ws.textAll(json);
}

// ================= WEBSOCKET =================
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.println("WS Client connected: " + String(client->id()));

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WS Client disconnected");
    stopMotors();
    deactivateAlert();

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

      String msg = "";
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      Serial.println("RAW CMD: " + msg);

      int    colonIdx = msg.indexOf(':');
      String cmd      = (colonIdx != -1) ? msg.substring(0, colonIdx) : msg;
      int    spd      = (colonIdx != -1) ? msg.substring(colonIdx + 1).toInt() : currentSpeed;
      spd = constrain(spd, 0, 255);

      Serial.println("CMD=" + cmd + " SPD=" + String(spd));

      if (cmd != currentDirection) lastDirection = currentDirection;
      currentDirection = cmd;

      if      (cmd == "F") { Serial.println("FORWARD");  moveForward(spd);  }
      else if (cmd == "B") { Serial.println("BACKWARD"); moveBackward(spd); }
      else if (cmd == "L") { Serial.println("LEFT");     turnLeft(spd);     }
      else if (cmd == "R") { Serial.println("RIGHT");    turnRight(spd);    }
      else if (cmd == "S") { Serial.println("STOP");     stopMotors();      }
      else                 { Serial.println("UNKNOWN: " + cmd);             }
    }
  }
}

// ================= WEBPAGE =================
String webpage() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>NEXDRIVE</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body {
  background:#050505; color:#f5f5f5;
  font-family:'Courier New',monospace;
  display:flex; justify-content:center;
  align-items:center; height:100vh; overflow:hidden;
  touch-action:none;
}
.panel {
  width:100%; max-width:430px;
  background:#0b0b0b;
  border:1px solid #220000; border-radius:14px;
  padding:14px; box-shadow:0 0 40px rgba(255,0,0,0.25);
  position:relative;
}
.header {
  display:flex; justify-content:space-between;
  align-items:center; margin-bottom:10px;
}
.logo {
  font-size:18px; color:#ff2a2a;
  letter-spacing:3px; font-weight:bold;
  text-shadow:0 0 10px #ff0000;
}
.status { font-size:11px; color:#777; letter-spacing:2px; }
.status.on { color:#00ff88; }
.telemetry {
  display:grid; grid-template-columns:repeat(4,1fr);
  gap:6px; margin-bottom:10px;
}
.box {
  background:#111; border:1px solid #220000;
  border-radius:8px; padding:6px; text-align:center;
}
.val {
  font-size:18px; color:#ff2a2a;
  font-weight:bold; text-shadow:0 0 6px #ff0000;
}
.val.warn   { color:#ffaa00; text-shadow:0 0 6px #ffaa00; }
.val.danger { color:#ff2a2a; animation:blink 0.5s infinite; }
.label { font-size:10px; color:#888; letter-spacing:1px; }
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.2} }
@keyframes spin  { from{transform:rotate(0deg);} to{transform:rotate(360deg);} }
.radar {
  width:180px; height:180px; border-radius:50%;
  border:1px solid rgba(255,0,0,0.4);
  margin:8px auto; position:relative; overflow:hidden;
}
.sweep {
  position:absolute; width:100%; height:100%;
  background:conic-gradient(rgba(255,0,0,0.5),transparent 70%);
  animation:spin 3s linear infinite;
}
.dot {
  width:12px; height:12px; background:#ff0000;
  border-radius:50%; box-shadow:0 0 10px #ff0000;
  position:absolute; transition:top 0.2s, left 0.2s;
  display:none;
}
.car {
  width:60px; height:36px; background:#111;
  border:1px solid #ff2a2a; border-radius:6px;
  position:absolute; top:50%; left:50%;
  transform:translate(-50%,-50%);
  box-shadow:0 0 12px rgba(255,0,0,0.35);
}
.speedRow {
  display:flex; align-items:center;
  gap:10px; margin-bottom:10px;
}
.speedLabel { font-size:11px; color:#777; }
.speedValue { font-size:13px; color:#ff2a2a; min-width:36px; text-align:right; }
input[type=range] { flex:1; accent-color:red; height:4px; cursor:pointer; }
.controls {
  display:grid;
  grid-template-columns:repeat(3, 1fr);
  grid-template-rows:repeat(3, 70px);
  gap:6px;
}
.btn {
  background:#111; border:1px solid #330000;
  border-radius:10px; font-size:24px;
  color:#ff2a2a; cursor:pointer; transition:.15s;
  display:flex; align-items:center; justify-content:center;
  user-select:none; -webkit-user-select:none;
  touch-action:none;
}
.btn:active, .btn.pressed {
  background:#440000;
  box-shadow:0 0 10px rgba(255,0,0,0.6);
  transform:scale(0.94);
}
.btn-stop { border-color:#440000; color:#ff4444; }
.alert {
  position:absolute; top:-35px; left:50%;
  transform:translateX(-50%);
  background:#330000; border:1px solid #ff0000;
  padding:6px 14px; border-radius:6px;
  font-size:12px; color:#ff4d4d;
  animation:blink 0.4s infinite;
  white-space:nowrap; display:none;
}
</style>
</head>
<body>
<div class="panel">
  <div class="alert" id="alert">⚠ OBSTACLE DETECTED</div>
  <div class="header">
    <div class="logo">NEXDRIVE</div>
    <div class="status" id="status">OFFLINE</div>
  </div>
  <div class="telemetry">
    <div class="box">
      <div class="val" id="dist">--</div>
      <div class="label">DIST cm</div>
    </div>
    <div class="box">
      <div class="val" id="spd">0</div>
      <div class="label">SPD %</div>
    </div>
    <div class="box">
      <div class="val" id="cms">0</div>
      <div class="label">CM/S</div>
    </div>
    <div class="box">
      <div class="val" id="odom">0.00</div>
      <div class="label">ODOM m</div>
    </div>
  </div>
  <div class="radar">
    <div class="sweep"></div>
    <div class="car"></div>
    <div class="dot" id="dot"></div>
  </div>
  <div class="speedRow">
    <div class="speedLabel">SPD</div>
    <input type="range" min="80" max="255" value="200" id="slider"
      oninput="onSpd(this.value)"
      onchange="onSpd(this.value)">
    <div class="speedValue" id="spdTxt">78%</div>
  </div>
  <div class="controls">
    <div></div>
    <button class="btn" id="btnF"
      onmousedown="press('F')" onmouseup="rel()"
      ontouchstart="ev(event);press('F')" ontouchend="ev(event);rel()">↑</button>
    <div></div>
    <button class="btn" id="btnL"
      onmousedown="press('L')" onmouseup="rel()"
      ontouchstart="ev(event);press('L')" ontouchend="ev(event);rel()">←</button>
    <button class="btn btn-stop" id="btnS"
      onmousedown="rel()"
      ontouchstart="ev(event);rel()">■</button>
    <button class="btn" id="btnR"
      onmousedown="press('R')" onmouseup="rel()"
      ontouchstart="ev(event);press('R')" ontouchend="ev(event);rel()">→</button>
    <div></div>
    <button class="btn" id="btnB"
      onmousedown="press('B')" onmouseup="rel()"
      ontouchstart="ev(event);press('B')" ontouchend="ev(event);rel()">↓</button>
    <div></div>
  </div>
</div>
<script>
  var ws;
  var speed      = 200;
  var currentCmd = null;
  var currentBtn = null;

  function ev(e) { e.preventDefault(); }

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + '/ws');
    ws.onopen = function() {
      var s = document.getElementById('status');
      s.textContent = 'ONLINE';
      s.className   = 'status on';
    };
    ws.onclose = function() {
      var s = document.getElementById('status');
      s.textContent = 'OFFLINE';
      s.className   = 'status';
      setTimeout(connect, 1500);
    };
    ws.onerror = function() { ws.close(); };
    ws.onmessage = function(e) {
      try {
        var d = JSON.parse(e.data);
        var dist   = Math.round(d.distF);
        var distEl = document.getElementById('dist');
        distEl.textContent = dist;
        distEl.className   = 'val' + (dist < 30 ? ' danger' : dist < 80 ? ' warn' : '');
        document.getElementById('spd').textContent  = Math.round(d.speed / 255 * 100);
        document.getElementById('cms').textContent  = d.spdCms || 0;
        document.getElementById('odom').textContent = parseFloat(d.odom).toFixed(2);
        updateRadar(d.distF);
      } catch(err) {}
    };
  }

  connect();

  function send(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN)
      ws.send(cmd + ':' + speed);
  }

  function press(cmd) {
    if (currentCmd === cmd) return;
    currentCmd = cmd;
    if (currentBtn) currentBtn.classList.remove('pressed');
    currentBtn = document.getElementById('btn' + cmd);
    if (currentBtn) currentBtn.classList.add('pressed');
    send(cmd);
  }

  function rel() {
    send('S');
    currentCmd = null;
    if (currentBtn) currentBtn.classList.remove('pressed');
    currentBtn = null;
  }

  function onSpd(v) {
    speed = parseInt(v);
    document.getElementById('spdTxt').textContent = Math.round(speed / 255 * 100) + '%';
    document.getElementById('spd').textContent    = Math.round(speed / 255 * 100);
    if (currentCmd && currentCmd !== 'S') send(currentCmd);
  }

  function updateRadar(dist) {
    var dot     = document.getElementById('dot');
    var alertEl = document.getElementById('alert');
    if (dist > 150) {
      dot.style.display     = 'none';
      alertEl.style.display = 'none';
      return;
    }
    dot.style.display = 'block';
    var pct        = 1 - (dist / 150);
    dot.style.left = '84px';
    dot.style.top  = (90 - pct * 70) + 'px';
    alertEl.style.display = dist < 30 ? 'block' : 'none';
  }

  document.addEventListener('keydown', function(e) {
    if (e.repeat) return;
    if (e.key === 'ArrowUp')    press('F');
    if (e.key === 'ArrowDown')  press('B');
    if (e.key === 'ArrowLeft')  press('L');
    if (e.key === 'ArrowRight') press('R');
    if (e.key === ' ')          rel();
  });

  document.addEventListener('keyup', function(e) {
    if (e.key === 'ArrowUp'    ||
        e.key === 'ArrowDown'  ||
        e.key === 'ArrowLeft'  ||
        e.key === 'ArrowRight') rel();
  });
</script>
</body>
</html>
)rawliteral";
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);

  ledcAttachPin(IN2, PWM_A);
  ledcAttachPin(IN4, PWM_B);

  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  pinMode(LED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN,    LOW);
  digitalWrite(BUZZER_PIN, LOW);

  stopMotors();
  deactivateAlert();

  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(200);
  WiFi.mode(WIFI_AP);
  delay(100);

  WiFi.softAP(ssid, password);
  delay(200);

  Serial.println("IP: " + WiFi.softAPIP().toString());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", webpage());
  });

  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Server started");
}

// ================= LOOP =================
void loop() {

  // ===== SENSOR =====
  if (millis() - lastSensorRead >= sensorInterval) {
    bool settled = (millis() - motorStopTime) > motorSettleMs;
    if (settled) {
      rawDist = readDistance();
      if (rawDist > 100)
        smoothDist = (smoothDist * 0.3) + (rawDist * 0.7);
      else
        smoothDist = (smoothDist * 0.4) + (rawDist * 0.6);
      currentDistF = smoothDist;
    }
    lastSensorRead = millis();
  }

  // ===== SAFETY =====

if (currentDistF <= 30 && currentDistF > 2) {
  activateAlert();
  stopCounter++;
  if (stopCounter >= stopThreshold && movingForward) {
    stopMotors();
    currentDirection = "S";
    stopCounter = 0;
  }
} else if (currentDistF > 40) {
  stopCounter = 0;
  deactivateAlert();
}

  // ===== ODOMETER =====
  if (movingForward || movingBackward) {
    unsigned long now = millis();
    if (lastOdomUpdate > 0) {
      float dt       = (now - lastOdomUpdate) / 1000.0;
      float speedCms = (currentSpeed / 255.0) * 40.0;
      odometerCm    += speedCms * dt;
    }
    lastOdomUpdate = millis();
  } else {
    lastOdomUpdate = 0;
  }

  // ===== TELEMETRY =====
  if (millis() - lastTelemetry >= telemetryInterval) {
    pushTelemetry();
    lastTelemetry = millis();
  }

  ws.cleanupClients();
}