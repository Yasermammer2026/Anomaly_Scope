#include <SimpleFOC.h>
#include <WiFiS3.h>
#include <Wire.h>
#include <math.h>

// ================= WiFi =================
const char WIFI_SSID[] = "YOUR_WIFI_SSID";
const char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";

WiFiServer webServer(80);
bool wifiReady = false;

// ================= Motor data (DFRobot 2804, FIT1034) =================
const int   POLE_PAIRS       = 7;         // 14 poles = 7 pole pairs
const float SUPPLY_VOLTAGE   = 12.0f;
const float MAX_RPM          = 2600.0f;
const float PHASE_RESISTANCE = 2.3f;      // ohms
const float MOTOR_KV         = 220.0f;    // RPM per volt

// ================= Settings (change here if needed) =================
const float CURRENT_LIMIT  = 0.8f;    // amps (motor max is 1A)
const float ACCEL_RPM_S    = 500.0f;  // how fast speed changes (RPM per second)
const float KP             = 0.003f;  // speed controller: volts per RPM of error
const float KI             = 0.02f;   // speed controller: volts per RPM*second
const float VELOCITY_TF    = 0.02f;   // speed filter (seconds)
const float LEAD_FACTOR    = 1.0f;    // phase lead (best for top speed)
const float START_VOLTAGE  = 3.0f;    // voltage used to drag the motor at start

// ================= Internal constants =================
const float RPM_TO_RAD      = 2.0f * PI / 60.0f;
const float VOLTS_PER_RPM   = 1.0f / (MOTOR_KV * 1.732f);   // back-EMF: ~1V per 381 RPM
const float MAX_UQ          = SUPPLY_VOLTAGE / 1.732f;      // max voltage with Space Vector PWM
const float MAX_SHAFT_RAD_S = MAX_RPM * 1.5f * RPM_TO_RAD;  // fastest physically possible
const float INTEGRAL_MAX    = 3.0f;                         // max integral correction (volts)
const float MAX_LEAD_RAD    = 1.4f;                         // max phase lead (80 deg)
const float ALIGN_VOLTAGES[3] = { 3.0f, 4.0f, 4.0f };       // calibration voltage per attempt

// Start sequence: drag the motor open-loop, then switch to closed-loop
const float KICK_MAX_RPM        = 200.0f;
const float KICK_ACCEL          = 600.0f;
const unsigned long KICK_MIN_MS = 300;
const unsigned long KICK_MAX_MS = 2500;

// Encoder (AS5600 on I2C, Uno R4 WiFi: SDA = pin 18 / A4, SCL = pin 19 / A5)
const uint8_t  AS5600_ADDR = 0x36;
const uint32_t I2C_CLOCK   = 400000;
const uint8_t  I2C_SDA_PIN = 18;
const uint8_t  I2C_SCL_PIN = 19;

const unsigned long NETWORK_POLL_MS = 100;  // how often to check WiFi
const unsigned long MEASURE_MS      = 250;  // how often to update measured RPM

// ================= Encoder with glitch filter =================
// Ignores angle readings that jump further than the motor could physically move.
class FilteredAS5600 : public MagneticSensorI2C {
public:
  FilteredAS5600(MagneticSensorI2CConfig_s config) : MagneticSensorI2C(config) {}

  float getSensorAngle() override {
    float a = MagneticSensorI2C::getSensorAngle();
    unsigned long t = micros();

    if (hasPrev) {
      float d = a - prevAngle;
      while (d > PI)  d -= 2.0f * PI;
      while (d < -PI) d += 2.0f * PI;

      float maxMove = MAX_SHAFT_RAD_S * (t - prevMicros) * 1e-6f + 0.05f;
      if (!isfinite(a) || (fabs(d) > maxMove && rejects < 3)) {
        rejects++;
        return prevAngle;   // keep last good angle
      }
    }

    rejects = 0;
    prevAngle = a;
    prevMicros = t;
    hasPrev = true;
    return a;
  }

private:
  bool hasPrev = false;
  float prevAngle = 0.0f;
  unsigned long prevMicros = 0;
  int rejects = 0;
};

// ================= Objects =================
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 6);
FilteredAS5600 sensor = FilteredAS5600(AS5600_I2C);

// ================= State =================
bool hardwareReady = false;   // driver + encoder found
bool motorReady = false;      // calibration OK
bool running = false;
bool kicking = false;         // true while dragging the motor at start
bool voltageLimited = false;  // true at top speed (voltage maxed out)

float targetRPM = 0.0f;       // speed you asked for
float rampRPM = 0.0f;         // smoothly ramped setpoint
float kickRPM = 0.0f;         // drag speed during start
float filteredRPM = 0.0f;     // filtered real speed (used by controller)
float measuredRPM = 0.0f;     // averaged real speed (for display)
float integral = 0.0f;
float appliedVoltage = 0.0f;
float estimatedCurrent = 0.0f;
float baseZeroAngle = 0.0f;   // electrical zero from calibration
float leadRad = 0.0f;
float loopPeriod = 0.001f;    // filtered loop time (seconds)

double revCount = 0.0;        // revolutions since START
unsigned long runStartMs = 0;
unsigned long kickStartMs = 0;
unsigned long stallRecoveries = 0;
int stallTicks = 0;

float velSum = 0.0f;
unsigned long velSamples = 0;
unsigned long lastMeasureMs = 0;
unsigned long lastControlMicros = 0;
unsigned long lastNetworkPoll = 0;

char commandBuffer[40];
uint8_t commandLength = 0;

// Collects replies as text (to send to the web page in one piece)
class StringOut : public Print {
public:
  String text;
  size_t write(uint8_t c) override {
    text += (char)c;
    return 1;
  }
};

// ================= Helpers =================

const char *stateText() {
  if (!motorReady)    return "NOT CALIBRATED";
  if (!running)       return "STOPPED";
  if (kicking)        return "STARTING";
  if (voltageLimited) return "TOP SPEED";
  return "RUNNING";
}

String runTimeText() {
  unsigned long s = running ? (millis() - runStartMs) / 1000 : 0;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", s / 60, s % 60);
  return String(buf);
}

// ================= Motor control =================

void useClosedLoop() {
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = MAX_UQ;
}

// Start: drag the motor with a rotating field (open-loop)
void startKick() {
  kicking = true;
  kickRPM = 0;
  kickStartMs = millis();
  integral = 0;
  rampRPM = 0;
  voltageLimited = false;
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = START_VOLTAGE;
  lastControlMicros = micros();
}

// Drag until the encoder confirms it spins, then switch to closed-loop
void updateKick(float dt) {
  float kickTarget = min(KICK_MAX_RPM, targetRPM);

  kickRPM = min(kickRPM + KICK_ACCEL * dt, kickTarget);
  motor.move(kickRPM * RPM_TO_RAD);
  appliedVoltage = START_VOLTAGE;
  estimatedCurrent = 0;

  unsigned long elapsed = millis() - kickStartMs;

  if (elapsed > KICK_MIN_MS && filteredRPM > kickTarget * 0.6f) {
    kicking = false;
    useClosedLoop();
    rampRPM = min(filteredRPM, targetRPM);
    integral = 0;
    return;
  }

  if (elapsed > KICK_MAX_MS) {   // didn't catch: pause and retry
    motor.disable();
    delay(300);
    motor.enable();
    startKick();
  }
}

void stopMotor() {
  running = false;
  kicking = false;
  targetRPM = 0;
  rampRPM = 0;
  integral = 0;
  appliedVoltage = 0;
  estimatedCurrent = 0;
  voltageLimited = false;
  stallTicks = 0;
  leadRad = 0;
  motor.zero_electric_angle = baseZeroAngle;
  useClosedLoop();
  motor.move(0);
  motor.disable();
}

void recoverFromStall() {
  stallRecoveries++;
  Serial.println("WARNING: motor stalled - restarting");
  motor.disable();
  delay(300);
  filteredRPM = 0;
  motor.enable();
  startKick();
}

bool calibrateMotor() {
  useClosedLoop();
  for (int attempt = 0; attempt < 3; attempt++) {
    motor.voltage_sensor_align = ALIGN_VOLTAGES[attempt];
    motor.sensor_direction = Direction::UNKNOWN;
    motor.zero_electric_angle = NOT_SET;

    Serial.print("Calibrating, attempt ");
    Serial.print(attempt + 1);
    Serial.println("... motor will move a little");

    motor.enable();
    if (motor.initFOC()) {
      baseZeroAngle = motor.zero_electric_angle;
      motor.disable();
      lastControlMicros = micros();
      Serial.println("Calibration OK");
      return true;
    }
    motor.disable();
    delay(1000);
  }

  Serial.println("ERROR: Calibration failed. Check 12V supply and motor wires, then send CALIBRATE.");
  return false;
}

// Pushes slightly ahead of the rotor to make up for the encoder delay
void updatePhaseLead() {
  leadRad = 0;
  if (running && !kicking) {
    leadRad = filteredRPM * RPM_TO_RAD * POLE_PAIRS * loopPeriod * LEAD_FACTOR;
    leadRad = constrain(leadRad, -MAX_LEAD_RAD, MAX_LEAD_RAD);
  }
  motor.zero_electric_angle = baseZeroAngle - leadRad;
}

// Reads the encoder speed, ignores impossible values, filters it
void updateSpeed(float dt) {
  float direction = (float)((int)motor.sensor_direction);
  if (direction == 0) direction = 1;

  float v = sensor.getVelocity();
  if (!isfinite(v) || fabs(v) > MAX_SHAFT_RAD_S) return;

  float rpm = v * direction / RPM_TO_RAD;
  filteredRPM += (dt / (VELOCITY_TF + dt)) * (rpm - filteredRPM);

  velSum += rpm;
  velSamples++;
}

// Speed controller: base voltage for the speed + correction, limited by current
void speedControl(float dt) {
  // Ramp toward the target (hold if the motor can't go faster)
  float step = ACCEL_RPM_S * dt;
  if (rampRPM < targetRPM) {
    if (!(voltageLimited && rampRPM > filteredRPM + 100.0f)) {
      rampRPM = min(rampRPM + step, targetRPM);
    }
  } else if (rampRPM > targetRPM) {
    rampRPM = max(rampRPM - step, targetRPM);
  }

  float error = rampRPM - filteredRPM;

  integral = constrain(integral + KI * error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
  float uq = rampRPM * VOLTS_PER_RPM + KP * error + integral;

  // Current limit and driver voltage limit (with anti-windup)
  float backEMF = filteredRPM * VOLTS_PER_RPM;
  float high = min(backEMF + CURRENT_LIMIT * PHASE_RESISTANCE, MAX_UQ);
  float low  = max(backEMF - CURRENT_LIMIT * PHASE_RESISTANCE, -MAX_UQ);

  if (uq > high) {
    uq = high;
    if (error > 0) integral -= KI * error * dt;
  } else if (uq < low) {
    uq = low;
    if (error < 0) integral -= KI * error * dt;
  }

  voltageLimited = (uq >= MAX_UQ - 0.05f);
  appliedVoltage = uq;
  estimatedCurrent = (uq - backEMF) / PHASE_RESISTANCE;

  motor.move(uq);
}

// Every 0.25 s: update measured RPM, revolutions, and check for a stall
void updateMeasurement() {
  unsigned long now = millis();
  if (now - lastMeasureMs < MEASURE_MS) return;

  float dt = (now - lastMeasureMs) / 1000.0f;
  lastMeasureMs = now;

  if (velSamples > 0) measuredRPM = velSum / velSamples;
  velSum = 0;
  velSamples = 0;

  if (running) revCount += fabs(measuredRPM) / 60.0 * dt;

  if (running && !kicking && rampRPM > 300.0f && fabs(measuredRPM) < rampRPM * 0.2f) {
    stallTicks++;
  } else {
    stallTicks = 0;
  }

  if (stallTicks >= 2) {
    stallTicks = 0;
    recoverFromStall();
  }
}

// ================= Commands =================

void printStatus(Print &out) {
  out.print("State: ");         out.println(stateText());
  out.print("Target RPM: ");    out.println(targetRPM, 0);
  out.print("Measured RPM: ");  out.println(measuredRPM, 0);
  out.print("Voltage: ");       out.print(appliedVoltage, 2);
  out.print(" V (max ");        out.print(MAX_UQ, 2); out.println(" V)");
  out.print("Est. current: ");  out.print(estimatedCurrent, 2); out.println(" A");
  out.print("Phase lead: ");    out.print(leadRad * 57.3f, 1); out.println(" deg");
  out.print("Run time: ");      out.print(runTimeText());
  out.print("   Revolutions: "); out.println((unsigned long)revCount);
  out.print("Stall recoveries: "); out.println(stallRecoveries);
}

void handleCommand(String cmd, Print &out) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "STOP") {
    stopMotor();
    out.println("STOPPED");
    return;
  }

  if (cmd == "STATUS") {
    printStatus(out);
    return;
  }

  if (cmd == "HELP") {
    out.println("SPEED x   - set speed (0-2600)");
    out.println("START     - start motor");
    out.println("STOP      - stop motor");
    out.println("STATUS    - show status");
    out.println("CALIBRATE - redo encoder calibration");
    return;
  }

  if (cmd == "CALIBRATE") {
    if (!hardwareReady) {
      out.println("ERROR: Driver or encoder not found - check wiring and reset");
      return;
    }
    if (running) {
      out.println("ERROR: STOP the motor first");
      return;
    }
    motorReady = calibrateMotor();
    out.println(motorReady ? "Calibration OK" : "Calibration FAILED - check 12V and motor wires");
    return;
  }

  if (!motorReady) {
    out.println("ERROR: Motor not calibrated - press CALIBRATE");
    return;
  }

  if (cmd.startsWith("SPEED ")) {
    float rpm = cmd.substring(6).toFloat();
    if (!isfinite(rpm) || rpm < 0 || rpm > MAX_RPM) {
      out.println("ERROR: RPM must be 0 - 2600");
      return;
    }
    targetRPM = rpm;
    if (rpm == 0) stopMotor();
    out.print("Target RPM: ");
    out.println(targetRPM, 0);
    if (!running && rpm > 0) out.println("Now press START");
    return;
  }

  if (cmd == "START") {
    if (targetRPM <= 0) {
      out.println("ERROR: Set SPEED first");
      return;
    }
    if (!running) {
      stallTicks = 0;
      filteredRPM = 0;
      revCount = 0;
      runStartMs = millis();
      motor.enable();
      running = true;
      startKick();
    }
    out.println("MOTOR STARTED");
    return;
  }

  out.println("ERROR: Unknown command (type HELP)");
}

// ================= WiFi & web page =================

void connectWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("ERROR: WiFi module not found");
    return;
  }

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  int attempts = 0;
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED && attempts < 5) {
    delay(3000);
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR: WiFi connection failed (USB still works)");
    return;
  }

  unsigned long start = millis();
  while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && millis() - start < 10000) {
    delay(200);
  }

  webServer.begin();
  wifiReady = true;

  Serial.print("WiFi connected. Open: http://");
  Serial.println(WiFi.localIP());
}

// Small data packet for the live display (only requested when Live is ON)
String buildData() {
  String d = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
             "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
  d += "{\"state\":\"" + String(stateText()) + "\"";
  d += ",\"target\":"   + String(targetRPM, 0);
  d += ",\"measured\":" + String(measuredRPM, 0);
  d += ",\"voltage\":"  + String(appliedVoltage, 2);
  d += ",\"current\":"  + String(estimatedCurrent, 2);
  d += ",\"time\":\""   + runTimeText() + "\"";
  d += ",\"revs\":"     + String((unsigned long)revCount) + "}";
  return d;
}

String buildPage(const String &message) {
  String p;
  p.reserve(5000);

  p += "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
  p += "<!DOCTYPE html><html><head>"
       "<meta name='viewport' content='width=device-width,initial-scale=1'>"
       "<title>Nerva Motor</title><style>"
       "body{font-family:sans-serif;max-width:420px;margin:20px auto;padding:0 10px}"
       "button,input{font-size:20px;padding:10px;margin:5px 0;width:100%;box-sizing:border-box}"
       ".stop{background:#c00;color:#fff}.start{background:#1a7f37;color:#fff}"
       ".warn{color:#c00}pre{background:#f2f2f2;padding:8px}"
       ".live{background:#f7f7f7;border:1px solid #ddd;border-radius:6px;padding:10px;line-height:1.6}"
       ".dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#bbb;margin-right:6px}"
       ".big{font-size:28px;font-weight:bold}#lv{font-size:16px;padding:6px;margin:0 0 6px 0}"
       "</style></head><body><h2>Nerva Motor</h2>";

  if (!motorReady) {
    p += "<p class='warn'><b>Not calibrated.</b> Check 12V is on, then press CALIBRATE.</p>";
  }

  // Values box
  p += "<div class='live'>"
       "<button id='lv' type='button' onclick='tg()'>Live: OFF</button>"
       "<span class='dot' id='dot'></span>";
  p += "State: <b id='st'>" + String(stateText()) + "</b><br>";
  p += "Measured: <span class='big' id='ms'>" + String(measuredRPM, 0) + "</span> RPM<br>";
  p += "Target: <b id='tgt'>" + String(targetRPM, 0) + "</b> RPM<br>";
  p += "Voltage: <b id='vo'>" + String(appliedVoltage, 2) + "</b> V &nbsp; ";
  p += "Current: <b id='cu'>" + String(estimatedCurrent, 2) + "</b> A<br>";
  p += "Run time: <b id='tm'>" + runTimeText() + "</b> &nbsp; ";
  p += "Revolutions: <b id='rv'>" + String((unsigned long)revCount) + "</b></div>";

  // Controls
  p += "<form action='/speed'><input type='number' name='rpm' min='0' max='2600' step='50' value='";
  p += String((int)targetRPM);
  p += "'><button>Set speed</button></form>"
       "<form action='/start'><button class='start'>START</button></form>"
       "<form action='/stop'><button class='stop'>STOP</button></form>"
       "<form action='/status'><button>STATUS</button></form>"
       "<form action='/calibrate'><button>CALIBRATE</button></form>";

  if (message.length() > 0) {
    p += "<pre>" + message + "</pre>";
  }

  // Live ON/OFF (remembered in the browser); when ON, fetch values every second
  p += "<script>"
       "var timer=null;"
       "function s(i,v){document.getElementById(i).textContent=v;}"
       "async function u(){var dot=document.getElementById('dot');"
       "try{var r=await fetch('/data',{cache:'no-store'});var d=await r.json();"
       "s('st',d.state);s('ms',d.measured);s('tgt',d.target);"
       "s('vo',d.voltage.toFixed(2));s('cu',d.current.toFixed(2));"
       "s('tm',d.time);s('rv',d.revs);dot.style.background='#1a7f37';"
       "}catch(e){dot.style.background='#c00';}}"
       "function setLive(on){var b=document.getElementById('lv');"
       "if(timer){clearInterval(timer);timer=null;}"
       "if(on){timer=setInterval(u,1000);u();b.textContent='Live: ON (every second)';"
       "b.style.background='#1a7f37';b.style.color='#fff';}"
       "else{b.textContent='Live: OFF';b.style.background='';b.style.color='';"
       "document.getElementById('dot').style.background='#bbb';}"
       "try{localStorage.setItem('live',on?'1':'0');}catch(e){}}"
       "function tg(){setLive(timer===null);}"
       "var saved='0';try{saved=localStorage.getItem('live')||'0';}catch(e){}"
       "setLive(saved==='1');"
       "</script></body></html>";
  return p;
}

void handleWeb() {
  WiFiClient client = webServer.available();
  if (!client) return;

  client.setTimeout(50);
  String requestLine = client.readStringUntil('\n');
  while (client.available()) client.read();

  int first = requestLine.indexOf(' ');
  int second = requestLine.indexOf(' ', first + 1);
  String path = (first >= 0 && second > first) ? requestLine.substring(first + 1, second) : "/";

  int q = path.indexOf('?');
  String route = (q >= 0) ? path.substring(0, q) : path;
  String query = (q >= 0) ? path.substring(q + 1) : "";

  if (route == "/favicon.ico") {
    client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
    client.stop();
    return;
  }

  if (route == "/data") {
    client.print(buildData());
    client.stop();
    return;
  }

  String cmd = "";
  if (route == "/start")          cmd = "START";
  else if (route == "/stop")      cmd = "STOP";
  else if (route == "/status")    cmd = "STATUS";
  else if (route == "/calibrate") cmd = "CALIBRATE";
  else if (route == "/speed" && query.startsWith("rpm=")) cmd = "SPEED " + query.substring(4);

  StringOut result;
  if (cmd.length() > 0) handleCommand(cmd, result);

  client.print(buildPage(result.text));
  client.stop();
}

// ================= Encoder (I2C) setup =================

// Frees the I2C bus if the encoder got stuck after a reset
void recoverI2C() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
  }
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(10);
}

bool encoderFound() {
  for (int attempt = 0; attempt < 5; attempt++) {
    recoverI2C();
    Wire.begin();
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() == 0) return true;
    Wire.end();
    delay(200);
  }
  return false;
}

// Sets the AS5600's internal filter to its fastest setting (less delay)
bool configureAS5600Fast() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x07);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();

  hi = (hi & ~0x1F) | 0x03;   // fastest filter
  lo = lo & ~0x0C;            // no hysteresis

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x07);
  Wire.write(hi);
  Wire.write(lo);
  return Wire.endTransmission() == 0;
}

// ================= Setup & loop =================

void setupMotor() {
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE;
  driver.pwm_frequency = 25000;

  if (!driver.init()) {
    Serial.println("ERROR: Driver initialization failed");
    return;
  }

  if (!encoderFound()) {
    Serial.println("ERROR: Encoder not found - check SDA/SCL/3.3V/GND wiring");
    driver.disable();
    return;
  }

  sensor.init(&Wire);
  Wire.setClock(I2C_CLOCK);
  if (!configureAS5600Fast()) {
    Serial.println("WARNING: could not set encoder filter");
  }

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = MAX_UQ;
  motor.velocity_limit = MAX_RPM * RPM_TO_RAD + 20.0f;

  if (!motor.init()) {
    Serial.println("ERROR: Motor initialization failed");
    driver.disable();
    return;
  }

  hardwareReady = true;
  delay(500);
  motorReady = calibrateMotor();

  lastMeasureMs = millis();
  lastControlMicros = micros();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotor();
  connectWiFi();

  Serial.println("NERVA READY - commands: SPEED x, START, STOP, STATUS, CALIBRATE");
}

void loop() {
  // USB commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleCommand(String(commandBuffer), Serial);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
    }
  }

  // WiFi (motor coasts freely during the short WiFi pause)
  if (wifiReady && millis() - lastNetworkPoll >= NETWORK_POLL_MS) {
    lastNetworkPoll = millis();
    bool wasRunning = running;
    if (wasRunning) driver.disable();
    handleWeb();
    if (wasRunning && running) driver.enable();
  }

  if (!motorReady) return;

  // Loop timing
  unsigned long now = micros();
  float dt = (now - lastControlMicros) * 1e-6f;
  lastControlMicros = now;
  if (dt <= 0 || dt > 0.01f) {
    dt = loopPeriod;
  } else {
    loopPeriod += 0.01f * (dt - loopPeriod);
  }

  updatePhaseLead();
  motor.loopFOC();
  updateSpeed(dt);

  if (running) {
    if (kicking) updateKick(dt);
    else         speedControl(dt);
  } else {
    motor.move(0);
  }

  updateMeasurement();
}