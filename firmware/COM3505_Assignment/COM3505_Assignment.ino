// COM3505_Assignment.ino
// COM3505 - IoT Assignment: ESP32 Sensor & LED Controller
//
// Reads two temperature sensors (TMP36 + NTC thermistor analog), drives
// three discrete LEDs through a non-blocking pattern engine, and exchanges
// JSON state with a Python Flask server over Wi-Fi (HTTP). A push button
// is debounced and triple-press detection cycles the active pattern.
//
// Circuit (Adafruit Feather ESP32-S3):
//   TMP36 pin 1 (Vs)   -> 3V
//   TMP36 pin 2 (Vout) -> A5 / GPIO 8  (ADC1 - works while Wi-Fi is up)
//   TMP36 pin 3 (GND)  -> GND
//   NTC thermistor div -> GPIO 18 (A0): 3V - 10k ref - junction - NTC - GND
//   Push button        -> GPIO 6 (other leg to GND, INPUT_PULLUP, active LOW)
//   Green  LED + 120R  -> GPIO 9
//   Yellow LED + 120R  -> GPIO 10
//   Red    LED + 120R  -> GPIO 11
//
// WiFi: fill in your own SSID, password, and the IP/port of your Flask server.

#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define FLASK_HOST    "192.168.1.100"
#define FLASK_PORT    8080

// ===== Debug macros (Ex11 house style) =====
#define dbg(b, s) if(b) Serial.print(s)
#define dln(b, s) if(b) Serial.println(s)
#define startupDBG true
#define loopDBG    true

// ===== Pin map + constants =====
#define TMP36_PIN    8     // A5 on Feather ESP32-S3 (ADC1)
#define NTC_PIN      18    // NTC thermistor voltage-divider junction (A0)
#define BTN_PIN      6     // Push button (INPUT_PULLUP, active LOW)
#define LED_GREEN    9
#define LED_YELLOW  10
#define LED_RED     11
#define NUM_LEDS     3     // G, Y, R - indexed 0,1,2 in led_buffer

// NTC thermistor constants (matches Ex12 — 10k NTC, 10k reference, beta=3950)
#define NTC_REF_OHMS    10000.0   // change to 4700.0 if using 4.7k from kit
#define NTC_NOMINAL_R   10000.0
#define NTC_NOMINAL_T   25.0
#define NTC_BETA        3950.0
#define NTC_VREF        3.3
#define NTC_ADC_MAX     4095.0

// Timing intervals (ms)
#define SEND_INTERVAL     2000   // POST JSON to Flask every 2 s
#define NTC_INTERVAL      2000   // NTC sampling period
#define CMD_POLL_INTERVAL 1500   // GET /api/command every 1.5 s
#define WIFI_RETRY_MS     5000   // Background reconnect attempt period
#define NTC_STALE_MS     10000   // Mark NTC readings stale if older than this
#define DEBOUNCE_MS         50
#define MULTI_PRESS_WINDOW 800
#define BTN_STUCK_MS      3000   // ignore button if held LOW longer than this
#define HEAT_SPIKE_DELTA   5.0   // degC above baseline for FIRE auto-switch
#define HEAT_SPIKE_NEEDED  3     // consecutive readings before triggering

// ===== Color + Pattern enums =====
// Per the spec: led_buffer stores the COLOR VALUE for each LED slot,
// not a brightness. Hardware translation happens in flushLEDs().
enum Color : uint8_t { OFF = 0, GREEN = 1, YELLOW = 2, RED = 3 };

enum PatternId : uint8_t {
  PAT_BLINK = 0, PAT_RAINBOW, PAT_CHASE, PAT_FIRE, PAT_TEMPERATURE,
  PAT_COLOR_RED, PAT_COLOR_GREEN, PAT_COLOR_YELLOW, PAT_ALL_OFF, PAT_COUNT
};

// ===== State globals (flat - no struct, by spec) =====
Color    led_buffer[NUM_LEDS]      = { OFF, OFF, OFF };  // color values
uint8_t  fire_brightness[NUM_LEDS] = { 0, 0, 0 };        // PWM duty for FIRE
PatternId currentPatternId = PAT_TEMPERATURE;
PatternId lastPatternId    = PAT_COUNT;   // sentinel - forces first-run reset
uint16_t  patternStep      = 0;
unsigned long patternLastTick = 0;

// Wi-Fi / Flask state
unsigned long lastSendTime    = 0;
unsigned long lastCmdPollTime = 0;
unsigned long lastWifiRetry   = 0;
unsigned long userOverrideUntil = 0;  // suppress auto-switch after user command
String pendingAck = "";   // pattern name to ACK back to Flask on next POST

// TMP36 state
float tmp36Temp        = 0.0;
float baselineTemp     = 0.0;
bool  baselineSet      = false;
int   tmp36WarmupCount = 0;       // discard first 5 samples
int   tmp36LockCount   = 0;       // consecutive samples within 1 degC
float tmp36LastReading = 0.0;
int   heatSpikeCount   = 0;
int   tmp36RelockCount = 0;       // consecutive samples >30degC off baseline -> re-lock
Preferences prefs;

// NTC thermistor state (JSON keys reuse "dht_*" for dashboard compatibility)
float ntcTemp = NAN;
unsigned long lastNTCTime     = 0;
unsigned long lastValidNTC_ms = 0;
bool  ntcEverValid = false;

// Button state
bool          lastBtnReading = HIGH;
bool          btnState       = HIGH;
unsigned long debounceT      = 0;
int           pressCount     = 0;
unsigned long firstPressT    = 0;
unsigned long totalPresses   = 0;
unsigned long btnLowSince    = 0;
bool          btnIgnored     = false;

// ===== Forward declarations =====
void readTMP36();
void readNTC();
void handleButton();
void patternTick(unsigned long now);
void tickBlink(unsigned long now);
void tickRainbow(unsigned long now);
void tickChase(unsigned long now);
void tickFire(unsigned long now);
void tickTemperature(unsigned long now);
void tickSolid(Color c);
void tickAllOff();
void flushLEDs();
void clearLedBuffer();
void setPatternById(PatternId p, const char* source);
PatternId patternFromString(const String& s);
String patternToString(PatternId p);
void sendToFlask();
void checkFlaskCommand();
void checkSerialCommand();
String buildJSON();
void loadBaseline();
void saveBaseline(float v);
void connectWiFi();
void maybeReconnectWiFi();

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  dln(startupDBG, "");
  dln(startupDBG, "=== COM3505 IoT Assignment ===");

  // ADC config (Ex11 house style)
  analogReadResolution(12);
  analogSetPinAttenuation(TMP36_PIN, ADC_11db);

  // LED pins
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  clearLedBuffer();
  flushLEDs();

  pinMode(BTN_PIN, INPUT_PULLUP);
  analogSetPinAttenuation(NTC_PIN, ADC_11db);

  // NVS - load persisted baseline if present
  prefs.begin("com3505", false);
  loadBaseline();

  // Wi-Fi - bounded 5 s connect; non-blocking reconnect runs in loop()
  connectWiFi();

  // Quick startup blink (only place we use delay() outside the WiFi block)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, HIGH);
    delay(120);
    digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_YELLOW, LOW);  digitalWrite(LED_RED, LOW);
    delay(120);
  }
  dln(startupDBG, "System ready. Default pattern: TEMPERATURE.");
}

// ===== LOOP =====
void loop() {
  unsigned long now = millis();

  readTMP36();
  if (now - lastNTCTime >= NTC_INTERVAL) { readNTC(); lastNTCTime = now; }

  handleButton();
  patternTick(now);
  flushLEDs();             // single place LEDs are touched
  checkSerialCommand();    // dev/USB fallback

  if (now - lastSendTime >= SEND_INTERVAL) {
    Serial.println("JSON:" + buildJSON());  // local mirror for USB clients
    if (WiFi.status() == WL_CONNECTED) sendToFlask();
    lastSendTime = now;
  }
  if (now - lastCmdPollTime >= CMD_POLL_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) checkFlaskCommand();
    lastCmdPollTime = now;
  }
  maybeReconnectWiFi();
}

// ===== Pattern engine - non-blocking dispatch =====
void patternTick(unsigned long now) {
  // On pattern change: reset step/timer and clear buffer so the new
  // pattern paints from a known state on its next computed tick.
  if (currentPatternId != lastPatternId) {
    // Leaving FIRE: force LEDC PWM duty to 0, then re-mode pins as plain
    // OUTPUT so digitalWrite() actually drives them. Arduino-ESP32 v3's
    // implicit detach on digitalWrite is unreliable; do it explicitly.
    if (lastPatternId == PAT_FIRE) {
      analogWrite(LED_GREEN,  0);
      analogWrite(LED_YELLOW, 0);
      analogWrite(LED_RED,    0);
      pinMode(LED_GREEN,  OUTPUT);
      pinMode(LED_YELLOW, OUTPUT);
      pinMode(LED_RED,    OUTPUT);
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED,    LOW);
    }
    patternStep = 0;
    patternLastTick = 0;
    clearLedBuffer();
    for (int i = 0; i < NUM_LEDS; i++) fire_brightness[i] = 0;
    lastPatternId = currentPatternId;
    dbg(loopDBG, "Pattern -> "); dln(loopDBG, patternToString(currentPatternId));
  }

  switch (currentPatternId) {
    case PAT_BLINK:        tickBlink(now);       break;
    case PAT_RAINBOW:      tickRainbow(now);     break;
    case PAT_CHASE:        tickChase(now);       break;
    case PAT_FIRE:         tickFire(now);        break;
    case PAT_TEMPERATURE:  tickTemperature(now); break;
    case PAT_COLOR_RED:    tickSolid(RED);       break;
    case PAT_COLOR_GREEN:  tickSolid(GREEN);     break;
    case PAT_COLOR_YELLOW: tickSolid(YELLOW);    break;
    case PAT_ALL_OFF:      tickAllOff();         break;
    default:               tickAllOff();         break;
  }
}

// BLINK: 500 ms half-period (1 Hz visible). All 3 LEDs toggle together —
// each slot holds its own physical color so flushLEDs lights every pin.
void tickBlink(unsigned long now) {
  if (now - patternLastTick < 500 && patternLastTick != 0) return;
  patternLastTick = now;
  bool on = !(patternStep & 0x1);
  led_buffer[0] = on ? GREEN  : OFF;
  led_buffer[1] = on ? YELLOW : OFF;
  led_buffer[2] = on ? RED    : OFF;
  patternStep++;
}

// RAINBOW: 250 ms/step, GREEN -> YELLOW -> RED -> YELLOW
void tickRainbow(unsigned long now) {
  if (now - patternLastTick < 250 && patternLastTick != 0) return;
  patternLastTick = now;
  static const Color seq[4] = { GREEN, YELLOW, RED, YELLOW };
  Color c = seq[patternStep & 0x3];
  for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = c;
  patternStep++;
}

// CHASE: 200 ms/step, single LED walking G -> Y -> R -> G
void tickChase(unsigned long now) {
  if (now - patternLastTick < 200 && patternLastTick != 0) return;
  patternLastTick = now;
  static const Color seq[NUM_LEDS] = { GREEN, YELLOW, RED };
  for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = OFF;
  uint8_t pos = patternStep % NUM_LEDS;
  led_buffer[pos] = seq[pos];
  patternStep++;
}

// FIRE: 60 ms/frame randomized warm flicker via PWM.
// Bias R~200, Y~120, G~60 - tuned by eye for warm flicker effect.
void tickFire(unsigned long now) {
  if (now - patternLastTick < 60 && patternLastTick != 0) return;
  patternLastTick = now;
  led_buffer[0] = GREEN;  led_buffer[1] = YELLOW;  led_buffer[2] = RED;
  fire_brightness[0] = constrain(60  + (int)random(-30, 31), 0, 255);
  fire_brightness[1] = constrain(120 + (int)random(-50, 51), 0, 255);
  fire_brightness[2] = constrain(200 + (int)random(-40, 41), 0, 255);
  patternStep++;
}

// TEMPERATURE: solid LEDs based on delta vs baseline.
void tickTemperature(unsigned long now) {
  if (now - patternLastTick < 200 && patternLastTick != 0) return;
  patternLastTick = now;
  if (!baselineSet) {
    led_buffer[0] = GREEN; led_buffer[1] = OFF; led_buffer[2] = OFF;
    return;
  }
  float delta = fabs(tmp36Temp - baselineTemp);
  if (delta < 2.0) {
    led_buffer[0] = GREEN; led_buffer[1] = OFF;    led_buffer[2] = OFF;
  } else if (delta < 5.0) {
    led_buffer[0] = GREEN; led_buffer[1] = YELLOW; led_buffer[2] = OFF;
  } else {
    led_buffer[0] = GREEN; led_buffer[1] = YELLOW; led_buffer[2] = RED;
  }
}

// Solid single-color: requested color lights its physical LED only.
void tickSolid(Color c) {
  led_buffer[0] = (c == GREEN)  ? GREEN  : OFF;
  led_buffer[1] = (c == YELLOW) ? YELLOW : OFF;
  led_buffer[2] = (c == RED)    ? RED    : OFF;
}

void tickAllOff() { for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = OFF; }

// ===== Hardware flush - the only place LEDs are written =====
void flushLEDs() {
  // FIRE pattern uses analogWrite on every pin with per-LED brightness.
  if (currentPatternId == PAT_FIRE) {
    analogWrite(LED_GREEN,  fire_brightness[0]);
    analogWrite(LED_YELLOW, fire_brightness[1]);
    analogWrite(LED_RED,    fire_brightness[2]);
    return;
  }
  // Discrete patterns: each pin is bound to its own color identity.
  for (int i = 0; i < NUM_LEDS; i++) {
    int pin; Color expected;
    if (i == 0)      { pin = LED_GREEN;  expected = GREEN;  }
    else if (i == 1) { pin = LED_YELLOW; expected = YELLOW; }
    else             { pin = LED_RED;    expected = RED;    }
    switch (led_buffer[i]) {
      case OFF:    digitalWrite(pin, LOW);  break;
      case GREEN:
      case YELLOW:
      case RED:    digitalWrite(pin, (led_buffer[i] == expected) ? HIGH : LOW); break;
    }
  }
}

void clearLedBuffer() { for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = OFF; }

// ===== Sensor reading =====
void readTMP36() {
  // analogReadMilliVolts returns calibrated mV on ESP32-S3
  int mV = analogReadMilliVolts(TMP36_PIN);
  float t = (mV - 500) / 10.0;             // TMP36: T(C) = (mV-500)/10
  if (!(t > -40.0 && t < 125.0)) return;   // runtime range clamp
  tmp36Temp = t;

  // Auto-relock: if the saved baseline is way off the current reading
  // for 5 consecutive samples, the wiring or sensor changed -- clear
  // the stale baseline so it relocks at the new ambient.
  if (baselineSet && fabs(t - baselineTemp) > 30.0) {
    tmp36RelockCount++;
    if (tmp36RelockCount >= 5) {
      dln(startupDBG, ">>> Baseline drift > 30 degC for 5 samples - relocking.");
      baselineSet       = false;
      tmp36WarmupCount  = 0;
      tmp36LockCount    = 0;
      tmp36RelockCount  = 0;
      prefs.remove("baseline");
    }
  } else {
    tmp36RelockCount = 0;
  }

  // Baseline: discard first 5 samples, then require 3 consecutive
  // readings within 1 degC before locking.
  if (!baselineSet) {
    if (tmp36WarmupCount < 5) { tmp36WarmupCount++; tmp36LastReading = t; return; }
    if (fabs(t - tmp36LastReading) < 1.0) tmp36LockCount++;
    else                                  tmp36LockCount = 0;
    tmp36LastReading = t;
    if (tmp36LockCount >= 3) {
      baselineTemp = t;
      baselineSet  = true;
      saveBaseline(t);
      dbg(startupDBG, "Baseline temp locked: ");
      dln(startupDBG, String(baselineTemp, 1));
    }
    return;
  }

  // Auto-switch (extra credit): heat spike -> FIRE
  // Sanity gate: if either side of the delta is at the sensor floor,
  // the wiring is faulty (floating ADC). Skip auto-switch entirely.
  if (tmp36Temp < -30.0 || baselineTemp < -30.0) {
    heatSpikeCount = 0;
    return;
  }
  // User-override cooldown: any explicit command suppresses auto-switch
  // for 30 s so the user's choice actually sticks.
  if (millis() < userOverrideUntil) {
    heatSpikeCount = 0;
    return;
  }
  if ((tmp36Temp - baselineTemp) > HEAT_SPIKE_DELTA) {
    heatSpikeCount++;
    if (heatSpikeCount >= HEAT_SPIKE_NEEDED && currentPatternId != PAT_FIRE) {
      dln(loopDBG, ">>> Heat spike detected - auto-switching to FIRE");
      setPatternById(PAT_FIRE, "auto");
    }
  } else {
    heatSpikeCount = 0;
  }
}

// NTC voltage-divider read with Steinhart/beta equation (mirrors Ex12).
// Wiring: 3V - NTC_REF_OHMS - junction - NTC - GND, junction -> NTC_PIN.
void readNTC() {
  // Stale fallback: must run on every call, NOT only after a successful
  // read - otherwise ntcTemp gets frozen at the last good value when
  // wiring breaks.
  if (ntcEverValid && (millis() - lastValidNTC_ms) > NTC_STALE_MS) {
    ntcTemp = NAN;
  }

  int adc = analogRead(NTC_PIN);
  if (adc <= 5 || adc >= (int)NTC_ADC_MAX - 5) return;  // floating/shorted
  float voltage    = adc * NTC_VREF / NTC_ADC_MAX;
  // Rtherm = Rref * Vout / (Vref - Vout)
  float resistance = (voltage * NTC_REF_OHMS) / (NTC_VREF - voltage);
  float tempK = 1.0 / ((log(resistance / NTC_NOMINAL_R) / NTC_BETA) +
                       (1.0 / (NTC_NOMINAL_T + 273.15)));
  float t = tempK - 273.15;
  if (!(t > -40.0 && t < 125.0)) return;               // sanity range
  ntcTemp        = t;
  lastValidNTC_ms = millis();
  ntcEverValid    = true;
}

// ===== Button handling =====
// 50 ms debounce, 800 ms multi-press window, triple-press cycles to next
// pattern. Held LOW > BTN_STUCK_MS -> ignored until released.
void handleButton() {
  bool reading = digitalRead(BTN_PIN);
  unsigned long now = millis();

  // Stuck-LOW guard
  if (reading == LOW) {
    if (btnLowSince == 0) btnLowSince = now;
    if (!btnIgnored && (now - btnLowSince) > BTN_STUCK_MS) {
      btnIgnored = true;
      pressCount = 0;
      dln(loopDBG, "Button stuck LOW - ignoring until release.");
    }
  } else {
    btnLowSince = 0;
    btnIgnored  = false;
  }
  if (btnIgnored) { lastBtnReading = reading; return; }

  // Standard debounce
  if (reading != lastBtnReading) debounceT = now;
  lastBtnReading = reading;

  if ((now - debounceT) > DEBOUNCE_MS) {
    if (reading == LOW && btnState == HIGH) {
      totalPresses++; pressCount++;
      if (pressCount == 1) firstPressT = now;
      dbg(loopDBG, "Button press #"); dln(loopDBG, String(pressCount));
    }
    btnState = reading;
  }

  // Multi-press window expiry: 3+ presses force RAINBOW (traffic-light feel).
  if (pressCount > 0 && (now - firstPressT) > MULTI_PRESS_WINDOW) {
    if (pressCount >= 3) {
      dln(loopDBG, ">>> Triple press - rainbow.");
      setPatternById(PAT_RAINBOW, "button");
    }
    pressCount = 0;
  }
}

// ===== Pattern ID helpers =====
void setPatternById(PatternId p, const char* source) {
  if (p == currentPatternId) return;
  currentPatternId = p;
  pendingAck = patternToString(p);  // tell Flask what we're now running
  // Any explicit command (button / serial / flask) locks out auto-switch
  // for 30 s so the user-chosen pattern actually sticks.
  if (strcmp(source, "auto") != 0) {
    userOverrideUntil = millis() + 30000;
  }
  dbg(loopDBG, "["); dbg(loopDBG, source); dbg(loopDBG, "] -> ");
  dln(loopDBG, pendingAck);
}

PatternId patternFromString(const String& s) {
  String n = s; n.toLowerCase();
  if (n == "blink")        return PAT_BLINK;
  if (n == "rainbow")      return PAT_RAINBOW;
  if (n == "chase")        return PAT_CHASE;
  if (n == "fire")         return PAT_FIRE;
  if (n == "temperature")  return PAT_TEMPERATURE;
  if (n == "color_red"    || n == "red")    return PAT_COLOR_RED;
  if (n == "color_green"  || n == "green")  return PAT_COLOR_GREEN;
  if (n == "color_yellow" || n == "yellow") return PAT_COLOR_YELLOW;
  if (n == "all_off"      || n == "alloff" || n == "off") return PAT_ALL_OFF;
  return PAT_COUNT;  // sentinel for "unknown"
}

String patternToString(PatternId p) {
  switch (p) {
    case PAT_BLINK:        return "blink";
    case PAT_RAINBOW:      return "rainbow";
    case PAT_CHASE:        return "chase";
    case PAT_FIRE:         return "fire";
    case PAT_TEMPERATURE:  return "temperature";
    case PAT_COLOR_RED:    return "color_red";
    case PAT_COLOR_GREEN:  return "color_green";
    case PAT_COLOR_YELLOW: return "color_yellow";
    case PAT_ALL_OFF:      return "all_off";
    default:               return "unknown";
  }
}

// ===== Wi-Fi - bounded connect + non-blocking reconnect =====
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);  // setup-only - allowed by spec
  dbg(startupDBG, "Connecting to WiFi: "); dln(startupDBG, WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Bounded 5 s: 10 attempts of 500 ms - Ex11 dot-loop idiom but capped.
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 10) {
    delay(500);  // setup-only
    dbg(startupDBG, ".");
    tries++;
  }
  dln(startupDBG, "");
  if (WiFi.status() == WL_CONNECTED) {
    dbg(startupDBG, "Connected! IP: "); dln(startupDBG, WiFi.localIP().toString());
  } else {
    dln(startupDBG, "WiFi timeout - running offline; will retry in loop.");
  }
}

void maybeReconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastWifiRetry < WIFI_RETRY_MS) return;
  lastWifiRetry = now;
  WiFi.reconnect();   // non-blocking
  dln(loopDBG, "WiFi.reconnect() fired");
}

// ===== Flask communication - HTTP POST /api/data, GET /api/command =====
void sendToFlask() {
  HTTPClient http;
  String url = "http://" + String(FLASK_HOST) + ":" + String(FLASK_PORT) + "/api/data";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(2000);
  int code = http.POST(buildJSON());
  if (code > 0) {
    dbg(loopDBG, "POST -> Flask: "); dln(loopDBG, String(code));
    if (code == 200) pendingAck = "";  // Flask saw the new pattern
  } else {
    dbg(loopDBG, "POST failed: "); dln(loopDBG, http.errorToString(code));
  }
  http.end();
}

void checkFlaskCommand() {
  HTTPClient http;
  String url = "http://" + String(FLASK_HOST) + ":" + String(FLASK_PORT) + "/api/command";
  http.begin(url);
  http.setTimeout(2000);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();
    // Manual parse of {"pattern":"xxx"} - house style, no ArduinoJson.
    int idx = resp.indexOf("\"pattern\":\"");
    if (idx >= 0) {
      int start = idx + 11;
      int end   = resp.indexOf("\"", start);
      if (end > start) {
        String cmd = resp.substring(start, end);
        if (cmd.length() > 0 && cmd != "none") {
          PatternId p = patternFromString(cmd);
          if (p != PAT_COUNT) setPatternById(p, "flask");
          else { dbg(loopDBG, "Unknown Flask pattern: "); dln(loopDBG, cmd); }
        }
      }
    }
  }
  http.end();
}

// ===== Serial command interface (dev fallback over USB) =====
// Format: CMD:<pattern>   e.g. "CMD:rainbow"
void checkSerialCommand() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("CMD:")) {
      String cmd = line.substring(4); cmd.trim();
      PatternId p = patternFromString(cmd);
      if (p != PAT_COUNT) {
        Serial.println("ACK:" + patternToString(p));
        setPatternById(p, "serial");
      } else {
        Serial.println("ERR:unknown_pattern");
      }
    }
  }
}

// ===== JSON builder - String concat with manual escapes (house style) =====
String buildJSON() {
  String json = "{";
  json += "\"tmp36\":" + String(tmp36Temp, 2) + ",";
  if (isnan(ntcTemp))     json += "\"dht_temp\":null,";
  else                    json += "\"dht_temp\":" + String(ntcTemp, 2) + ",";
  json += "\"dht_hum\":null,";   // NTC has no humidity — kept null for schema compat
  json += "\"baseline\":" + String(baselineTemp, 2) + ",";
  json += "\"baseline_set\":" + String(baselineSet ? "true" : "false") + ",";
  float delta = baselineSet ? fabs(tmp36Temp - baselineTemp) : 0.0;
  json += "\"delta\":" + String(delta, 2) + ",";
  json += "\"btn_total\":" + String(totalPresses) + ",";
  json += "\"pattern\":\"" + patternToString(currentPatternId) + "\"";
  if (pendingAck.length() > 0) json += ",\"ack\":\"" + pendingAck + "\"";
  unsigned long ntcAge = (lastValidNTC_ms == 0) ? 0 : (millis() - lastValidNTC_ms);
  json += ",\"dht_age_ms\":" + String(ntcAge);
  json += "}";
  return json;
}

// ===== NVS helpers - persist baseline via Preferences =====
void loadBaseline() {
  if (prefs.isKey("baseline")) {
    float v = prefs.getFloat("baseline", NAN);
    if (!isnan(v) && v > -40.0 && v < 125.0) {
      baselineTemp = v;
      baselineSet  = true;
      dbg(startupDBG, "Loaded baseline from NVS: ");
      dln(startupDBG, String(baselineTemp, 1));
    }
  } else {
    dln(startupDBG, "No NVS baseline yet; will lock from first stable reads.");
  }
}

void saveBaseline(float v) { prefs.putFloat("baseline", v); }
