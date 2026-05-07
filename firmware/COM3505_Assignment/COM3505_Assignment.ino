// COM3505_Assignment.ino
// COM3505 — IoT Assignment: self-hosted ESP32 sensor & LED dashboard.
//
// All-in-one. Reads TMP36 + push button, drives three LEDs through a
// non-blocking pattern engine (9 patterns), and serves its own dashboard
// + JSON API at http://<board-ip>/. The whole project fits in this single
// .ino — no external server runtime required.
//
// Circuit (Adafruit Feather ESP32-S3):
//   TMP36  Vs->3V, Vout->GPIO 8 (A5/ADC1), GND->GND
//   Button GPIO 6 to GND, INPUT_PULLUP, active LOW
//   LEDs   Green=GPIO 9, Yellow=GPIO 10, Red=GPIO 11, each via 120R to GND.

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ===== Wi-Fi credentials =====
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ===== Pin map =====
#define TMP36_PIN     8
#define BTN_PIN       6
#define LED_GREEN     9
#define LED_YELLOW   10
#define LED_RED      11
#define NUM_LEDS      3

// ===== Timing (ms) =====
#define TMP36_INTERVAL    100
#define HISTORY_INTERVAL 2000
#define WIFI_RETRY_MS    5000
#define DEBOUNCE_MS         50
#define MULTI_PRESS_WINDOW 800
#define BTN_STUCK_MS      3000
#define HEAT_SPIKE_DELTA   5.0
#define HEAT_SPIKE_NEEDED    3
#define USER_OVERRIDE_MS 30000

// ===== Color + Pattern enums =====
enum Color : uint8_t { OFF = 0, GREEN = 1, YELLOW = 2, RED = 3 };

enum PatternId : uint8_t {
  PAT_BLINK = 0, PAT_RAINBOW, PAT_CHASE, PAT_FIRE, PAT_TEMPERATURE,
  PAT_COLOR_RED, PAT_COLOR_GREEN, PAT_COLOR_YELLOW, PAT_ALL_OFF, PAT_COUNT
};

// Single source of truth - used by both patternFromString and patternToString.
struct PatternEntry { PatternId id; const char* name; };
static const PatternEntry PATTERN_TABLE[] = {
  { PAT_BLINK,        "blink"        },
  { PAT_RAINBOW,      "rainbow"      },
  { PAT_CHASE,        "chase"        },
  { PAT_FIRE,         "fire"         },
  { PAT_TEMPERATURE,  "temperature"  },
  { PAT_COLOR_RED,    "color_red"    },
  { PAT_COLOR_GREEN,  "color_green"  },
  { PAT_COLOR_YELLOW, "color_yellow" },
  { PAT_ALL_OFF,      "all_off"      },
};
static const size_t PATTERN_TABLE_LEN = sizeof(PATTERN_TABLE) / sizeof(PATTERN_TABLE[0]);

// ===== Globals =====
WebServer   webServer(80);
Preferences prefs;

Color     led_buffer[NUM_LEDS]      = { OFF, OFF, OFF };
uint8_t   fire_brightness[NUM_LEDS] = { 0, 0, 0 };
PatternId currentPatternId = PAT_TEMPERATURE;
PatternId lastPatternId    = PAT_COUNT;
uint16_t  patternStep      = 0;
unsigned long patternLastTick   = 0;
bool          led_dirty         = true;
unsigned long userOverrideUntil = 0;
unsigned long lastWifiRetry     = 0;

// TMP36 / baseline lock state
float tmp36Temp        = 0.0;
float baselineTemp     = 0.0;
bool  baselineSet      = false;
int   tmp36WarmupCount = 0;
int   tmp36LockCount   = 0;
float tmp36LastReading = 0.0;
int   heatSpikeCount   = 0;
int   tmp36RelockCount = 0;
unsigned long lastTmp36Read = 0;

// Button
bool          lastBtnReading = HIGH;
bool          btnState       = HIGH;
unsigned long debounceT      = 0;
int           pressCount     = 0;
unsigned long firstPressT    = 0;
unsigned long totalPresses   = 0;
unsigned long btnLowSince    = 0;
bool          btnIgnored     = false;

// History circular buffer (drives the chart on the dashboard)
struct HistorySample { float tmp36; uint32_t ts_s; };
#define HISTORY_SIZE 60
HistorySample history_buf[HISTORY_SIZE];
int history_idx = 0;
int history_cnt = 0;
unsigned long lastHistoryPush = 0;

// ===== Forward declarations =====
void readTMP36();
void handleButton();
void patternTick(unsigned long now);
void tickBlink(unsigned long now);
void tickRainbow(unsigned long now);
void tickChase(unsigned long now);
void tickFire(unsigned long now);
void tickTemperature(unsigned long now);
void tickSolid(Color c);
void flushLEDs();
void clearLedBuffer();
void setPatternById(PatternId p, const char* source);
PatternId   patternFromString(const char* s);
const char* patternToString(PatternId p);
void connectWiFi();
void maybeReconnectWiFi();
void startWebServer();
void handleRoot();
void handleData();
void handleHistory();
void handleCmd();
void handleNotFound();
void historyPush(unsigned long now);
size_t buildJSON(char* buf, size_t cap);
void loadBaseline();
void saveBaseline(float v);

// ===== Embedded dashboard (HTML + CSS + JS, served from PROGMEM) =====
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TELEMETRY // COM3505 ESP32 NODE-01</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:#14181a;color:#cfcfc4;font:13px 'SF Mono','Menlo','Consolas',monospace;letter-spacing:.02em}
.bezel,.panel{border:1px solid #0a0c0a;border-top:1px solid #495149;border-left:1px solid #495149}
.bezel{max-width:1100px;margin:18px auto;padding:14px;background:linear-gradient(#1f2422,#14181a)}
.rack-head .bar{display:flex;justify-content:space-between;align-items:center;font-size:11px;text-transform:uppercase;color:#6f7670}
.brand{color:#d8a14a;font-weight:700;letter-spacing:.18em}
.title{color:#cfcfc4;letter-spacing:.12em}
.link{display:inline-flex;align-items:center;gap:6px}
.link .led{width:8px;height:8px;background:#d6493a;border:1px solid #0a0c0a}
.link .led.ok{background:#6ee06b}
.rule{color:#495149;margin:4px 0 12px;user-select:none;overflow:hidden}
.grid{display:grid;gap:10px;grid-template-columns:repeat(2,1fr)}
.panel{background:#1b1f1d;padding:10px 12px;position:relative}
.panel::before{content:"";position:absolute;inset:4px;border:1px dashed #333a35;pointer-events:none}
.panel-head{display:flex;justify-content:space-between;font-size:10px;text-transform:uppercase;letter-spacing:.18em;color:#6f7670;margin-bottom:6px}
.addr{color:#6b4d18}
.big{font-size:38px;font-weight:700;color:#6ee06b;font-variant-numeric:tabular-nums lining-nums;letter-spacing:-.02em;margin:8px 0;min-height:1.05em}
.ticks{display:flex;justify-content:space-between;font-size:10px;color:#6f7670;border-top:1px solid #2a302c;padding-top:4px}
.leds{display:flex;gap:14px;margin-top:8px}
.bay{display:flex;align-items:center;gap:6px}
.bay label{font-size:10px;color:#6f7670;text-transform:uppercase;letter-spacing:.12em}
.led-rect{width:14px;height:8px;background:#1a1d1a;border:1px solid #0a0c0a;display:inline-block;transition:background-color 180ms ease-out}
.led-rect.on#led-g{background:#6ee06b}
.led-rect.on#led-y{background:#d8a14a}
.led-rect.on#led-r{background:#d6493a}
.chart{grid-column:1/-1}
.chart canvas{width:100%;height:220px;display:block;background:#0d1010}
.legend,.rack-foot{display:flex;font-size:10px;color:#6f7670;text-transform:uppercase;letter-spacing:.18em}
.legend{gap:16px;align-items:center;margin-top:6px}
.sw{display:inline-block;width:12px;height:2px;vertical-align:middle;margin-right:4px}
.sw.a{background:#d8a14a}
.now{margin-left:auto}.now b{color:#d8a14a}
.cmd{grid-column:1/-1}
.btns{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}
.btns button{background:#14181a;color:#cfcfc4;border:1px solid #495149;padding:8px 12px;font-family:inherit;font-size:11px;letter-spacing:.18em;text-transform:uppercase;cursor:pointer;transition:background-color .1s ease,color .1s ease}
.btns button:hover{background:#1f2422;color:#6ee06b}
.btns button.active{background:#2e6e2c;color:#fff;border-color:#6ee06b;outline:1px dashed #6ee06b}
.rack-foot{gap:8px;justify-content:center;margin-top:12px;letter-spacing:.2em}
</style>
</head>
<body>
<div class="bezel">
<header class="rack-head">
<div class="bar">
<span class="brand">[ COM3505 ]</span>
<span class="title">TELEMETRY CONSOLE &mdash; NODE&#8209;01 / ESP32</span>
<span class="link"><span class="led" id="dot"></span><span id="status-text">LINK</span></span>
</div>
<div class="rule">━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━</div>
</header>
<main class="grid">
<section class="panel">
<div class="panel-head"><span>CH-A &middot; TMP36</span><span class="addr">0x01</span></div>
<div class="big"><span id="tmp36">--.-</span> &deg;C</div>
<div class="ticks"><span>0</span><span>10</span><span>20</span><span>30</span><span>40</span></div>
</section>
<section class="panel">
<div class="panel-head"><span>CH-B &middot; PULSE COUNT</span><span class="addr">0x02</span></div>
<div class="big"><span id="btn_total">0</span></div>
<div class="ticks"><span>events</span><span>since boot</span></div>
</section>
<section class="panel">
<div class="panel-head"><span>CH-C &middot; &Delta; FROM BASELINE</span><span class="addr">0x03</span></div>
<div class="big"><span id="delta">+0.0</span> &deg;C</div>
<div class="leds">
<div class="bay"><span class="led-rect" id="led-g"></span><label>NOMINAL</label></div>
<div class="bay"><span class="led-rect" id="led-y"></span><label>WARN</label></div>
<div class="bay"><span class="led-rect" id="led-r"></span><label>ALARM</label></div>
</div>
</section>
<section class="panel chart">
<div class="panel-head"><span>TRACE &middot; CH-A vs t</span><span class="addr">SCOPE</span></div>
<canvas id="chart" width="900" height="260"></canvas>
<div class="legend">
<span><span class="sw a"></span>TMP36</span>
<span class="now">PATTERN: <b id="pattern">--</b></span>
</div>
</section>
<section class="panel cmd">
<div class="panel-head"><span>COMMAND BUS &middot; LED PATTERN</span><span class="addr">TX</span></div>
<div id="pattern-btns" class="btns">
<button data-pattern="blink">BLINK</button>
<button data-pattern="rainbow">RAINBOW</button>
<button data-pattern="chase">CHASE</button>
<button data-pattern="fire">FIRE</button>
<button data-pattern="temperature">TEMP</button>
<button data-pattern="color_red">RED</button>
<button data-pattern="color_green">GREEN</button>
<button data-pattern="color_yellow">YELLOW</button>
<button data-pattern="all_off">OFF</button>
</div>
</section>
</main>
<footer class="rack-foot">
<span>POLL 0.67 Hz</span><span>&#9475;</span><span>SRC /api/data</span><span>&#9475;</span><span>FW v3505.5</span>
</footer>
</div>
<script>
(function(){
const $=function(id){return document.getElementById(id);};
const els={tmp36:$('tmp36'),btn_total:$('btn_total'),delta:$('delta'),pattern:$('pattern'),dot:$('dot'),status:$('status-text'),ledG:$('led-g'),ledY:$('led-y'),ledR:$('led-r'),chart:$('chart')};
const ctx=els.chart.getContext('2d');
let h=[],ts=0,rq=false,W=0,H=0,poll=null;
function sz(){const x=window.devicePixelRatio||1;const r=els.chart.getBoundingClientRect();W=r.width;H=r.height;els.chart.width=W*x;els.chart.height=H*x;ctx.setTransform(x,0,0,x,0,0);sc();}
function sc(){if(rq)return;rq=true;requestAnimationFrame(dr);}
function dr(){rq=false;const p={t:16,r:16,b:26,l:44};ctx.clearRect(0,0,W,H);
if(h.length<2){ctx.fillStyle='#6f7670';ctx.font='11px monospace';ctx.textAlign='center';ctx.fillText('NO DATA — WAITING',W/2,H/2);return;}
const tmp=h.map(function(v){return v.tmp36;});
const all=tmp.filter(function(v){return v!=null&&!isNaN(v);});
let mn=Math.min.apply(null,all)-2,mx=Math.max.apply(null,all)+2;
if(mx-mn<5){mn-=2;mx+=2;}
const pw=W-p.l-p.r,ph=H-p.t-p.b;
ctx.strokeStyle='#3a423c';ctx.lineWidth=1;ctx.strokeRect(p.l,p.t,pw,ph);
ctx.strokeStyle='#26302a';ctx.lineWidth=.5;
for(let i=1;i<5;i++){const y=p.t+(ph/5)*i;ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(W-p.r,y);ctx.stroke();}
for(let i=1;i<10;i++){const x=p.l+(pw/10)*i;ctx.beginPath();ctx.moveTo(x,p.t);ctx.lineTo(x,p.t+ph);ctx.stroke();}
ctx.fillStyle='#6f7670';ctx.font='10px monospace';ctx.textAlign='right';
for(let i=0;i<=5;i++){const y=p.t+(ph/5)*i;ctx.fillText((mx-(mx-mn)*i/5).toFixed(1)+'°',p.l-6,y+3);}
function ln(d2,c){ctx.beginPath();ctx.strokeStyle=c;ctx.lineWidth=1.5;
d2.forEach(function(v,i){if(v==null||isNaN(v))return;const x=p.l+(i/(d2.length-1))*pw;const y=p.t+(1-(v-mn)/(mx-mn))*ph;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});
ctx.stroke();}
ln(tmp,'#d8a14a');}
function led(p,d2){const g=els.ledG,y=els.ledY,r=els.ledR;g.classList.remove('on');y.classList.remove('on');r.classList.remove('on');
if(p==='all_off')return;
if(p==='temperature'){g.classList.add('on');if(Math.abs(d2)>=2)y.classList.add('on');if(Math.abs(d2)>=5)r.classList.add('on');}
else if(p==='color_red')r.classList.add('on');
else if(p==='color_green')g.classList.add('on');
else if(p==='color_yellow')y.classList.add('on');
else{g.classList.add('on');y.classList.add('on');r.classList.add('on');}}
function hl(p){document.querySelectorAll('#pattern-btns button[data-pattern]').forEach(function(b){b.classList.toggle('active',b.dataset.pattern===p);});}
function apply(d2){els.tmp36.textContent=(d2.tmp36||0).toFixed(1);
els.btn_total.textContent=d2.btn_total||0;
const dt=d2.delta||0;els.delta.textContent=(dt>=0?'+':'')+dt.toFixed(1);
els.pattern.textContent=(d2.pattern||'--').toUpperCase();
led(d2.pattern,dt);hl(d2.pattern);
if(d2.ts&&d2.ts!==ts){ts=d2.ts;h.push(d2);if(h.length>120)h.shift();sc();}
els.dot.classList.add('ok');
const age=Math.round((Date.now()/1000)-(d2.ts||0));
els.status.textContent=d2.ts?('LINK · '+age+'s'):'WAITING';}
function down(){els.dot.classList.remove('ok');els.status.textContent='NO LINK';}
const btns=document.querySelectorAll('#pattern-btns button[data-pattern]');
btns.forEach(function(b){b.addEventListener('click',function(){btns.forEach(function(o){o.classList.toggle('active',o===b);});fetch('/api/cmd',{method:'POST',body:b.dataset.pattern});});});
async function once(){try{const r=await fetch('/api/data');apply(await r.json());}catch(e){down();}}
function start(){if(poll)return;once();poll=setInterval(once,1500);}
function stop(){if(poll){clearInterval(poll);poll=null;}}
document.addEventListener('visibilitychange',function(){if(document.hidden)stop();else start();});
(async function(){try{const r=await fetch('/api/history');h=await r.json();sc();}catch(e){}})();
sz();window.addEventListener('resize',sz);
start();
})();
</script>
</body>
</html>
)rawliteral";

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== COM3505 IoT Assignment (single-file) ===");

  analogReadResolution(12);
  analogSetPinAttenuation(TMP36_PIN, ADC_11db);

  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  clearLedBuffer();
  flushLEDs();

  pinMode(BTN_PIN, INPUT_PULLUP);

  prefs.begin("com3505", false);
  loadBaseline();

  connectWiFi();
  startWebServer();

  // Startup blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, HIGH);
    delay(120);
    digitalWrite(LED_GREEN, LOW);  digitalWrite(LED_YELLOW, LOW);  digitalWrite(LED_RED, LOW);
    delay(120);
  }
  Serial.println("System ready. Default pattern: TEMPERATURE.");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open dashboard at: http://");
    Serial.println(WiFi.localIP());
  }
}

// ===== LOOP =====
void loop() {
  unsigned long now = millis();
  readTMP36();
  handleButton();
  patternTick(now);
  flushLEDs();
  historyPush(now);
  webServer.handleClient();
  maybeReconnectWiFi();
}

// ===== Pattern engine =====
void patternTick(unsigned long now) {
  if (currentPatternId != lastPatternId) {
    // Leaving FIRE: detach LEDC PWM and revert pins to digital OUTPUT
    // (Arduino-ESP32 v3's implicit detach-on-digitalWrite is unreliable).
    if (lastPatternId == PAT_FIRE) {
      analogWrite(LED_GREEN,  0); analogWrite(LED_YELLOW, 0); analogWrite(LED_RED, 0);
      pinMode(LED_GREEN,  OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_RED, OUTPUT);
      digitalWrite(LED_GREEN, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED, LOW);
    }
    patternStep = 0;
    patternLastTick = 0;
    clearLedBuffer();
    for (int i = 0; i < NUM_LEDS; i++) fire_brightness[i] = 0;
    lastPatternId = currentPatternId;
    led_dirty = true;
    Serial.print("Pattern -> "); Serial.println(patternToString(currentPatternId));
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
    case PAT_ALL_OFF:      tickSolid(OFF);       break;
    default:               tickSolid(OFF);       break;
  }
}

void tickBlink(unsigned long now) {
  if (now - patternLastTick < 500 && patternLastTick != 0) return;
  patternLastTick = now;
  bool on = !(patternStep & 0x1);
  led_buffer[0] = on ? GREEN  : OFF;
  led_buffer[1] = on ? YELLOW : OFF;
  led_buffer[2] = on ? RED    : OFF;
  led_dirty = true;
  patternStep++;
}

void tickRainbow(unsigned long now) {
  if (now - patternLastTick < 250 && patternLastTick != 0) return;
  patternLastTick = now;
  static const Color seq[4] = { GREEN, YELLOW, RED, YELLOW };
  Color c = seq[patternStep & 0x3];
  for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = c;
  led_dirty = true;
  patternStep++;
}

void tickChase(unsigned long now) {
  if (now - patternLastTick < 200 && patternLastTick != 0) return;
  patternLastTick = now;
  static const Color seq[NUM_LEDS] = { GREEN, YELLOW, RED };
  for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = OFF;
  uint8_t pos = patternStep % NUM_LEDS;
  led_buffer[pos] = seq[pos];
  led_dirty = true;
  patternStep++;
}

void tickFire(unsigned long now) {
  if (now - patternLastTick < 60 && patternLastTick != 0) return;
  patternLastTick = now;
  led_buffer[0] = GREEN;  led_buffer[1] = YELLOW;  led_buffer[2] = RED;
  fire_brightness[0] = constrain(60  + (int)random(-30, 31), 0, 255);
  fire_brightness[1] = constrain(120 + (int)random(-50, 51), 0, 255);
  fire_brightness[2] = constrain(200 + (int)random(-40, 41), 0, 255);
  led_dirty = true;
  patternStep++;
}

void tickTemperature(unsigned long now) {
  if (now - patternLastTick < 200 && patternLastTick != 0) return;
  patternLastTick = now;
  if (!baselineSet) {
    led_buffer[0] = GREEN; led_buffer[1] = OFF; led_buffer[2] = OFF;
    led_dirty = true;
    return;
  }
  float delta = fabs(tmp36Temp - baselineTemp);
  if (delta < 2.0)      { led_buffer[0] = GREEN; led_buffer[1] = OFF;    led_buffer[2] = OFF; }
  else if (delta < 5.0) { led_buffer[0] = GREEN; led_buffer[1] = YELLOW; led_buffer[2] = OFF; }
  else                  { led_buffer[0] = GREEN; led_buffer[1] = YELLOW; led_buffer[2] = RED; }
  led_dirty = true;
}

void tickSolid(Color c) {
  led_buffer[0] = (c == GREEN)  ? GREEN  : OFF;
  led_buffer[1] = (c == YELLOW) ? YELLOW : OFF;
  led_buffer[2] = (c == RED)    ? RED    : OFF;
  led_dirty = true;
}

void clearLedBuffer() {
  for (int i = 0; i < NUM_LEDS; i++) led_buffer[i] = OFF;
  led_dirty = true;
}

// ===== flushLEDs (only paints when dirty; FIRE always renders frame) =====
void flushLEDs() {
  if (!led_dirty && currentPatternId != PAT_FIRE) return;
  if (currentPatternId == PAT_FIRE) {
    analogWrite(LED_GREEN,  fire_brightness[0]);
    analogWrite(LED_YELLOW, fire_brightness[1]);
    analogWrite(LED_RED,    fire_brightness[2]);
    led_dirty = false;
    return;
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    int pin; Color expected;
    if (i == 0)      { pin = LED_GREEN;  expected = GREEN;  }
    else if (i == 1) { pin = LED_YELLOW; expected = YELLOW; }
    else             { pin = LED_RED;    expected = RED;    }
    digitalWrite(pin, (led_buffer[i] == expected) ? HIGH : LOW);
  }
  led_dirty = false;
}

// ===== Sensors =====
void readTMP36() {
  unsigned long now = millis();
  if (now - lastTmp36Read < TMP36_INTERVAL) return;
  lastTmp36Read = now;

  int mV = analogReadMilliVolts(TMP36_PIN);
  float t = (mV - 500) / 10.0;
  if (!(t > -40.0 && t < 125.0)) return;
  tmp36Temp = t;

  // Auto-relock: if saved baseline is way off current readings for 5 in a row,
  // wiring/sensor changed -- wipe baseline so it locks at the new ambient.
  if (baselineSet && fabs(t - baselineTemp) > 30.0) {
    tmp36RelockCount++;
    if (tmp36RelockCount >= 5) {
      Serial.println(">>> Baseline drift > 30 degC for 5 samples - relocking.");
      baselineSet = false;
      tmp36WarmupCount = 0; tmp36LockCount = 0; tmp36RelockCount = 0;
      prefs.remove("baseline");
    }
  } else {
    tmp36RelockCount = 0;
  }

  // Baseline lock: discard first 20 samples (~2s warmup so TMP36 settles),
  // then require 5 consecutive readings within 0.5 degC before locking.
  if (!baselineSet) {
    if (tmp36WarmupCount < 20) { tmp36WarmupCount++; tmp36LastReading = t; return; }
    if (fabs(t - tmp36LastReading) < 0.5) tmp36LockCount++; else tmp36LockCount = 0;
    tmp36LastReading = t;
    if (tmp36LockCount >= 5) {
      baselineTemp = t; baselineSet = true; saveBaseline(t);
      Serial.print("Baseline locked: "); Serial.println(baselineTemp, 1);
    }
    return;
  }

  // Auto-switch to FIRE on heat spike (extra credit).
  if (tmp36Temp < -30.0 || baselineTemp < -30.0) { heatSpikeCount = 0; return; }
  if (millis() < userOverrideUntil)              { heatSpikeCount = 0; return; }
  if ((tmp36Temp - baselineTemp) > HEAT_SPIKE_DELTA) {
    heatSpikeCount++;
    if (heatSpikeCount >= HEAT_SPIKE_NEEDED && currentPatternId != PAT_FIRE) {
      Serial.println(">>> Heat spike - auto FIRE");
      setPatternById(PAT_FIRE, "auto");
    }
  } else {
    heatSpikeCount = 0;
  }
}

// ===== Button (50ms debounce, 800ms multi-press window, triple-press -> rainbow) =====
void handleButton() {
  bool reading = digitalRead(BTN_PIN);
  unsigned long now = millis();

  if (reading == LOW) {
    if (btnLowSince == 0) btnLowSince = now;
    if (!btnIgnored && (now - btnLowSince) > BTN_STUCK_MS) {
      btnIgnored = true; pressCount = 0;
      Serial.println("Button stuck LOW - ignoring until release.");
    }
  } else {
    btnLowSince = 0; btnIgnored = false;
  }
  if (btnIgnored) { lastBtnReading = reading; return; }

  if (reading != lastBtnReading) debounceT = now;
  lastBtnReading = reading;

  if ((now - debounceT) > DEBOUNCE_MS) {
    if (reading == LOW && btnState == HIGH) {
      totalPresses++; pressCount++;
      if (pressCount == 1) firstPressT = now;
    }
    btnState = reading;
  }

  if (pressCount > 0 && (now - firstPressT) > MULTI_PRESS_WINDOW) {
    if (pressCount >= 3) {
      Serial.println(">>> Triple press - rainbow.");
      setPatternById(PAT_RAINBOW, "button");
    }
    pressCount = 0;
  }
}

// ===== Pattern table helpers =====
void setPatternById(PatternId p, const char* source) {
  if (p == currentPatternId) return;
  currentPatternId = p;
  if (strcmp(source, "auto") != 0) userOverrideUntil = millis() + USER_OVERRIDE_MS;
  Serial.print("["); Serial.print(source); Serial.print("] -> "); Serial.println(patternToString(p));
}

PatternId patternFromString(const char* s) {
  if (!s || !*s) return PAT_COUNT;
  for (size_t i = 0; i < PATTERN_TABLE_LEN; i++) {
    if (strcmp(s, PATTERN_TABLE[i].name) == 0) return PATTERN_TABLE[i].id;
  }
  // Friendly aliases
  if (strcmp(s, "red")    == 0) return PAT_COLOR_RED;
  if (strcmp(s, "green")  == 0) return PAT_COLOR_GREEN;
  if (strcmp(s, "yellow") == 0) return PAT_COLOR_YELLOW;
  if (strcmp(s, "off")    == 0 || strcmp(s, "alloff") == 0) return PAT_ALL_OFF;
  return PAT_COUNT;
}

const char* patternToString(PatternId p) {
  for (size_t i = 0; i < PATTERN_TABLE_LEN; i++) {
    if (PATTERN_TABLE[i].id == p) return PATTERN_TABLE[i].name;
  }
  return "unknown";
}

// ===== Wi-Fi (bounded 5s connect, non-blocking reconnect in loop) =====
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);  // setup-only
  Serial.print("Connecting to WiFi: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 10) {
    delay(500); Serial.print("."); tries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi timeout - dashboard unreachable until reconnect.");
  }
}

void maybeReconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastWifiRetry < WIFI_RETRY_MS) return;
  lastWifiRetry = now;
  WiFi.reconnect();
}

// ===== Web server =====
void startWebServer() {
  webServer.on("/",            HTTP_GET,  handleRoot);
  webServer.on("/api/data",    HTTP_GET,  handleData);
  webServer.on("/api/history", HTTP_GET,  handleHistory);
  webServer.on("/api/cmd",     HTTP_POST, handleCmd);
  webServer.onNotFound(handleNotFound);
  webServer.begin();
  Serial.println("HTTP server listening on port 80.");
}

void handleRoot() {
  webServer.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  char buf[320];
  buildJSON(buf, sizeof(buf));
  webServer.send(200, "application/json", buf);
}

void handleHistory() {
  // Each sample serializes to ~50 bytes; 60 max -> ~3KB. Stream-build with String.
  char rec[80];
  String resp = "[";
  resp.reserve(HISTORY_SIZE * 64);
  bool first = true;
  for (int i = 0; i < history_cnt; i++) {
    int idx = (history_idx - history_cnt + i + HISTORY_SIZE) % HISTORY_SIZE;
    HistorySample& s = history_buf[idx];
    if (!first) resp += ",";
    first = false;
    snprintf(rec, sizeof(rec), "{\"tmp36\":%.2f,\"ts\":%lu}",
             s.tmp36, (unsigned long)s.ts_s);
    resp += rec;
  }
  resp += "]";
  webServer.send(200, "application/json", resp);
}

void handleCmd() {
  String body = webServer.arg("plain");
  body.trim();
  PatternId p = patternFromString(body.c_str());
  if (p != PAT_COUNT) {
    setPatternById(p, "web");
    char ok[64];
    snprintf(ok, sizeof(ok), "{\"status\":\"ok\",\"pattern\":\"%s\"}", patternToString(p));
    webServer.send(200, "application/json", ok);
  } else {
    webServer.send(400, "application/json", "{\"error\":\"unknown_pattern\"}");
  }
}

void handleNotFound() {
  webServer.send(404, "text/plain", "404 Not Found");
}

// ===== History push (called every loop, throttled to HISTORY_INTERVAL) =====
void historyPush(unsigned long now) {
  if (now - lastHistoryPush < HISTORY_INTERVAL) return;
  lastHistoryPush = now;
  HistorySample& s = history_buf[history_idx];
  s.tmp36 = tmp36Temp;
  s.ts_s  = (uint32_t)(now / 1000);
  history_idx = (history_idx + 1) % HISTORY_SIZE;
  if (history_cnt < HISTORY_SIZE) history_cnt++;
}

// ===== /api/data JSON builder (snprintf into caller buffer, no heap churn) =====
size_t buildJSON(char* buf, size_t cap) {
  float delta = baselineSet ? fabs(tmp36Temp - baselineTemp) : 0.0;
  unsigned long ts_s = millis() / 1000;
  return snprintf(buf, cap,
    "{\"tmp36\":%.2f,\"baseline\":%.2f,\"baseline_set\":%s,"
    "\"delta\":%.2f,\"btn_total\":%lu,\"pattern\":\"%s\",\"ts\":%lu}",
    tmp36Temp, baselineTemp, baselineSet ? "true" : "false",
    delta, totalPresses, patternToString(currentPatternId), ts_s);
}

// ===== NVS-persisted baseline =====
void loadBaseline() {
  if (prefs.isKey("baseline")) {
    float v = prefs.getFloat("baseline", NAN);
    if (!isnan(v) && v > -40.0 && v < 125.0) {
      baselineTemp = v; baselineSet = true;
      Serial.print("Loaded baseline from NVS: "); Serial.println(baselineTemp, 1);
    }
  }
}

void saveBaseline(float v) { prefs.putFloat("baseline", v); }
