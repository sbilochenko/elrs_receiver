
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// ---------- CRSF / USB ----------
static constexpr int CRSF_RX_PIN = 16;
static constexpr int CRSF_TX_PIN = 17;
static constexpr uint32_t CRSF_BAUD = 420000;
static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t OUTPUT_PERIOD_MS = 20;
static constexpr uint32_t CHANNEL_TIMEOUT_MS = 150;

// ---------- Wi-Fi setup AP ----------
// Зміни пароль на власний: щонайменше 8 символів.
static const char *AP_SSID = "ELRS-Setup";
static const char *AP_PASSWORD = "2128506Za";

WebServer server(80);
Preferences preferences;

// ---------- Calibration ----------
struct Calibration {
  int ch1Min = 174;
  int ch1Center = 992;
  int ch1Max = 1811;

  int ch3Min = 191;
  int ch3Center = 907;
  int ch3Max = 1811;

  float deadZone = 0.05f;
};

Calibration cal;
Calibration draftCal;

uint16_t channels[16] = {};
bool haveChannels = false;
uint32_t lastChannelsMs = 0;
uint32_t lastOutputMs = 0;

uint8_t frame[64];
size_t frameSize = 0;
uint32_t crcErrors = 0;
uint32_t validFrames = 0;

// ---------- CRSF decoder ----------
uint8_t crc8DvbS2(const uint8_t *data, size_t len) {
  uint8_t crc = 0;

  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80)
        ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
        : static_cast<uint8_t>(crc << 1);
    }
  }

  return crc;
}

bool decodeChannels(const uint8_t *p, size_t len) {
  if (len != 22) return false;

  uint32_t bits = 0;
  int bitCount = 0;
  size_t pos = 0;

  for (int ch = 0; ch < 16; ++ch) {
    while (bitCount < 11) {
      bits |= static_cast<uint32_t>(p[pos++]) << bitCount;
      bitCount += 8;
    }

    channels[ch] = bits & 0x07FF;
    bits >>= 11;
    bitCount -= 11;
  }

  return true;
}

bool processFrame(const uint8_t *f, size_t total) {
  const uint8_t length = f[1];
  const uint8_t type = f[2];

  const uint8_t expectedCrc = f[total - 1];
  const uint8_t actualCrc = crc8DvbS2(&f[2], length - 1);

  if (actualCrc != expectedCrc) {
    ++crcErrors;
    return false;
  }

  if (type == 0x16 && length == 24) {
    if (decodeChannels(&f[3], 22)) {
      haveChannels = true;
      lastChannelsMs = millis();
      ++validFrames;
    }
  }

  return true;
}

void readCrsf() {
  while (Serial2.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial2.read());

    if (frameSize >= sizeof(frame)) {
      frameSize = 0;
    }

    frame[frameSize++] = b;

    while (frameSize >= 2) {
      const uint8_t length = frame[1];

      if (length < 2 || length > 62) {
        memmove(frame, frame + 1, --frameSize);
        continue;
      }

      const size_t total = static_cast<size_t>(length) + 2;
      if (frameSize < total) break;

      if (!processFrame(frame, total)) {
        // A bad candidate may have started mid-stream. Drop one byte and
        // search again so a valid frame inside the candidate is not skipped.
        memmove(frame, frame + 1, --frameSize);
        continue;
      }

      frameSize -= total;
      if (frameSize > 0) {
        memmove(frame, frame + total, frameSize);
      }
    }
  }
}

// ---------- Calibration math ----------
float normalizeChannel(int value, int minValue,
                       int centerValue, int maxValue) {
  float result = 0.0f;

  if (value >= centerValue) {
    const int range = maxValue - centerValue;
    if (range > 0) {
      result = static_cast<float>(value - centerValue) / range;
    }
  } else {
    const int range = centerValue - minValue;
    if (range > 0) {
      result = -static_cast<float>(centerValue - value) / range;
    }
  }

  return constrain(result, -1.0f, 1.0f);
}

float applyDeadZone(float value) {
  if (fabsf(value) <= cal.deadZone) return 0.0f;

  const float adjusted =
      (fabsf(value) - cal.deadZone) / (1.0f - cal.deadZone);

  return copysignf(adjusted, value);
}

bool signalValid() {
  return haveChannels &&
         (millis() - lastChannelsMs <= CHANNEL_TIMEOUT_MS);
}

float getLinear() {
  if (!signalValid()) return 0.0f;

  return applyDeadZone(
      normalizeChannel(channels[2],
                       cal.ch3Min, cal.ch3Center, cal.ch3Max));
}

float getAngular() {
  if (!signalValid()) return 0.0f;

  return applyDeadZone(
      normalizeChannel(channels[0],
                       cal.ch1Min, cal.ch1Center, cal.ch1Max));
}

// ---------- Preferences ----------
bool validCalibration(const Calibration &candidate) {
  return candidate.ch1Min >= 0 && candidate.ch1Max <= 2047 &&
         candidate.ch3Min >= 0 && candidate.ch3Max <= 2047 &&
         candidate.ch1Min < candidate.ch1Center &&
         candidate.ch1Center < candidate.ch1Max &&
         candidate.ch3Min < candidate.ch3Center &&
         candidate.ch3Center < candidate.ch3Max &&
         isfinite(candidate.deadZone) &&
         candidate.deadZone >= 0.0f && candidate.deadZone <= 0.25f;
}

void loadCalibration() {
  preferences.begin("elrs-cal", true);

  cal.ch1Min = preferences.getInt("c1min", 174);
  cal.ch1Center = preferences.getInt("c1ctr", 992);
  cal.ch1Max = preferences.getInt("c1max", 1811);

  cal.ch3Min = preferences.getInt("c3min", 191);
  cal.ch3Center = preferences.getInt("c3ctr", 907);
  cal.ch3Max = preferences.getInt("c3max", 1811);

  cal.deadZone = preferences.getFloat("dead", 0.05f);

  preferences.end();

  if (!validCalibration(cal)) {
    cal = Calibration{};
  }

  draftCal = cal;
}

bool saveCalibration() {
  if (!validCalibration(cal)) {
    return false;
  }

  if (!preferences.begin("elrs-cal", false)) {
    return false;
  }

  bool ok = true;
  ok = (preferences.putInt("c1min", cal.ch1Min) > 0) && ok;
  ok = (preferences.putInt("c1ctr", cal.ch1Center) > 0) && ok;
  ok = (preferences.putInt("c1max", cal.ch1Max) > 0) && ok;
  ok = (preferences.putInt("c3min", cal.ch3Min) > 0) && ok;
  ok = (preferences.putInt("c3ctr", cal.ch3Center) > 0) && ok;
  ok = (preferences.putInt("c3max", cal.ch3Max) > 0) && ok;
  ok = (preferences.putFloat("dead", cal.deadZone) > 0) && ok;

  preferences.end();
  return ok;
}

// ---------- Web page ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="uk">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ELRS Setup</title>
<style>
body{font-family:system-ui,sans-serif;max-width:760px;margin:20px auto;
padding:0 14px;background:#f4f6f8;color:#18212b}
.card{background:white;padding:16px;margin:12px 0;border-radius:14px}
h1{font-size:1.6rem}h2{font-size:1.15rem}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
label{font-size:.85rem;color:#526170}
input{box-sizing:border-box;width:100%;padding:9px;margin:4px 0 10px}
button{padding:10px 12px;margin:4px 2px;border:0;border-radius:8px;
background:#174ea6;color:white;font-weight:600}
.secondary{background:#596579}
.value{font-size:1.4rem;font-weight:700}
.bar{height:18px;background:#e2e8f0;border-radius:9px;overflow:hidden}
.fill{height:100%;width:50%;background:#1976d2;transition:width .08s}
small{color:#526170}
</style>
</head>
<body>
<h1>ELRS — калібрування</h1>
<div class="card">
  <b>Стан приймача:</b> <span id="state">перевірка…</span>
  <p>CH1: <span class="value" id="raw1">—</span>
     &nbsp; CH3: <span class="value" id="raw3">—</span></p>
  <p>Linear: <span id="lin">—</span>
     &nbsp; Angular: <span id="ang">—</span></p>
  <small>Нормалізовані значення: −1 … +1</small>
  <div class="bar"><div id="bar3" class="fill"></div></div>
  <div class="bar" style="margin-top:8px"><div id="bar1" class="fill"></div></div>
</div>

<div class="card">
<h2>CH1 — правий стик, горизонталь</h2>
<div class="grid">
  <label>Мінімум<input id="c1min" type="number"></label>
  <label>Центр<input id="c1ctr" type="number"></label>
  <label>Максимум<input id="c1max" type="number"></label>
</div>
<button onclick="capture('c1min')">Захопити лівий край</button>
<button onclick="capture('c1ctr')">Захопити центр</button>
<button onclick="capture('c1max')">Захопити правий край</button>
</div>

<div class="card">
<h2>CH3 — лівий стик, вертикаль</h2>
<div class="grid">
  <label>Мінімум<input id="c3min" type="number"></label>
  <label>Центр<input id="c3ctr" type="number"></label>
  <label>Максимум<input id="c3max" type="number"></label>
</div>
<button onclick="capture('c3min')">Захопити нижній край</button>
<button onclick="capture('c3ctr')">Захопити центр</button>
<button onclick="capture('c3max')">Захопити верхній край</button>
</div>

<div class="card">
<h2>Dead zone</h2>
<label>Розмір: <span id="deadLabel">0.05</span>
<input id="dead" type="range" min="0" max="0.25" step="0.01"
       value="0.05" oninput="deadLabel.textContent=Number(this.value).toFixed(2)">
</label>
<button onclick="applyConfig()">Застосувати параметри</button>
<button onclick="saveConfig()">Зберегти у ESP32</button>
<button class="secondary" onclick="loadConfig()">Перечитати з ESP32</button>
<p id="message"></p>
<small>Захоплюй краї, рухаючи стик до упору. Для центра відпусти стик у нейтральне положення. Потім натисни «Застосувати» та «Зберегти».</small>
</div>

<script>
const ids=['c1min','c1ctr','c1max','c3min','c3ctr','c3max'];
let latest=null;

async function api(path){
  const r=await fetch(path);
  if(!r.ok) throw new Error(await r.text());
  return await r.json();
}
function setFields(d){
  for(const id of ids) document.getElementById(id).value=d[id];
  document.getElementById('dead').value=d.dead;
  document.getElementById('deadLabel').textContent=Number(d.dead).toFixed(2);
}
async function loadConfig(){
  try{setFields(await api('/api/status')); message('Налаштування завантажено');}
  catch(e){message('Помилка: '+e.message);}
}
async function capture(field){
  if(!latest || !latest.valid){message('Немає валідного сигналу ELRS');return;}
  try{
    const d=await api('/api/capture?field='+field);
    setFields(d);
    message('Захоплено '+field+' = '+d[field]);
  }catch(e){message('Помилка: '+e.message);}
}
async function applyConfig(showMessage=true){
  const p=new URLSearchParams();
  for(const id of ids)p.set(id,document.getElementById(id).value);
  p.set('dead',document.getElementById('dead').value);
  try{
    setFields(await api('/api/config?'+p.toString()));
    if(showMessage)message('Параметри застосовано');
    return true;
  }catch(e){
    message('Помилка: '+e.message);
    return false;
  }
}
async function saveConfig(){
  try{
    // Never persist the previous active calibration if applying the current
    // values failed validation.
    if(!await applyConfig(false)) return;
    const d=await api('/api/save');
    message(d.message);
  }catch(e){message('Помилка: '+e.message);}
}
function message(s){document.getElementById('message').textContent=s;}
async function update(){
  try{
    const d=await api('/api/status');latest=d;
    document.getElementById('state').textContent=d.valid?'СИГНАЛ OK':'НЕМАЄ СВІЖИХ КАДРІВ';
    document.getElementById('raw1').textContent=d.ch1;
    document.getElementById('raw3').textContent=d.ch3;
    document.getElementById('lin').textContent=d.linear.toFixed(3);
    document.getElementById('ang').textContent=d.angular.toFixed(3);
    document.getElementById('bar3').style.width=((d.linear+1)*50)+'%';
    document.getElementById('bar1').style.width=((d.angular+1)*50)+'%';
  }catch(e){document.getElementById('state').textContent='Помилка зв’язку';}
}
loadConfig();update();setInterval(update,250);
</script>
</body>
</html>
)rawliteral";

void sendConfigJson(bool includeLive) {
  String json = "{";
  json += "\"c1min\":" + String(draftCal.ch1Min) + ",";
  json += "\"c1ctr\":" + String(draftCal.ch1Center) + ",";
  json += "\"c1max\":" + String(draftCal.ch1Max) + ",";
  json += "\"c3min\":" + String(draftCal.ch3Min) + ",";
  json += "\"c3ctr\":" + String(draftCal.ch3Center) + ",";
  json += "\"c3max\":" + String(draftCal.ch3Max) + ",";
  json += "\"dead\":" + String(draftCal.deadZone, 3);

  if (includeLive) {
    json += ",\"valid\":" + String(signalValid() ? "true" : "false");
    json += ",\"ch1\":" + String(channels[0]);
    json += ",\"ch3\":" + String(channels[2]);
    json += ",\"linear\":" + String(getLinear(), 4);
    json += ",\"angular\":" + String(getAngular(), 4);
  }

  json += "}";
  server.send(200, "application/json", json);
}

bool readConfigFromRequest(Calibration &next) {
  if (!server.hasArg("c1min") || !server.hasArg("c1ctr") ||
      !server.hasArg("c1max") || !server.hasArg("c3min") ||
      !server.hasArg("c3ctr") || !server.hasArg("c3max") ||
      !server.hasArg("dead")) {
    return false;
  }

  next.ch1Min = server.arg("c1min").toInt();
  next.ch1Center = server.arg("c1ctr").toInt();
  next.ch1Max = server.arg("c1max").toInt();
  next.ch3Min = server.arg("c3min").toInt();
  next.ch3Center = server.arg("c3ctr").toInt();
  next.ch3Max = server.arg("c3max").toInt();
  next.deadZone = server.arg("dead").toFloat();

  return validCalibration(next);
}
bool authorizeCalibrationRequest() {
  if (!server.authenticate("admin", AP_PASSWORD)) {
    server.requestAuthentication(BASIC_AUTH, "ELRS Calibration");
    return false;
  }

  return true;
}
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  server.on("/api/status", HTTP_GET, []() {
    sendConfigJson(true);
  });

  server.on("/api/capture", HTTP_GET, []() {
    if (!signalValid()) {
      server.send(409, "application/json",
                  "{\"error\":\"No valid RC signal\"}");
      return;
    }

    const String field = server.arg("field");
    const int ch1 = channels[0];
    const int ch3 = channels[2];

    if (field == "c1min") draftCal.ch1Min = ch1;
    else if (field == "c1ctr") draftCal.ch1Center = ch1;
    else if (field == "c1max") draftCal.ch1Max = ch1;
    else if (field == "c3min") draftCal.ch3Min = ch3;
    else if (field == "c3ctr") draftCal.ch3Center = ch3;
    else if (field == "c3max") draftCal.ch3Max = ch3;
    else {
      server.send(400, "application/json", "{\"error\":\"Bad field\"}");
      return;
    }

    sendConfigJson(false);
  });

  server.on("/api/config", HTTP_GET, []() {
    Calibration next = draftCal;

    if (!readConfigFromRequest(next)) {
      server.send(400, "application/json",
                  "{\"error\":\"Invalid calibration values\"}");
      return;
    }

    cal = next;
    draftCal = next;
    sendConfigJson(false);
  });

  server.on("/api/save", HTTP_GET, []() {
    if (!saveCalibration()) {
      server.send(400, "application/json",
                  "{\"error\":\"Calibration invalid; not saved\"}");
      return;
    }

    server.send(200, "application/json",
                "{\"message\":\"Налаштування збережено у ESP32\"}");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
}

void setup() {
  Serial.begin(USB_BAUD);
  Serial2.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  loadCalibration();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  setupWebServer();
}

void loop() {
  readCrsf();
  server.handleClient();

  const uint32_t now = millis();
  if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
  lastOutputMs = now;

  const bool valid = signalValid();

  const float linear = valid ? getLinear() : 0.0f;
  const float angular = valid ? getAngular() : 0.0f;

  // USB Serial: RC,valid,ch1,ch2,...,ch16
  Serial.printf("RC,%d", valid ? 1 : 0);

  for (int i = 0; i < 16; ++i) {
    Serial.printf(",%u", valid ? channels[i] : 0);
  }

  Serial.println();
}
