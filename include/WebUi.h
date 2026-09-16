#pragma once
//
// Optional Wi-Fi web UI + OTA firmware update.
//
// Off by default: a switching-mode Wi-Fi radio a few centimetres from an SWR
// bridge is not free, and a tuner should work with no network at all. Turn it
// on with `wifi <ssid> <pass>` then `wifi on`.
//
// Note ADC2 is unusable while Wi-Fi is up. Both bridge inputs are on ADC1
// (GPIO 1-10), so this is safe here - but it is the reason the pin map insists
// on ADC1 for the forward/reverse channels.
//

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

#include "App.h"
#include "Commands.h"
#include "Config.h"
#include "Types.h"

namespace atu {

static const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ATU-1000</title><style>
:root{--bg:#11151a;--fg:#e6edf3;--mut:#8b98a5;--pan:#1a212a;--ac:#4aa8ff;--ok:#3fb950;--wn:#d29922;--er:#f85149;--bd:#2b3440}
*{box-sizing:border-box}body{margin:0;font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:14px 18px;border-bottom:1px solid var(--bd);display:flex;align-items:baseline;gap:12px;flex-wrap:wrap}
h1{font-size:17px;margin:0;letter-spacing:.5px}
.pill{font-size:12px;color:var(--mut);border:1px solid var(--bd);border-radius:99px;padding:2px 9px}
main{max-width:900px;margin:0 auto;padding:18px;display:grid;gap:16px}
.card{background:var(--pan);border:1px solid var(--bd);border-radius:10px;padding:16px}
.freq{font-size:40px;font-weight:600;letter-spacing:-1px;font-variant-numeric:tabular-nums}
.freq small{font-size:15px;color:var(--mut);font-weight:400;margin-left:6px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:14px}
.k{font-size:11px;color:var(--mut);text-transform:uppercase;letter-spacing:.6px}
.v{font-size:22px;font-variant-numeric:tabular-nums}
.bar{height:9px;background:#0d1117;border:1px solid var(--bd);border-radius:5px;overflow:hidden;margin-top:6px}
.bar i{display:block;height:100%;background:var(--ac);width:0;transition:width .25s}
.bar.w i{background:var(--wn)}.bar.e i{background:var(--er)}
button{font:inherit;padding:9px 15px;border-radius:7px;border:1px solid var(--bd);background:#222b36;color:var(--fg);cursor:pointer}
button:hover{border-color:var(--ac)}button.p{background:var(--ac);border-color:var(--ac);color:#04121f;font-weight:600}
.row{display:flex;gap:9px;flex-wrap:wrap}
input,select{font:inherit;padding:9px;border-radius:7px;border:1px solid var(--bd);background:#0d1117;color:var(--fg)}
input[type=text]{flex:1;min-width:200px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
pre{background:#0d1117;border:1px solid var(--bd);border-radius:7px;padding:11px;overflow:auto;max-height:320px;font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;white-space:pre-wrap;margin:10px 0 0}
.al{padding:10px 13px;border-radius:7px;font-weight:600;margin-bottom:12px}
.al.e{background:#3d1418;border:1px solid var(--er);color:#ffb3ad}
.al.w{background:#3a2d0c;border:1px solid var(--wn);color:#ffdf9e}
a{color:var(--ac)}h2{font-size:13px;text-transform:uppercase;letter-spacing:.7px;color:var(--mut);margin:0 0 12px}
</style></head><body>
<header><h1>ATU-1000</h1><span class="pill" id="disp">-</span><span class="pill" id="cat">-</span>
<span class="pill" id="mode">-</span><span class="pill" id="up">-</span></header>
<main>
<div class="card">
<div id="alert"></div>
<div class="freq"><span id="f">--.---</span><small id="band"></small></div>
<div class="grid" style="margin-top:14px">
<div><div class="k">SWR</div><div class="v" id="swr">--</div><div class="bar" id="sb"><i></i></div></div>
<div><div class="k">Power</div><div class="v" id="pw">--</div><div class="bar" id="pb"><i></i></div></div>
<div><div class="k">Network</div><div class="v" id="lc" style="font-size:16px">--</div></div>
<div><div class="k">Status</div><div class="v" id="st" style="font-size:16px">--</div></div>
</div></div>

<div class="card"><h2>Control</h2>
<div class="row">
<button class="p" onclick="cmd('tune')">Tune</button>
<button onclick="cmd('tune cat')">CAT Tune</button>
<button onclick="cmd('tune force')">Force Tune</button>
<button onclick="cmd('abort')">Abort</button>
<button onclick="cmd('bypass on')">Bypass On</button>
<button onclick="cmd('bypass off')">Bypass Off</button>
<button onclick="tog()">Toggle Auto</button>
<button onclick="cmd('power reset')">Reset Protection</button>
</div></div>

<div class="card"><h2>Console</h2>
<div class="row"><input type="text" id="ci" placeholder="type a command, e.g. status" onkeydown="if(event.key=='Enter')run()"><button onclick="run()">Run</button>
<button onclick="cmd('status')">Status</button><button onclick="cmd('config')">Config</button><button onclick="cmd('help')">Help</button></div>
<pre id="out">Ready.</pre></div>

<div class="card"><h2>Memory</h2>
<div class="row"><a href="/api/mem.csv" download="atu-memory.csv"><button>Download CSV</button></a>
<button onclick="cmd('mem list')">List</button><button onclick="if(confirm('Erase all stored tunes?'))cmd('mem clear')">Clear</button></div>
<div class="row" style="margin-top:10px"><input type="file" id="mf" accept=".csv,text/csv"><button onclick="upmem()">Upload CSV</button></div></div>

<div class="card"><h2>Firmware update</h2>
<form method="POST" action="/update" enctype="multipart/form-data" onsubmit="document.getElementById('uo').textContent='Uploading, do not power off...'">
<div class="row"><input type="file" name="firmware" accept=".bin" required><button class="p" type="submit">Flash</button></div></form>
<pre id="uo">Upload a firmware .bin built by PlatformIO (.pio/build/esp32s3/firmware.bin).</pre></div>
</main>
<script>
const $=i=>document.getElementById(i);
let auto=false;
function bar(el,p,cls){el.querySelector('i').style.width=Math.max(0,Math.min(100,p*100))+'%';el.className='bar'+(cls?' '+cls:'')}
async function poll(){try{const r=await fetch('/api/status');const d=await r.json();
$('f').textContent=d.freq>0?(d.freq/1e6).toFixed(3):'--.---';
$('band').textContent=d.band?(' '+d.band):'';
$('swr').textContent=d.valid?d.swr.toFixed(2):'--';
$('pw').textContent=d.pw.toFixed(0)+' W';
bar($('sb'),d.valid?(d.swr-1)/2:0,d.swr>2?'w':'');
bar($('pb'),d.pw/d.plim,d.ovl?'e':(d.warn?'w':''));
$('lc').textContent=d.byp?'BYPASS':(d.l.toFixed(2)+' uH / '+d.c+' pF '+(d.topo?'Hi-Z':'Lo-Z'));
$('st').textContent=d.tuning?('TUNING '+d.pct+'%'):d.status;
$('disp').textContent='Display: '+d.disp;$('cat').textContent='CAT: '+d.cat;
$('mode').textContent=d.auto?'AUTO':'MANUAL';auto=d.auto;
$('up').textContent=Math.floor(d.up/60)+'m  '+d.temp;
let a='';if(d.hold)a='<div class="al e">PROTECTION - TX inhibit asserted, RF still present, holding match</div>';
else if(d.ovl)a='<div class="al e">PROTECTION - TX inhibit, bypass engaged</div>';
else if(d.tlvl==3)a='<div class="al e">OVER TEMPERATURE - bypass engaged</div>';
else if(d.tlvl==2)a='<div class="al w">HOT - transmit inhibited</div>';
else if(d.swra)a='<div class="al w">High SWR</div>';
else if(d.warn)a='<div class="al w">Power warning</div>';
if(d.pend)a+='<div class="al w">Relay change held until RF drops</div>';
$('alert').innerHTML=a;}catch(e){}}
async function cmd(c){const r=await fetch('/api/cmd?c='+encodeURIComponent(c));$('out').textContent=await r.text();poll()}
function run(){const v=$('ci').value.trim();if(v){cmd(v);$('ci').value=''}}
function tog(){cmd(auto?'auto off':'auto on')}
async function upmem(){const f=$('mf').files[0];if(!f)return;const t=await f.text();
const r=await fetch('/api/mem.csv',{method:'POST',body:t});$('out').textContent=await r.text();}
poll();setInterval(poll,1000);
</script></body></html>)HTML";

class WebUi {
 public:
  void begin(Settings* settings) { cfg_ = settings; }

  // Starts connecting and returns straight away; loop() finishes the job.
  // The old version sat in a delay() loop for up to 15 s, during which nothing
  // else in the firmware ran.
  bool start(Print& log) {
    if (running_ || connecting_) return true;
    if (cfg_->wifiSsid[0] == '\0' && !cfg_->wifiApFallback) {
      log.println("No SSID configured; use 'wifi <ssid> <pass>'");
      return false;
    }

    WiFi.persistent(false);
    WiFi.setHostname(cfg_->hostname);

    if (cfg_->wifiSsid[0] != '\0') {
      WiFi.mode(WIFI_STA);
      WiFi.begin(cfg_->wifiSsid, cfg_->wifiPass);
      log.printf("Connecting to '%s' in the background...\n", cfg_->wifiSsid);
      connecting_ = true;
      connectStartMs_ = millis();
      return true;
    }
    startAp();
    return true;
  }

  void stop() {
    if (!running_ && !connecting_) return;
    if (running_) {
      server_.stop();
      MDNS.end();
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    running_ = false;
    connecting_ = false;
    apMode_ = false;
  }

  void loop() {
    if (connecting_) {
      if (WiFi.status() == WL_CONNECTED) {
        connecting_ = false;
        apMode_ = false;
        Serial.printf("Wi-Fi connected, IP %s\n", WiFi.localIP().toString().c_str());
        startServer();
      } else if (elapsed(connectStartMs_) >= kConnectTimeoutMs) {
        connecting_ = false;
        if (cfg_->wifiApFallback) {
          Serial.println("Wi-Fi connection failed, starting access point");
          startAp();
        } else {
          Serial.println("Wi-Fi connection failed");
          WiFi.mode(WIFI_OFF);
        }
      }
    }
    if (running_) server_.handleClient();
  }

  bool connecting() const { return connecting_; }

  bool running() const { return running_; }
  bool up() const {
    return running_ && (apMode_ || WiFi.status() == WL_CONNECTED);
  }
  String ip() const {
    if (!running_) return String("");
    return apMode_ ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  }
  bool apMode() const { return apMode_; }

 private:
  static constexpr uint32_t kConnectTimeoutMs = 15000;

  Settings* cfg_ = nullptr;
  WebServer server_{80};
  bool running_ = false;
  bool connecting_ = false;
  bool apMode_ = false;
  bool routesAdded_ = false;
  uint32_t connectStartMs_ = 0;

  void startAp() {
    WiFi.mode(WIFI_AP);
    char ssid[36];
    snprintf(ssid, sizeof(ssid), "%s-setup", cfg_->hostname);
    WiFi.softAP(ssid);
    apMode_ = true;
    Serial.printf("Access point '%s', IP %s\n", ssid, WiFi.softAPIP().toString().c_str());
    startServer();
  }

  void startServer() {
    if (MDNS.begin(cfg_->hostname)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Reachable at http://%s.local/\n", cfg_->hostname);
    }
    if (!routesAdded_) {
      routes();
      routesAdded_ = true;
    }
    server_.begin();
    running_ = true;
    Serial.printf("Web UI: http://%s/\n", ip().c_str());
  }

  void routes() {
    server_.on("/", HTTP_GET, [this]() {
      server_.sendHeader("Cache-Control", "no-store");
      server_.send_P(200, "text/html", kIndexHtml);
    });

    server_.on("/api/status", HTTP_GET, [this]() { sendStatus(); });

    server_.on("/api/cmd", HTTP_GET, [this]() {
      if (!server_.hasArg("c")) { server_.send(400, "text/plain", "missing c"); return; }
      StringPrint sp;
      handleCommand(server_.arg("c").c_str(), sp);
      server_.send(200, "text/plain", sp.str());
    });

    server_.on("/api/mem.csv", HTTP_GET, [this]() {
      StringPrint sp;
      gApp.memory.exportCsv(sp);
      server_.sendHeader("Content-Disposition", "attachment; filename=atu-memory.csv");
      server_.send(200, "text/csv", sp.str());
    });

    server_.on("/api/mem.csv", HTTP_POST, [this]() {
      String body = server_.arg("plain");
      int ok = 0, bad = 0, start = 0;
      while (start < static_cast<int>(body.length())) {
        int nl = body.indexOf('\n', start);
        String lineStr = (nl < 0) ? body.substring(start) : body.substring(start, nl);
        lineStr.trim();
        if (lineStr.length()) {
          if (gApp.memory.importCsvLine(lineStr.c_str())) ++ok; else ++bad;
        }
        if (nl < 0) break;
        start = nl + 1;
      }
      gApp.memory.flush();
      char msg[96];
      snprintf(msg, sizeof(msg), "Imported %d row(s), %d rejected. %u entries stored.\n",
               ok, bad, static_cast<unsigned>(gApp.memory.size()));
      server_.send(200, "text/plain", msg);
    });

    // OTA
    server_.on("/update", HTTP_POST,
               [this]() {
                 bool ok = !Update.hasError();
                 server_.sendHeader("Connection", "close");
                 server_.send(200, "text/html",
                              ok ? "<h2>Update OK - rebooting</h2><a href=\"/\">back</a>"
                                 : "<h2>Update FAILED</h2><a href=\"/\">back</a>");
                 if (ok) { delay(500); ESP.restart(); }
               },
               [this]() { handleUpload(); });

    server_.onNotFound([this]() { server_.send(404, "text/plain", "not found"); });
  }

  void handleUpload() {
    HTTPUpload& up = server_.upload();
    if (up.status == UPLOAD_FILE_START) {
      // Refuse to start a tune-critical operation while RF might be present.
      gApp.tuner.abort();
      gApp.sweep.stop();
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_WRITE) {
      if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_END) {
      if (!Update.end(true)) Update.printError(Serial);
    }
  }

  void sendStatus() {
    App& a = gApp;
    Settings& c = a.cfg();
    char buf[720];
    snprintf(buf, sizeof(buf),
             "{\"freq\":%lu,\"band\":\"%s\",\"swr\":%.2f,\"pw\":%.1f,\"valid\":%s,"
             "\"swra\":%s,\"hold\":%s,\"pend\":%s,\"cattune\":%s,"
             "\"l\":%.2f,\"c\":%u,\"topo\":%s,\"byp\":%s,\"auto\":%s,"
             "\"status\":\"%s\",\"cat\":\"%s\",\"disp\":\"%s\","
             "\"tuning\":%s,\"pct\":%u,\"sweep\":%s,"
             "\"warn\":%s,\"ovl\":%s,\"plim\":%.0f,"
             "\"temp\":\"%s\",\"tlvl\":%u,\"mem\":%u,\"up\":%lu,\"heap\":%lu}",
             static_cast<unsigned long>(a.currentFreqHz),
             bandName(a.currentFreqHz),
             a.reading.valid ? a.reading.swr : 0.0f,
             a.reading.powerW,
             a.reading.valid ? "true" : "false",
             a.swrAlarm ? "true" : "false",
             (a.protectState == ProtectState::WaitRfDrop) ? "true" : "false",
             a.relayPending ? "true" : "false",
             a.catTune.active() ? "true" : "false",
             a.relays.totalL(), static_cast<unsigned>(a.relays.totalC()),
             a.state.topology ? "true" : "false",
             a.state.bypass ? "true" : "false",
             c.autoTune ? "true" : "false",
             a.status.c_str(), a.cat.protocolName(), a.display.kindName(),
             a.tuner.running() ? "true" : "false",
             static_cast<unsigned>(a.tuner.percent()),
             a.sweep.isRunning() ? "true" : "false",
             a.powerProt.warning() ? "true" : "false",
             (a.powerProt.overload() || a.protectionLatched) ? "true" : "false",
             c.powerLimitW,
             tempString(), static_cast<unsigned>(a.thermal.level()),
             static_cast<unsigned>(a.memory.size()),
             static_cast<unsigned long>(elapsed(a.bootMs) / 1000UL),
             static_cast<unsigned long>(ESP.getFreeHeap()));
    server_.sendHeader("Cache-Control", "no-store");
    server_.send(200, "application/json", buf);
  }

  static const char* tempString() {
    static char t[12];
    if (!gApp.thermal.available()) { strcpy(t, ""); return t; }
    snprintf(t, sizeof(t), "%.0fC", gApp.thermal.tempC());
    return t;
  }
};

extern WebUi gWeb;

// Declared in Commands.h, defined here because it needs the Wi-Fi object.
inline bool cmdWifi(const char* args, Print& out) {
  App& a = gApp;
  Settings& c = a.cfg();
  while (*args == ' ') ++args;

  if (!*args || !strcasecmp(args, "status")) {
    out.printf("Wi-Fi: %s\n", gWeb.up() ? (gWeb.apMode() ? "access point" : "connected")
                                        : gWeb.connecting() ? "connecting"
                                        : (c.wifiEnabled ? "down" : "disabled"));
    out.printf("SSID    : %s\n", c.wifiSsid[0] ? c.wifiSsid : "<not set>");
    out.printf("Hostname: %s\n", c.hostname);
    if (gWeb.up()) {
      out.printf("Address : http://%s/\n", gWeb.ip().c_str());
      out.printf("mDNS    : http://%s.local/\n", c.hostname);
    }
    return true;
  }

  if (!strcasecmp(args, "on")) {
    c.wifiEnabled = true;
    a.settingsStore.save();
    if (!gWeb.start(out)) out.println("Failed to bring Wi-Fi up");
    else if (gWeb.up()) out.printf("Web UI at http://%s/\n", gWeb.ip().c_str());
    else out.println("The address is printed on the serial console once connected; see 'wifi status'.");
    return true;
  }

  if (!strcasecmp(args, "off")) {
    c.wifiEnabled = false;
    a.settingsStore.save();
    gWeb.stop();
    out.println("Wi-Fi off");
    return true;
  }

  // wifi <ssid> <passphrase>
  char ssid[33];
  const char* p = args;
  size_t n = 0;
  while (*p && *p != ' ' && n < sizeof(ssid) - 1) ssid[n++] = *p++;
  ssid[n] = '\0';
  while (*p == ' ') ++p;

  if (!n) {
    out.println("Usage: wifi status|on|off | wifi <ssid> <passphrase>");
    return true;
  }
  if (strlen(p) >= sizeof(c.wifiPass)) {
    out.println("Passphrase too long");
    return true;
  }

  strncpy(c.wifiSsid, ssid, sizeof(c.wifiSsid) - 1);
  c.wifiSsid[sizeof(c.wifiSsid) - 1] = '\0';
  strncpy(c.wifiPass, p, sizeof(c.wifiPass) - 1);
  c.wifiPass[sizeof(c.wifiPass) - 1] = '\0';
  a.settingsStore.save();
  out.printf("Credentials stored for '%s'. Run 'wifi on'.\n", c.wifiSsid);
  return true;
}

}  // namespace atu
