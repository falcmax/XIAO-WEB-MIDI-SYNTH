#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

extern "C" {
  #include "esp_http_server.h"
  #include "esp_https_server.h"
}

#include "SAM2695Synth.h"   // Seeed_Arduino_MIDIMaster
#include "key.h"            // <-- TLS_CERT_PEM / TLS_KEY_PEM

#define SHOW_SERIAL Serial
#define COM_SERIAL  Serial0
static const int MIDI_BAUD = 31250;

// -------- WiFi --------
static const char* WIFI_SSID = "YOUR_SSID";
static const char* WIFI_PASS = "YOUR_PASSWORD";

// mDNS: https://xiao-synth.local/
static const char* MDNS_HOST = "xiao-synth";


static SAM2695Synth<HardwareSerial>& synth = SAM2695Synth<HardwareSerial>::getInstance();

// -------- HTTPS server handle --------
static httpd_handle_t g_https = nullptr;

// -------- MIDI low-level helpers --------
static inline void midiWrite(uint8_t b) { COM_SERIAL.write(b); }

static void midiSend3(uint8_t st, uint8_t d1, uint8_t d2) {
  midiWrite(st);
  midiWrite(d1 & 0x7F);
  midiWrite(d2 & 0x7F);
}

static void midiSend2(uint8_t st, uint8_t d1) {
  midiWrite(st);
  midiWrite(d1 & 0x7F);
}

static void midiCC(uint8_t ch0, uint8_t cc, uint8_t val) { midiSend3(0xB0 | (ch0 & 0x0F), cc, val); }
static void midiPB(uint8_t ch0, uint8_t lsb, uint8_t msb) { midiSend3(0xE0 | (ch0 & 0x0F), lsb, msb); }
static void midiPC_raw(uint8_t ch0, uint8_t prog)         { midiSend2(0xC0 | (ch0 & 0x0F), prog); } // backup

static void midiAllNotesOff() {
  for (uint8_t ch=0; ch<16; ch++) {
    midiCC(ch, 123, 0); // All Notes Off
    midiCC(ch, 121, 0); // Reset All Controllers
    midiCC(ch, 120, 0); // All Sound Off
  }
}

// NRPN: CC99 (NRPN MSB), CC98 (NRPN LSB), CC6 (Data Entry MSB)
static void midiNRPN(uint8_t ch0, uint8_t nrpn_msb, uint8_t nrpn_lsb, uint8_t val) {
  midiCC(ch0, 99, nrpn_msb);
  midiCC(ch0, 98, nrpn_lsb);
  midiCC(ch0, 6,  val);
}

static uint8_t rolandChecksum(const uint8_t* addr_and_data, size_t len) {
  uint32_t sum = 0;
  for (size_t i=0; i<len; i++) sum += addr_and_data[i];
  return (uint8_t)((128 - (sum & 0x7F)) & 0x7F);
}

static void midiSysEx(const uint8_t* data, size_t len) {
  for (size_t i=0; i<len; i++) midiWrite(data[i]);
}

// Roland GS: F0 41 00 42 12 [addr1 addr2 addr3 data] [cs] F7
static void midiGSSysEx(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t v) {
  uint8_t payload[4] = { a1, a2, a3, (uint8_t)(v & 0x7F) };
  uint8_t cs = rolandChecksum(payload, sizeof(payload));
  uint8_t msg[11] = { 0xF0, 0x41, 0x00, 0x42, 0x12, a1, a2, a3, (uint8_t)(v & 0x7F), cs, 0xF7 };
  midiSysEx(msg, sizeof(msg));
}

static void midiGMReset() { const uint8_t msg[] = { 0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7 }; midiSysEx(msg, sizeof(msg)); }
static void midiGSReset() { const uint8_t msg[] = { 0xF0,0x41,0x00,0x42,0x12,0x40,0x00,0x7F,0x00,0x41,0xF7 }; midiSysEx(msg, sizeof(msg)); }

// FX mask NRPN 0x375F bits (da SAM2695 datasheet):
// bit0 EQ1, bit1 EQ2, bit2 MIC, bit3 OM, bit4 CHR, bit5 REV, bit6 ECH
static volatile uint8_t g_fxMask = 0;

// Defaults ragionevoli (rev+chorus on, EQ 4-band on)
static void applyDefaultFx() {
  g_fxMask = (1<<5) | (1<<4) | (1<<1) | (1<<0);  // REV + CHR + EQ(4b)
  midiNRPN(0, 0x37, 0x5F, g_fxMask);
  midiNRPN(0, 0x37, 0x18, 0x7F);                 // GM_POST ON

  // EQ defaults
  midiNRPN(0, 0x37, 0x00, 0x60);
  midiNRPN(0, 0x37, 0x01, 0x40);
  midiNRPN(0, 0x37, 0x02, 0x40);
  midiNRPN(0, 0x37, 0x03, 0x60);

  midiNRPN(0, 0x37, 0x08, 0x0C);
  midiNRPN(0, 0x37, 0x09, 0x1B);
  midiNRPN(0, 0x37, 0x0A, 0x72);
  midiNRPN(0, 0x37, 0x0B, 0x40);

  // Reverb global defaults (GS)
  midiGSSysEx(0x40,0x01,0x30,4);   // type Hall2
  midiGSSysEx(0x40,0x01,0x33,64);  // level
  midiGSSysEx(0x40,0x01,0x34,64);  // time
  midiGSSysEx(0x40,0x01,0x35,64);  // feedback (delay types)

  // Chorus global defaults (GS)
  midiGSSysEx(0x40,0x01,0x38,2);   // type Chorus3
  midiGSSysEx(0x40,0x01,0x3A,0);   // level
  midiGSSysEx(0x40,0x01,0x3B,64);  // feedback
  midiGSSysEx(0x40,0x01,0x3C,64);  // delay
  midiGSSysEx(0x40,0x01,0x3D,64);  // rate
  midiGSSysEx(0x40,0x01,0x3E,64);  // depth
}

// -------- Web UI (HTTPS) --------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>XIAO MIDI Synth</title>
<style>
  :root{--bg:#0b0f14;--card:#121826;--muted:#93a4b7;--fg:#e6edf3;--acc:#4ea1ff;--ok:#34d399;--bad:#fb7185;--line:#233047;}
  html,body{height:100%;margin:0;background:var(--bg);color:var(--fg);font:14px/1.35 system-ui,-apple-system,Segoe UI,Roboto,Arial;}
  .wrap{max-width:1120px;margin:0 auto;padding:18px;}
  h1{font-size:18px;margin:0 0 10px 0;font-weight:650;}
  .sub{color:var(--muted);margin:0 0 16px 0;}
  .grid{display:grid;grid-template-columns:repeat(12,1fr);gap:12px;}
  .card{grid-column:span 12;background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px 14px;box-shadow:0 10px 30px rgba(0,0,0,.25);}
  @media(min-width:900px){.c6{grid-column:span 6;}.c12{grid-column:span 12;}}
  .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;}
  label{color:var(--muted);font-size:12px;}
  select,input[type=number]{background:#0e1420;border:1px solid var(--line);color:var(--fg);border-radius:10px;padding:8px 10px;outline:none;}
  input[type=range]{width:240px;}
  button{background:#0e1420;border:1px solid var(--line);color:var(--fg);border-radius:12px;padding:9px 12px;cursor:pointer;}
  button:hover{border-color:#3b4b6b;}
  button.primary{background:rgba(78,161,255,.15);border-color:rgba(78,161,255,.35);}
  button.danger{background:rgba(251,113,133,.12);border-color:rgba(251,113,133,.35);}
  .pill{display:inline-block;padding:4px 8px;border-radius:999px;border:1px solid var(--line);color:var(--muted);font-size:12px;}
  .ok{color:var(--ok)} .bad{color:var(--bad)} .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;}
  #lastEvt{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:560px;}
  .section-title{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px}
  .section-title h2{font-size:13px;margin:0;font-weight:650;}
  .section-title span{font-size:12px;color:var(--muted);}
  .sliderline{display:flex;align-items:center;gap:10px;flex-wrap:wrap;}
  .sliderline .val{min-width:56px;text-align:right;color:var(--muted);}
  .two{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
  .note{font-size:12px;color:var(--muted);margin:8px 0 0 0;}
</style>
</head>
<body>
<div class="wrap">
  <h1>XIAO MIDI Synth - WebMIDI Keyboard Bridge</h1>
  <p class="sub">HTTPS + WSS. Attiva WebMIDI e seleziona la tua tastiera Korg.</p>

  <div class="grid">
    <div class="card c6">
      <div class="section-title"><h2>WebMIDI</h2><span class="mono" id="secureInfo"></span></div>
      <div class="row">
        <button class="primary" id="btnEnable">Enable WebMIDI</button>
        <label>Input</label>
        <select id="midiIn"><option value="">(none)</option></select>
        <label>Channel</label>
        <select id="chMode">
          <option value="follow">Follow keyboard</option>
          <option value="fixed">Force UI channel</option>
        </select>
        <select id="uiCh"></select>
      </div>
      <div class="row" style="margin-top:8px;">
        <span class="pill" id="midiStatus">MIDI: <span class="bad">disabled</span></span>
        <span class="pill" id="wsStatus">WSS: <span class="bad">disconnected</span></span>
        <span class="pill mono" id="lastEvt">Last: -</span>
      </div>
    </div>

    <div class="card c6">
      <div class="section-title"><h2>Synth + Preset</h2><span>Immediate</span></div>

      <div class="row">
        <button class="danger" id="btnPanic">Panic</button>

        <label>Instrument (GM)</label>
        <select id="instSel"></select>

        <label>Program</label>
        <input id="prog" type="number" min="0" max="127" value="0" style="width:88px">
        <button id="btnProg">Set</button>
      </div>

      <div class="row" style="margin-top:10px;">
        <label>FX Preset</label>
        <select id="presetSel"></select>
        <button class="primary" id="btnPreset">Apply preset</button>
      </div>

      <div class="sliderline" style="margin-top:10px;">
        <label style="min-width:140px">Channel Volume (CC7)</label>
        <input id="vol" type="range" min="0" max="127" value="100">
        <span class="val mono" id="volVal">100</span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Pan (CC10)</label>
        <input id="pan" type="range" min="0" max="127" value="64">
        <span class="val mono" id="panVal">64</span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Reverb Send (CC91)</label>
        <input id="revSend" type="range" min="0" max="127" value="40">
        <span class="val mono" id="revSendVal">40</span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Chorus Send (CC93)</label>
        <input id="choSend" type="range" min="0" max="127" value="0">
        <span class="val mono" id="choSendVal">0</span>
      </div>
    </div>

    <div class="card c6">
      <div class="section-title"><h2>Reverb (Global)</h2><span>GS SysEx</span></div>
      <div class="row">
        <label>Enable</label><input type="checkbox" id="revOn" checked>
        <label>Type</label>
        <select id="revType">
          <option value="0">Room1</option><option value="1">Room2</option><option value="2">Room3</option>
          <option value="3">Hall1</option><option value="4" selected>Hall2</option><option value="5">Plate</option>
          <option value="6">Delay</option><option value="7">Pan Delay</option>
        </select>
      </div>
      <div class="sliderline">
        <label style="min-width:160px">Master Level</label>
        <input id="revLvl" type="range" min="0" max="127" value="64">
        <span class="val mono" id="revLvlVal">64</span>
      </div>
      <div class="sliderline">
        <label style="min-width:160px">Time</label>
        <input id="revTime" type="range" min="0" max="127" value="64">
        <span class="val mono" id="revTimeVal">64</span>
      </div>
      <div class="sliderline">
        <label style="min-width:160px">Delay Feedback</label>
        <input id="revFb" type="range" min="0" max="127" value="64">
        <span class="val mono" id="revFbVal">64</span>
      </div>
    </div>

    <div class="card c6">
      <div class="section-title"><h2>Chorus (Global)</h2><span>GS SysEx</span></div>
      <div class="row">
        <label>Enable</label><input type="checkbox" id="choOn" checked>
        <label>Type</label>
        <select id="choType">
          <option value="0">Chorus1</option><option value="1">Chorus2</option><option value="2" selected>Chorus3</option>
          <option value="3">Chorus4</option><option value="4">Feedback</option><option value="5">Flanger</option>
          <option value="6">Short Delay</option><option value="7">FB Delay</option>
        </select>
      </div>
      <div class="sliderline">
        <label style="min-width:160px">Master Level</label>
        <input id="choLvl" type="range" min="0" max="127" value="0">
        <span class="val mono" id="choLvlVal">0</span>
      </div>
      <div class="two">
        <div class="sliderline">
          <label style="min-width:160px">Feedback</label>
          <input id="choFb" type="range" min="0" max="127" value="64">
          <span class="val mono" id="choFbVal">64</span>
        </div>
        <div class="sliderline">
          <label style="min-width:160px">Delay</label>
          <input id="choDelay" type="range" min="0" max="127" value="64">
          <span class="val mono" id="choDelayVal">64</span>
        </div>
      </div>
      <div class="two">
        <div class="sliderline">
          <label style="min-width:160px">Rate</label>
          <input id="choRate" type="range" min="0" max="127" value="64">
          <span class="val mono" id="choRateVal">64</span>
        </div>
        <div class="sliderline">
          <label style="min-width:160px">Depth</label>
          <input id="choDepth" type="range" min="0" max="127" value="64">
          <span class="val mono" id="choDepthVal">64</span>
        </div>
      </div>
    </div>

    <div class="card c12">
      <div class="section-title"><h2>Equalizer (Post FX)</h2><span>NRPN 37xx</span></div>
      <div class="row">
        <label>Mode</label>
        <select id="eqMode">
          <option value="off">Off</option>
          <option value="2b">2-Band</option>
          <option value="4b" selected>4-Band</option>
        </select>
        <button id="btnEqFlat">Flat</button>
        <button id="btnEqDefault">Defaults</button>
      </div>

      <div class="sliderline">
        <label style="min-width:140px">Bass (Low)</label>
        <input id="eqLB" type="range" min="0" max="127" value="96">
        <span class="val mono" id="eqLBVal">96</span>
        <span class="val" id="eqLBdB"></span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Mid-Low</label>
        <input id="eqMLB" type="range" min="0" max="127" value="64">
        <span class="val mono" id="eqMLBVal">64</span>
        <span class="val" id="eqMLBdB"></span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Mid-High</label>
        <input id="eqMHB" type="range" min="0" max="127" value="64">
        <span class="val mono" id="eqMHBVal">64</span>
        <span class="val" id="eqMHBdB"></span>
      </div>
      <div class="sliderline">
        <label style="min-width:140px">Treble (High)</label>
        <input id="eqHB" type="range" min="0" max="127" value="96">
        <span class="val mono" id="eqHBVal">96</span>
        <span class="val" id="eqHBdB"></span>
      </div>

      <details style="margin-top:8px;">
        <summary style="color:var(--muted);cursor:pointer;">Band cutoff frequencies (advanced)</summary>
        <div class="two" style="margin-top:8px;">
          <div class="sliderline">
            <label style="min-width:140px">Low cutoff</label>
            <input id="eqfLB" type="range" min="0" max="127" value="12">
            <span class="val mono" id="eqfLBVal">12</span>
          </div>
          <div class="sliderline">
            <label style="min-width:140px">Mid-Low cutoff</label>
            <input id="eqfMLB" type="range" min="0" max="127" value="27">
            <span class="val mono" id="eqfMLBVal">27</span>
          </div>
          <div class="sliderline">
            <label style="min-width:140px">Mid-High cutoff</label>
            <input id="eqfMHB" type="range" min="0" max="127" value="114">
            <span class="val mono" id="eqfMHBVal">114</span>
          </div>
          <div class="sliderline">
            <label style="min-width:140px">High cutoff</label>
            <input id="eqfHB" type="range" min="0" max="127" value="64">
            <span class="val mono" id="eqfHBVal">64</span>
          </div>
        </div>
      </details>
    </div>

  </div>
</div>

<script>
(() => {
  const $ = (id)=>document.getElementById(id);

  $("secureInfo").textContent = "origin=" + location.origin + " secure=" + (window.isSecureContext ? "true":"false");

  // Channels 1..16
  for(let i=1;i<=16;i++){
    const o=document.createElement("option");
    o.value=String(i); o.textContent=String(i);
    if(i===1) o.selected=true;
    $("uiCh").appendChild(o);
  }

  // GM instrument names (128)
  const GM = [
    "Acoustic Grand Piano","Bright Acoustic Piano","Electric Grand Piano","Honky-tonk Piano","Electric Piano 1","Electric Piano 2","Harpsichord","Clavi",
    "Celesta","Glockenspiel","Music Box","Vibraphone","Marimba","Xylophone","Tubular Bells","Dulcimer",
    "Drawbar Organ","Percussive Organ","Rock Organ","Church Organ","Reed Organ","Accordion","Harmonica","Tango Accordion",
    "Acoustic Guitar (nylon)","Acoustic Guitar (steel)","Electric Guitar (jazz)","Electric Guitar (clean)","Electric Guitar (muted)","Overdriven Guitar","Distortion Guitar","Guitar Harmonics",
    "Acoustic Bass","Electric Bass (finger)","Electric Bass (pick)","Fretless Bass","Slap Bass 1","Slap Bass 2","Synth Bass 1","Synth Bass 2",
    "Violin","Viola","Cello","Contrabass","Tremolo Strings","Pizzicato Strings","Orchestral Harp","Timpani",
    "String Ensemble 1","String Ensemble 2","SynthStrings 1","SynthStrings 2","Choir Aahs","Voice Oohs","Synth Voice","Orchestra Hit",
    "Trumpet","Trombone","Tuba","Muted Trumpet","French Horn","Brass Section","SynthBrass 1","SynthBrass 2",
    "Soprano Sax","Alto Sax","Tenor Sax","Baritone Sax","Oboe","English Horn","Bassoon","Clarinet",
    "Piccolo","Flute","Recorder","Pan Flute","Blown Bottle","Shakuhachi","Whistle","Ocarina",
    "Lead 1 (square)","Lead 2 (sawtooth)","Lead 3 (calliope)","Lead 4 (chiff)","Lead 5 (charang)","Lead 6 (voice)","Lead 7 (fifths)","Lead 8 (bass + lead)",
    "Pad 1 (new age)","Pad 2 (warm)","Pad 3 (polysynth)","Pad 4 (choir)","Pad 5 (bowed)","Pad 6 (metallic)","Pad 7 (halo)","Pad 8 (sweep)",
    "FX 1 (rain)","FX 2 (soundtrack)","FX 3 (crystal)","FX 4 (atmosphere)","FX 5 (brightness)","FX 6 (goblins)","FX 7 (echoes)","FX 8 (sci-fi)",
    "Sitar","Banjo","Shamisen","Koto","Kalimba","Bag pipe","Fiddle","Shanai",
    "Tinkle Bell","Agogo","Steel Drums","Woodblock","Taiko Drum","Melodic Tom","Synth Drum","Reverse Cymbal",
    "Guitar Fret Noise","Breath Noise","Seashore","Bird Tweet","Telephone Ring","Helicopter","Applause","Gunshot"
  ];

  // Populate instrument select
  for(let i=0;i<128;i++){
    const o=document.createElement("option");
    o.value=String(i);
    o.textContent = String(i).padStart(3,"0") + " - " + GM[i];
    $("instSel").appendChild(o);
  }

  // WebSocket
  let ws=null;
  function wsConnect(){
    const url = (location.protocol==="https:" ? "wss://" : "ws://") + location.host + "/ws";
    ws = new WebSocket(url);
    ws.onopen = ()=>{ $("wsStatus").innerHTML = 'WSS: <span class="ok">connected</span>'; };
    ws.onclose = ()=>{ $("wsStatus").innerHTML = 'WSS: <span class="bad">disconnected</span>'; setTimeout(wsConnect, 1200); };
  }
  wsConnect();

  function sendLine(s){
    if(!ws || ws.readyState!==1) return;
    ws.send(s);
  }

  function uiChan0(){ return Number($("uiCh").value)-1; } // 0..15
  function clamp(v,a,b){ v = Number(v)||0; return Math.max(a, Math.min(b, v)); }

  // Debounce for sliders
  function debounce(fn, ms){
    let t=null, lastArgs=null;
    return (...args)=>{
      lastArgs=args;
      if(t) clearTimeout(t);
      t=setTimeout(()=>{ t=null; fn(...lastArgs); }, ms);
    };
  }

  function setLast(s){ $("lastEvt").textContent = "Last: " + s; }

  // WebMIDI
  let midiAccess=null;
  let currentIn=null;

  function refreshMidiInputs(){
    const sel = $("midiIn");
    sel.innerHTML = '<option value="">(select)</option>';
    if(!midiAccess) return;
    midiAccess.inputs.forEach((inp)=>{
      const opt=document.createElement("option");
      opt.value=inp.id;
      opt.textContent=inp.name || inp.manufacturer || inp.id;
      sel.appendChild(opt);
    });
  }

  function setMidiStatus(ok, msg){
    $("midiStatus").innerHTML = 'MIDI: ' + (ok ? '<span class="ok">enabled</span>' : '<span class="bad">disabled</span>') +
      (msg?(' <span style="color:var(--muted)">('+msg+')</span>'):'');
  }

  $("btnEnable").addEventListener("click", async ()=>{
    if(!navigator.requestMIDIAccess){
      setMidiStatus(false, "WebMIDI not supported");
      return;
    }
    try{
      midiAccess = await navigator.requestMIDIAccess({sysex:false});
      setMidiStatus(true, "OK");
      refreshMidiInputs();
      midiAccess.onstatechange = refreshMidiInputs;
    }catch(e){
      setMidiStatus(false, e && e.message ? e.message : "requestMIDIAccess failed");
    }
  });

  $("midiIn").addEventListener("change", ()=>{
    const id = $("midiIn").value;
    if(!midiAccess) return;
    if(currentIn) currentIn.onmidimessage=null;
    currentIn = midiAccess.inputs.get(id) || null;
    if(!currentIn) return;

    currentIn.onmidimessage = (ev)=>{
      const d = ev.data;
      if(!d || d.length<1) return;

      const st = d[0] & 0xF0;
      const ch = (d[0] & 0x0F) + 1; // 1..16
      const mode = $("chMode").value;
      const outCh = (mode==="fixed") ? Number($("uiCh").value) : ch;

      const data1 = d.length>1 ? d[1] : 0;
      const data2 = d.length>2 ? d[2] : 0;

      if(st===0x90 && data2>0){
        setLast(`NoteOn ch=${outCh} note=${data1} vel=${data2}`);
        sendLine(`note_on,${outCh-1},${data1},${data2}`);
      }else if(st===0x80 || (st===0x90 && data2===0)){
        setLast(`NoteOff ch=${outCh} note=${data1} vel=${data2}`);
        sendLine(`note_off,${outCh-1},${data1},${data2}`);
      }else if(st===0xB0){
        setLast(`CC ch=${outCh} cc=${data1} val=${data2}`);
        sendLine(`cc,${outCh-1},${data1},${data2}`);
      }else if(st===0xC0){
        setLast(`PC ch=${outCh} prog=${data1}`);
        sendLine(`pc,${outCh-1},${data1}`);
      }else if(st===0xE0){
        setLast(`PB ch=${outCh}`);
        sendLine(`pb,${outCh-1},${data1},${data2}`);
      }
    };
  });

  // Buttons + instrument/program
  $("btnPanic").addEventListener("click", ()=>{ sendLine("panic"); setLast("Panic"); });

  $("btnProg").addEventListener("click", ()=>{
    const p = clamp($("prog").value,0,127);
    sendLine(`pc,${uiChan0()},${p}`);
    setLast(`Set Program ${p} (ch ${uiChan0()+1})`);
  });

  $("instSel").addEventListener("change", ()=>{
    const p = Number($("instSel").value) || 0;
    $("prog").value = String(p);
    sendLine(`pc,${uiChan0()},${p}`);
    setLast(`Instrument: ${String(p).padStart(3,"0")} ${GM[p]} (ch ${uiChan0()+1})`);
  });

  // Sliders -> CC immediate
  function bindRange(id, valId, fn){
    const el=$(id), out=$(valId);
    const send = debounce((v)=>fn(v), 70);
    const upd=()=>{
      const v=clamp(el.value,0,127);
      out.textContent=String(v);
      send(v);
    };
    el.addEventListener("input", upd);
    upd();
  }

  bindRange("vol","volVal",      (v)=>sendLine(`cc,${uiChan0()},7,${v}`));
  bindRange("pan","panVal",      (v)=>sendLine(`cc,${uiChan0()},10,${v}`));
  bindRange("revSend","revSendVal",(v)=>sendLine(`cc,${uiChan0()},91,${v}`));
  bindRange("choSend","choSendVal",(v)=>sendLine(`cc,${uiChan0()},93,${v}`));

  // GS + NRPN helpers
  function gs(a1,a2,a3,v){ sendLine(`gs,${a1},${a2},${a3},${clamp(v,0,127)}`); }
  function nrpn(lsb, v){ sendLine(`nrpn,${lsb},${clamp(v,0,127)}`); }

  // FX mask (NRPN 0x5F)
  let fxMask = (1<<5)|(1<<4)|(1<<1)|(1<<0); // default
  function setFxMask(m){ fxMask = m & 0x7F; nrpn(95, fxMask); }

  function setFxBit(bit, on){
    if(on) fxMask |= (1<<bit);
    else fxMask &= ~(1<<bit);
    setFxMask(fxMask);
  }

  function setEqMode(mode){
    fxMask &= ~((1<<0)|(1<<1));  // clear EQ bits
    if(mode==="2b") fxMask |= (1<<1);
    if(mode==="4b") fxMask |= (1<<1)|(1<<0);
    setFxMask(fxMask);
  }

  // Reverb global
  $("revOn").addEventListener("change", ()=>setFxBit(5, $("revOn").checked));
  $("revType").addEventListener("change", ()=>gs(64,1,48, Number($("revType").value)));
  bindRange("revLvl","revLvlVal", (v)=>gs(64,1,51, v));
  bindRange("revTime","revTimeVal",(v)=>gs(64,1,52, v));
  bindRange("revFb","revFbVal",   (v)=>gs(64,1,53, v));

  // Chorus global
  $("choOn").addEventListener("change", ()=>setFxBit(4, $("choOn").checked));
  $("choType").addEventListener("change", ()=>gs(64,1,56, Number($("choType").value)));
  bindRange("choLvl","choLvlVal",     (v)=>gs(64,1,58, v));
  bindRange("choFb","choFbVal",       (v)=>gs(64,1,59, v));
  bindRange("choDelay","choDelayVal", (v)=>gs(64,1,60, v));
  bindRange("choRate","choRateVal",   (v)=>gs(64,1,61, v));
  bindRange("choDepth","choDepthVal", (v)=>gs(64,1,62, v));

  // EQ helpers + dB label
  function dB(v){
    const x=clamp(v,0,127);
    const db = (x-64) * (12/63);
    return (db>=0?"+":"") + db.toFixed(1) + "dB";
  }
  function bindEq(id,valId,dbId,lsb){
    const el=$(id), out=$(valId), dbEl=$(dbId);
    const send = debounce((v)=>nrpn(lsb,v), 70);
    const upd=()=>{
      const v=clamp(el.value,0,127);
      out.textContent=String(v);
      dbEl.textContent=dB(v);
      send(v);
    };
    el.addEventListener("input", upd); upd();
  }
  bindEq("eqLB","eqLBVal","eqLBdB", 0);
  bindEq("eqMLB","eqMLBVal","eqMLBdB", 1);
  bindEq("eqMHB","eqMHBVal","eqMHBdB", 2);
  bindEq("eqHB","eqHBVal","eqHBdB", 3);

  function bindEqF(id,valId,lsb){
    const el=$(id), out=$(valId);
    const send = debounce((v)=>nrpn(lsb,v), 70);
    const upd=()=>{
      const v=clamp(el.value,0,127);
      out.textContent=String(v);
      send(v);
    };
    el.addEventListener("input", upd); upd();
  }
  bindEqF("eqfLB","eqfLBVal", 8);
  bindEqF("eqfMLB","eqfMLBVal", 9);
  bindEqF("eqfMHB","eqfMHBVal", 10);
  bindEqF("eqfHB","eqfHBVal", 11);

  $("eqMode").addEventListener("change", ()=>setEqMode($("eqMode").value));
  $("btnEqFlat").addEventListener("click", ()=>{
    $("eqLB").value=64; $("eqMLB").value=64; $("eqMHB").value=64; $("eqHB").value=64;
    ["eqLB","eqMLB","eqMHB","eqHB"].forEach(id=>$(id).dispatchEvent(new Event("input")));
    setLast("EQ Flat");
  });
  $("btnEqDefault").addEventListener("click", ()=>{
    $("eqLB").value=96; $("eqMLB").value=64; $("eqMHB").value=64; $("eqHB").value=96;
    ["eqLB","eqMLB","eqMHB","eqHB"].forEach(id=>$(id).dispatchEvent(new Event("input")));
    $("eqfLB").value=12; $("eqfMLB").value=27; $("eqfMHB").value=114; $("eqfHB").value=64;
    ["eqfLB","eqfMLB","eqfMHB","eqfHB"].forEach(id=>$(id).dispatchEvent(new Event("input")));
    setLast("EQ Defaults");
  });

  // ---- FX Presets (apply in one shot) ----
  const presets = [
    { name:"Dry (No FX)", eq:"off", revOn:false, choOn:false, revType:4, revLvl:0, revTime:64, revFb:64, choType:2, choLvl:0, choFb:64, choDelay:64, choRate:64, choDepth:64, revSend:0, choSend:0 },
    { name:"Hall (Smooth)", eq:"4b", revOn:true, choOn:true, revType:4, revLvl:72, revTime:80, revFb:64, choType:2, choLvl:24, choFb:64, choDelay:64, choRate:64, choDepth:72, revSend:55, choSend:18 },
    { name:"Plate (Vocal)", eq:"4b", revOn:true, choOn:false, revType:5, revLvl:78, revTime:70, revFb:64, choType:2, choLvl:0, choFb:64, choDelay:64, choRate:64, choDepth:64, revSend:60, choSend:0 },
    { name:"Delay (Echo)", eq:"2b", revOn:true, choOn:false, revType:6, revLvl:70, revTime:64, revFb:92, choType:6, choLvl:0, choFb:64, choDelay:64, choRate:64, choDepth:64, revSend:58, choSend:0 },
    { name:"Wide Chorus", eq:"4b", revOn:true, choOn:true, revType:3, revLvl:55, revTime:60, revFb:64, choType:5, choLvl:46, choFb:70, choDelay:60, choRate:70, choDepth:86, revSend:28, choSend:40 },
    { name:"Lo-Fi Radio", eq:"4b", revOn:false, choOn:false, revType:4, revLvl:0, revTime:64, revFb:64, choType:2, choLvl:0, choFb:64, choDelay:64, choRate:64, choDepth:64, revSend:0, choSend:0,
      eqG:[48,64,64,48], eqF:[18,32,100,64] }
  ];

  // Populate presets
  presets.forEach((p,i)=>{
    const o=document.createElement("option");
    o.value=String(i);
    o.textContent=p.name;
    $("presetSel").appendChild(o);
  });

  function applyPreset(p){
    // Update UI values (without relying on CSS growth)
    $("eqMode").value = p.eq;
    setEqMode(p.eq);

    $("revOn").checked = !!p.revOn; setFxBit(5, $("revOn").checked);
    $("choOn").checked = !!p.choOn; setFxBit(4, $("choOn").checked);

    $("revType").value = String(p.revType); gs(64,1,48,p.revType);
    $("revLvl").value = String(p.revLvl);  $("revLvl").dispatchEvent(new Event("input"));
    $("revTime").value = String(p.revTime);$("revTime").dispatchEvent(new Event("input"));
    $("revFb").value = String(p.revFb);    $("revFb").dispatchEvent(new Event("input"));

    $("choType").value = String(p.choType); gs(64,1,56,p.choType);
    $("choLvl").value = String(p.choLvl);   $("choLvl").dispatchEvent(new Event("input"));
    $("choFb").value = String(p.choFb);     $("choFb").dispatchEvent(new Event("input"));
    $("choDelay").value = String(p.choDelay);$("choDelay").dispatchEvent(new Event("input"));
    $("choRate").value = String(p.choRate); $("choRate").dispatchEvent(new Event("input"));
    $("choDepth").value = String(p.choDepth);$("choDepth").dispatchEvent(new Event("input"));

    $("revSend").value = String(p.revSend); $("revSend").dispatchEvent(new Event("input"));
    $("choSend").value = String(p.choSend); $("choSend").dispatchEvent(new Event("input"));

    if(p.eqG){
      $("eqLB").value=String(p.eqG[0]); $("eqLB").dispatchEvent(new Event("input"));
      $("eqMLB").value=String(p.eqG[1]); $("eqMLB").dispatchEvent(new Event("input"));
      $("eqMHB").value=String(p.eqG[2]); $("eqMHB").dispatchEvent(new Event("input"));
      $("eqHB").value=String(p.eqG[3]); $("eqHB").dispatchEvent(new Event("input"));
    }
    if(p.eqF){
      $("eqfLB").value=String(p.eqF[0]); $("eqfLB").dispatchEvent(new Event("input"));
      $("eqfMLB").value=String(p.eqF[1]); $("eqfMLB").dispatchEvent(new Event("input"));
      $("eqfMHB").value=String(p.eqF[2]); $("eqfMHB").dispatchEvent(new Event("input"));
      $("eqfHB").value=String(p.eqF[3]); $("eqfHB").dispatchEvent(new Event("input"));
    }

    setLast("Preset: " + p.name);
  }

  $("btnPreset").addEventListener("click", ()=>{
    const idx = Number($("presetSel").value)||0;
    applyPreset(presets[idx]);
  });

  // Init: default instrument list selection and default preset = Hall
  $("instSel").value = "0";
  $("presetSel").value = "1";

})();
</script>
</body>
</html>
)HTML";

// -------- HTTPS handlers --------
static esp_err_t handle_root(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static void ws_send_text(httpd_req_t* req, const char* txt) {
  httpd_ws_frame_t out;
  memset(&out, 0, sizeof(out));
  out.type = HTTPD_WS_TYPE_TEXT;
  out.payload = (uint8_t*)txt;
  out.len = strlen(txt);
  httpd_ws_send_frame(req, &out);
}

static bool parse_int(const char* s, int& out) {
  if (!s || !*s) return false;
  char* endp = nullptr;
  long v = strtol(s, &endp, 10);
  if (endp == s) return false;
  out = (int)v;
  return true;
}

static void handle_ws_command(char* msg, httpd_req_t* req) {
  if (!msg || !*msg) return;

  char* tok[8] = {0};
  int n = 0;
  char* saveptr = nullptr;
  for (char* p = strtok_r(msg, ",", &saveptr); p && n < 8; p = strtok_r(nullptr, ",", &saveptr)) {
    tok[n++] = p;
  }
  if (n <= 0) return;

  const char* cmd = tok[0];

  if (!strcmp(cmd, "panic")) {
    midiAllNotesOff();
    SHOW_SERIAL.println("WS PANIC");
    ws_send_text(req, "ok");
    return;
  }

  if (!strcmp(cmd, "note_on") && n >= 4) {
    int ch0, note, vel;
    if (parse_int(tok[1], ch0) && parse_int(tok[2], note) && parse_int(tok[3], vel)) {
      synth.setNoteOn((uint8_t)ch0, (uint8_t)note, (uint8_t)vel);
      SHOW_SERIAL.printf("WS NoteOn  ch=%d note=%d vel=%d\n", ch0+1, note, vel);
    }
    return;
  }

  if (!strcmp(cmd, "note_off") && n >= 4) {
    int ch0, note, vel;
    if (parse_int(tok[1], ch0) && parse_int(tok[2], note) && parse_int(tok[3], vel)) {
      synth.setNoteOff((uint8_t)ch0, (uint8_t)note);
      SHOW_SERIAL.printf("WS NoteOff ch=%d note=%d vel=%d\n", ch0+1, note, vel);
    }
    return;
  }

  if (!strcmp(cmd, "cc") && n >= 4) {
    int ch0, cc, val;
    if (parse_int(tok[1], ch0) && parse_int(tok[2], cc) && parse_int(tok[3], val)) {
      midiCC((uint8_t)ch0, (uint8_t)cc, (uint8_t)val);
    }
    return;
  }

  if (!strcmp(cmd, "pc") && n >= 3) {
    int ch0, prog;
    if (parse_int(tok[1], ch0) && parse_int(tok[2], prog)) {
      // bank 0, program 0..127
      synth.setInstrument(0, (uint8_t)ch0, (uint8_t)prog);
      midiPC_raw((uint8_t)ch0, (uint8_t)prog); // fallback, harmless
      SHOW_SERIAL.printf("WS PC ch=%d prog=%d\n", ch0+1, prog);
    }
    return;
  }

  if (!strcmp(cmd, "pb") && n >= 4) {
    int ch0, lsb, msb;
    if (parse_int(tok[1], ch0) && parse_int(tok[2], lsb) && parse_int(tok[3], msb)) {
      midiPB((uint8_t)ch0, (uint8_t)lsb, (uint8_t)msb);
    }
    return;
  }

  if (!strcmp(cmd, "nrpn")) {
    // nrpn,<lsb>,<val>   (msb=0x37, ch=0)
    if (n == 3) {
      int lsb, val;
      if (parse_int(tok[1], lsb) && parse_int(tok[2], val)) {
        midiNRPN(0, 0x37, (uint8_t)lsb, (uint8_t)val);
        if ((uint8_t)lsb == 0x5F) g_fxMask = (uint8_t)val;
      }
      return;
    }
  }

  if (!strcmp(cmd, "gs") && n >= 5) {
    int a1, a2, a3, val;
    if (parse_int(tok[1], a1) && parse_int(tok[2], a2) && parse_int(tok[3], a3) && parse_int(tok[4], val)) {
      midiGSSysEx((uint8_t)a1, (uint8_t)a2, (uint8_t)a3, (uint8_t)val);
    }
    return;
  }

  SHOW_SERIAL.print("WS unknown: ");
  SHOW_SERIAL.println(cmd);
}

static esp_err_t handle_ws(httpd_req_t* req) {
  if (req->method == HTTP_GET) {
    SHOW_SERIAL.println("WSS open");
    return ESP_OK;
  }

  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(ws_pkt));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) return ret;
  if (ws_pkt.len == 0 || ws_pkt.len > 2048) return ESP_OK;

  char* buf = (char*)calloc(1, ws_pkt.len + 1);
  if (!buf) return ESP_ERR_NO_MEM;

  ws_pkt.payload = (uint8_t*)buf;
  ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
  if (ret == ESP_OK) {
    buf[ws_pkt.len] = 0;
    handle_ws_command(buf, req);
  }
  free(buf);
  return ret;
}

static bool start_https_server() {
  httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
  conf.httpd.uri_match_fn = httpd_uri_match_wildcard;
  conf.httpd.max_uri_handlers = 8;

  conf.servercert     = (const uint8_t*)TLS_CERT_PEM;
  conf.servercert_len = strlen(TLS_CERT_PEM) + 1;
  conf.prvtkey_pem    = (const uint8_t*)TLS_KEY_PEM;
  conf.prvtkey_len    = strlen(TLS_KEY_PEM) + 1;

  esp_err_t ret = httpd_ssl_start(&g_https, &conf);
  if (ret != ESP_OK) {
    SHOW_SERIAL.printf("httpd_ssl_start failed: %d\n", (int)ret);
    return false;
  }

  httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=handle_root, .user_ctx=nullptr };
  httpd_register_uri_handler(g_https, &root);

  httpd_uri_t ws = { .uri="/ws", .method=HTTP_GET, .handler=handle_ws, .user_ctx=nullptr, .is_websocket=true };
  httpd_register_uri_handler(g_https, &ws);

  httpd_uri_t ws_post = ws;
  ws_post.method = HTTP_POST;
  httpd_register_uri_handler(g_https, &ws_post);

  SHOW_SERIAL.println("HTTPS/WSS server started on port 443");
  return true;
}

void setup() {
  SHOW_SERIAL.begin(115200);
  delay(200);

  // Init SAM2695 UART (via library)
  synth.begin(COM_SERIAL, MIDI_BAUD);
  delay(40);

  midiGMReset(); delay(40);
  midiGSReset(); delay(40);

  midiAllNotesOff();

  // Basic channel defaults
  for (int ch=0; ch<16; ch++) {
    synth.setInstrument(0, (uint8_t)ch, 0);
    midiCC((uint8_t)ch, 7, 100);
    midiCC((uint8_t)ch, 10, 64);
    midiCC((uint8_t)ch, 91, 40);
    midiCC((uint8_t)ch, 93, 0);
  }

  applyDefaultFx();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOST);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  SHOW_SERIAL.print("WiFi connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) {
    delay(250);
    SHOW_SERIAL.print(".");
  }
  SHOW_SERIAL.println();

  if (WiFi.status() == WL_CONNECTED) {
    SHOW_SERIAL.print("IP: ");
    SHOW_SERIAL.println(WiFi.localIP());
  } else {
    SHOW_SERIAL.println("WiFi FAILED - check credentials");
  }

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("https", "tcp", 443);
    SHOW_SERIAL.print("mDNS: https://");
    SHOW_SERIAL.print(MDNS_HOST);
    SHOW_SERIAL.println(".local/");
  } else {
    SHOW_SERIAL.println("mDNS init failed");
  }

  start_https_server();

  SHOW_SERIAL.println("Ready.");
  SHOW_SERIAL.println("Open: https://xiao-synth.local/");
}

void loop() {
  delay(1000);
}
