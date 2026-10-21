struct Evt;   // forward-declared before the includes so arduino-cli's injected prototypes (pushEvt(Evt&) etc.) resolve
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ====== IDENTITY ======
// OTA / generic build: leave these BLANK so each board keeps the identity
// already stored in its flash. Only fill them in for a first USB provisioning flash.
#define BOARD_NUM 0
const char* BOARD_CODE = "";
const char* DEVICE_ID  = "";
const char* DEVICE_KEY = "";
// ======================

#define FW_VERSION "1.4.0"

// Batch event ingest (Stage 1): the PI-SHEQsys RPC. Auth = _device_id + _key in the body (validated
// server-side as sha256(_key)==api_key_hash), but PostgREST still needs the project's PUBLIC anon key in
// the apikey/Bearer headers. This key is published in the web app to every browser - it is NOT a secret and
// does nothing without a valid device key in the body.
const char* EVENTS_RPC_URL  = "https://cofrgojpwdyzfhfqnlch.supabase.co/rest/v1/rpc/device_events_push";
const char* SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImNvZnJnb2pwd2R5emZoZnFubGNoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Nzc0Mzg0MjIsImV4cCI6MjA5MzAxNDQyMn0.fc1dOwvHJEhRDDnfOzj4EpqhoN9qE2CWf9T8995yoBM";

const char* HEARTBEAT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-heartbeat";
const char* TAG_EVENT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-tag-event";
const char* CMD_RESULT_URL  = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-command-result";
const char* FW_VERSION_URL  = "https://raw.githubusercontent.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/main/fw-version.txt";
const char* FIRMWARE_URL    = "https://github.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/releases/latest/download/firmware.bin";
const char* MACHINE_EVENT_URL = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-machine-event";
const char* INJECTION_DEVICE_ID = "ec089dbd-7269-4c1d-86b8-e425400f80bf";  // machine 7 (board 7): injection monitor
const char* MACHINE6_DEVICE_ID  = "03100ff2-811b-47af-ba77-fe204ec1f9e6";  // machine 6 (board 6): injection monitor

Preferences prefs;
int boardNum = BOARD_NUM;
String deviceId = "";
String deviceKey = "";
String boardCode = "";

#define SCK_PIN  12
#define MISO_PIN 13
#define MOSI_PIN 11
#define SS_PIN   18
#define RST_PIN  17

WiFiMulti wifiMulti;
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key mifareKey;

bool rc522Ok = false;
bool firstBeat = true;
bool wifiUp = false;
bool everConnected = false;             // reached the server at least once this boot
bool everSawTag = false;                // read a tag at least once THIS boot (gates the reader-stuck reboot)
// ---- Injection-machine signal monitoring (machine 7 ONLY; dormant on every other board) ----
// Seven PC817-isolated signals from the moulding machine. Confirmed ACTIVE-LOW (GPIO 6 proved it:
// pulled LOW when the mould opened, released HIGH). So: INPUT_PULLUP on every channel, and a pin is
// "active" when it reads LOW, "idle" when it reads HIGH. An unconnected/idle input rests HIGH via the
// pull-up = idle (no false events); a real signal sinks it LOW = active. The seven struct entries are
// the SINGLE source of truth for BOTH the event path and the raw_levels string, so a signal name can
// never drift relative to its GPIO. GPIO 5 (purple, terminal 112 spare) is now "mould_closing" on every
// board - added as the LAST entry so raw_levels keeps the first 7 positions. It is held (not reported)
// until it toggles once, so a board where GPIO 5 isn't wired yet (pin stuck at pull-up) stays silent.
bool injectionArmed = false;
struct MachineSignal { byte pin; const char* name; bool isInjection; };  // isInjection drives the shot counter
MachineSignal machine7Signals[] = {
  {  4, "injection",        true  },   // raw_levels pos 1 - terminal 105
  {  6, "mould_opening",    false },   // raw_levels pos 2 - terminal 109  (confirmed working)
  {  7, "ejecting_forward", false },   // raw_levels pos 3 - terminal 110
  { 15, "carriage_forward", false },   // raw_levels pos 4 - terminal 104
  { 21, "charge",           false },   // raw_levels pos 5 - terminal 106  (plasticizing / screw recovery); moved off GPIO16
  { 40, "carriage_back",    false },   // raw_levels pos 6 - terminal 108
  { 41, "ejecting_back",    false },   // raw_levels pos 7 - terminal 111
  {  5, "mould_closing",    false },   // raw_levels pos 8 - GPIO5 purple -> MOULD CLOSE (active-LOW, machine-7 polarity); gated until wired
};
// Machine 6 (Yizumi UN200A5S, board 6), wired 15 Oct 2026. Its Y-outputs SINK: 24 V at idle, 0 V when
// active; opto + = signal wire, opto - = 0 V (202), so the opto LED is ON at idle and the GPIO reads
// HIGH when the function is ACTIVE (opposite of machine 7) -> machine6ActiveHigh = true.
// NO alarm (127): the opto's leakage through the alarm buzzer/lamp kept the alarm on. Never tap 127 this way.
MachineSignal machine6Signals[] = {
  {  4, "injection",        true  },   // blue   - terminal 118 (Y18 inject conversion, one pulse per shot)
  {  6, "mould_opening",    false },   // grey   - terminal 109
  {  7, "ejecting_forward", false },   // white  - terminal 110
  { 15, "carriage_forward", false },   // black  - terminal 104
  { 16, "charge",           false },   // brown  - terminal 106
  { 40, "carriage_back",    false },   // orange - terminal 108
  { 41, "ejecting_back",    false },   // red    - terminal 111
  {  5, "mould_closing",    false },   // purple - GPIO5 -> MOULD CLOSE (active-HIGH std); gated until wired (raw_levels pos 8)
};
MachineSignal* machineSignals = machine7Signals;     // chosen at boot by board identity
int  NUM_SIGNALS   = sizeof(machine7Signals) / sizeof(machine7Signals[0]);
bool sigActiveHigh = false;                          // false = LOW is active (machine 7); true = HIGH is active (machine 6)
const char* machineLabel = "machine 7";
#define MAX_SIGNALS 8

// ===================== STAGE 1: lossless machine-signal capture + batch ingest =====================
// A both-edge GPIO ISR timestamps every edge (esp_timer us) into a FreeRTOS queue; a CAPTURE task pinned to
// CORE 0 debounces (20 ms, stamped at the FIRST transition) and emits one event per real change into a big
// RAM ring; a SENDER task on CORE 1 batches events to the device_events_push RPC and deletes only what the
// server acks. Capture never shares a loop with Wi-Fi/HTTP, so sending can never make us miss an edge. Both
// tasks are fed to the task watchdog so a hang reboots instead of freezing.

// Core signals define a shot and are the only ones checked for stuck/silent: injection, charge,
// mould_opening, mould_closing. Carriage forward/back are position confirmations - recorded, never judged.
bool isCoreSignal(const char* n) {
  return !strcmp(n,"injection") || !strcmp(n,"charge") || !strcmp(n,"mould_opening") || !strcmp(n,"mould_closing");
}

const unsigned long SIG_DEBOUNCE_US = 20000;         // 20 ms stable to accept a change (per input)

// Raw edge from the ISR: pin index + the level read AT the edge + the esp_timer microsecond stamp.
struct RawEdge { uint8_t ch; uint8_t level; int64_t tUs; };
QueueHandle_t rawEdgeQ = nullptr;                    // ISR -> capture task (length 256)

// One captured, debounced event. Compact (pin index, not name) so a deep ring stays affordable.
struct Evt {
  uint32_t seq;
  uint8_t  pin;                                      // index into machineSignals[]; 0xFF = raw_levels
  uint8_t  active;                                   // 1 = active, 0 = idle
  uint32_t uptimeMs;                                 // ms at the FIRST transition (boot-relative)
  uint64_t tsMs;                                     // epoch ms at the FIRST transition (0 until NTP synced)
  uint32_t pulseMs;                                  // ON duration for idle events (0 otherwise)
  char     rawlv[12];                                // raw_levels h/l string (pin==0xFF only)
};
int   EVT_CAP = 5000;                                // ring capacity (clamped if PSRAM absent)
Evt*  evtRing = nullptr;
volatile int evtHead = 0;                            // oldest unsent
volatile int evtCount = 0;
portMUX_TYPE evtMux = portMUX_INITIALIZER_UNLOCKED;  // guards evtRing head/count (capture vs sender)

// Per-input capture FSM (capture task only) + rolling diagnostics reported in _status.
bool          capCommitted[MAX_SIGNALS];             // last committed active/idle
int8_t        capPend[MAX_SIGNALS];                  // pending level (-1 none, else 0/1 active)
int64_t       capPendUs[MAX_SIGNALS];                // us of the FIRST transition of the pending change
uint32_t      capCounter[MAX_SIGNALS];               // rising edges since boot (counters)
uint32_t      capNoise[MAX_SIGNALS];                 // sub-debounce reversals (noise)
int64_t       capActiveSinceUs[MAX_SIGNALS];         // us the input last went active (for pulse_ms)
unsigned long capLastChangeMs[MAX_SIGNALS];          // millis of last committed change (stuck/silent)
unsigned long capActiveStartMs[MAX_SIGNALS];         // millis the input last went active (stuck check)
volatile unsigned long injectionShotCount = 0;       // kept for the shot_count field + NVS continuity
unsigned long lastShotSaved = 0;

// seq counter: survives reboot in RTC, and power loss via NVS (restored on cold boot, saved periodically).
RTC_NOINIT_ATTR uint32_t rtcSeq;
RTC_NOINIT_ATTR uint32_t rtcSeqMagic;
const uint32_t RTC_SEQ_MAGIC = 0x5EC0FFEE;
volatile uint32_t gSeq = 0;
uint32_t lastSeqSaved = 0;

TaskHandle_t captureTaskH = nullptr, senderTaskH = nullptr;
bool pinmapSent = false;                             // pinmap included in _status once per boot
bool firstOkPost = false;                            // one ok:true POST happened (OTA validation gate)

// ---- NTP real-time + boot-anchor (accurate per-event timestamps) ----
bool          timeSynced   = false;
uint64_t      bootEpochMs  = 0;
unsigned long lastNtpCheck = 0;
unsigned long lastNtpResync = 0;
const unsigned long NTP_CHECK_INTERVAL  = 5000;
const unsigned long NTP_RESYNC_INTERVAL = 3UL * 3600UL * 1000UL;
String currentUID = "";
byte lastReaderVersion = 0;             // last RC522 version byte read (for the status line)
unsigned long lastSeenAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastRfidPollAt = 0;
unsigned long lastWifiCheckAt = 0;
unsigned long lastOtaAt = 0;            // millis() stamped when the last OTA check RETURNED (0 = none yet)
bool          otaInProgress = false;    // an OTA download/flash is running right now (heartbeat sends ota_in_s=0)
String        lastOtaResult = "pending"; // ok|no_update|http_404|flash_fail|http_<code>|wifi_down|pending
bool          otaPendingValidation = false; // new image booted, not yet proven good (rollback gate)
unsigned long otaBootMs = 0;             // millis at boot, for the 60s validation window
unsigned long lastReaderCheckAt = 0;
unsigned long lastStatusAt = 0;         // last time we printed the human status line
unsigned long lastCloseProbeAt = 0;     // last low-power "close tag" probe while searching
unsigned long lastGoodContactAt = 0;   // last successful (200) heartbeat
unsigned long emptySince = 0;          // when a HEALTHY reader first saw an empty field (0 = not empty)
unsigned long lastReprobeAt = 0;       // last recovery attempt while holding a tracked tag
unsigned long lastRefreshAt = 0;       // last proactive field-refresh blink while a tag is present
int           reprobeCount = 0;        // escalating recovery step count for the held tag
unsigned long lastLevelSnapAt = 0;      // last raw pin-level snapshot POST (machine 7 diagnostic)
unsigned long lastRawPrintAt = 0;       // last 1 Hz RAW serial print (machine 7 diagnostic)
unsigned long lastReinitAt = 0;         // last FULL reader re-init while hunting for a tag (auto-recovery)
unsigned long readerDownSince = 0;      // when the reader first went invalid/disconnected (0 = reader ok)

const unsigned long HEARTBEAT_INTERVAL   = 5000;
const unsigned long RFID_POLL_INTERVAL   = 250;
const unsigned long WIFI_CHECK_INTERVAL  = 5000;
const unsigned long LEVEL_SNAP_INTERVAL  = 15000;    // machine 7: raw pin-level snapshot period (diagnostic)
const unsigned long OTA_CHECK_INTERVAL   = 60000;    // 60s
const unsigned long READER_CHECK_INTERVAL = 3000;    // hot-plug re-check
const unsigned long STATUS_INTERVAL      = 2000;     // print a clear per-board status line this often
const unsigned long CLOSE_PROBE_INTERVAL = 1000;     // how often (max) to dip to low power for a close tag
const unsigned long REINIT_INTERVAL      = 8000;     // while hunting for a tag, FULLY re-init the reader
                                                     // (RST + PCD_Init + config, same as a reboot) this
                                                     // often - recovers an "answers but won't read" wedge
                                                     // that antenna blinks and serviceReader can't, so a
                                                     // stuck reader self-heals instead of staying "removed".
const unsigned long FIELD_REFRESH_INTERVAL = 20000;  // while a tag is present, proactively blink the field
                                                     // this often so it never sits in a continuous field
                                                     // long enough to go unresponsive (prevents sticking).
const unsigned long READER_DOWN_CLEAR_MS = 30000;    // reader invalid/disconnected this long (not a glitch)
                                                     // -> stop asserting a phantom held tag (no reader can
                                                     // confirm it). Prevents "present" on an unplugged reader.
const unsigned long OFFLINE_REBOOT_TIMEOUT = 120000; // no server contact for 120s -> self-reboot. 120s (was
                                                     // 60s) tolerates a transient server slowdown under bulk
                                                     // load (many boards writing at once) so the whole fleet
                                                     // doesn't reboot-loop together.
const unsigned long READER_STUCK_REBOOT_MS = 45000;  // held a tag, then reader reads NOTHING (no UID, not
                                                     // even a bare field answer) for this long -> the reader
                                                     // has wedged in the way only a reboot clears (same as
                                                     // "it works again after a software update"). Self-reboot
                                                     // to recover. Gated on everSawTag so a spare/empty reader
                                                     // never loops, and self-limiting (everSawTag clears on
                                                     // reboot) so a genuinely-removed tag reboots at most once.
const unsigned long REMOVE_CONFIRM_MS    = 8000;     // A HEALTHY reader must see a genuinely EMPTY field
                                                     // this long, CONTINUOUSLY, before a tracked tag is
                                                     // declared removed. A tag physically on the reader
                                                     // answers the field ping every poll and resets this,
                                                     // so it stays "present" indefinitely (months). A
                                                     // wedged/down reader never counts toward removal - it
                                                     // holds the tag and recovers. 8s (was 20s): a real
                                                     // removal off a healthy reader now reports ~3x quicker;
                                                     // a wedged reader can flash a brief false "removed" but
                                                     // self-recovers, and only the wedge-prone boards do that.
const unsigned long REPROBE_AFTER_MS     = 2000;     // when a tracked tag stops answering, cycle the antenna
                                                     // (drop+re-apply the field = "software re-seat" the tag)
                                                     // at most this often, before the empty window can run
                                                     // toward removal. A sitting tag that has gone
                                                     // unresponsive answers again after the field blinks.

void resolveIdentity() {
  prefs.begin("ident", false);
  if (strlen(DEVICE_ID) > 0 && strlen(DEVICE_KEY) > 0) {
    prefs.putString("id",   DEVICE_ID);
    prefs.putString("key",  DEVICE_KEY);
    prefs.putInt("num", BOARD_NUM);
    prefs.putString("code", BOARD_CODE);
  }
  deviceId  = prefs.getString("id",   "");
  deviceKey = prefs.getString("key",  "");
  boardNum  = prefs.getInt("num", BOARD_NUM);
  boardCode = prefs.getString("code", "");
  prefs.end();
}

String jsonStr(const String& s, const String& key) {
  String pat = "\"" + key + "\":\"";
  int i = s.indexOf(pat);
  if (i < 0) return "";
  i += pat.length();
  int j = s.indexOf('"', i);
  if (j < 0) return "";
  return s.substring(i, j);
}
long jsonInt(const String& s, const String& key, long def) {
  String pat = "\"" + key + "\":";
  int i = s.indexOf(pat);
  if (i < 0) return def;
  i += pat.length();
  while (i < (int)s.length() && s[i] == ' ') i++;
  int j = i;
  while (j < (int)s.length() && (isdigit(s[j]) || s[j] == '-')) j++;
  if (j == i) return def;
  return s.substring(i, j).toInt();
}

bool versionNewer(const String& remote, const String& local) {
  int r0=0,r1=0,r2=0,l0=0,l1=0,l2=0;
  sscanf(remote.c_str(), "%d.%d.%d", &r0,&r1,&r2);
  sscanf(local.c_str(),  "%d.%d.%d", &l0,&l1,&l2);
  if (r0 != l0) return r0 > l0;
  if (r1 != l1) return r1 > l1;
  return r2 > l2;
}

String uidToString(MFRC522::Uid uid) {
  String s = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
    if (i < uid.size - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

int postJson(const char* url, const String& body, String& respOut) {
  if (WiFi.status() != WL_CONNECTED) return -1;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  if (!http.begin(client, url)) return -2;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Id", deviceId);
  http.addHeader("X-Device-Key", deviceKey);
  int code = http.POST(body);
  respOut = http.getString();
  http.end();
  // ANY successful post (heartbeat, tag event, command result, machine event) counts as server contact.
  // This stops the offline watchdog from rebooting a board that is busy and successfully talking to the
  // server (e.g. processing a queue of writes) just because a plain heartbeat hasn't fired in a while.
  if (code >= 200 && code < 300) lastGoodContactAt = millis();
  return code;
}

void postTagEvent(const String& uid, const String& eventType) {
  String resp;
  String body = "{\"tag_uid\":\"" + uid + "\",\"event_type\":\"" + eventType + "\"}";
  int code = postJson(TAG_EVENT_URL, body, resp);
  Serial.println("[tag] " + eventType + " " + uid + " -> code=" + String(code));
}

void postCommandResult(const String& cmdId, const String& status, const String& detail) {
  String resp;
  String body = "{\"command_id\":\"" + cmdId + "\",\"status\":\"" + status + "\",\"detail\":\"" + detail + "\"}";
  int code = postJson(CMD_RESULT_URL, body, resp);
  Serial.println("[cmd] result " + status + " (" + detail + ") -> code=" + String(code));
}

bool writeTagBlock(int block, const String& data, String& detail) {
  if (!rc522Ok) { detail = "reader not detected"; return false; }
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    detail = "no tag on reader"; return false;
  }
  byte buf[16];
  memset(buf, 0, 16);
  for (int i = 0; i < 16 && i < (int)data.length(); i++) buf[i] = (byte)data[i];

  // NTAG / Mifare Ultralight tags (SAK 0x00) do NOT support Mifare Classic authentication. The old
  // code ran PCD_Authenticate on EVERY tag, which ALWAYS fails on an NTAG - and that failed auth
  // left the reader in a state that jammed subsequent reads (the small round bin tags are NTAG, so
  // repeated failed writes were knocking the bin-tag reads out). Detect NTAG by its SAK and write it
  // the correct way: 4-byte page writes, no auth. Mifare Classic tags fall through and behave exactly
  // as before, so nothing changes for cards/tags that were already writing fine.
  if (mfrc522.uid.sak == 0x00) {
    bool ok = true;
    for (int p = 0; p < 4 && ok; p++) {                 // 16 bytes = 4 x 4-byte NTAG pages
      byte page[4] = { buf[p*4], buf[p*4+1], buf[p*4+2], buf[p*4+3] };
      if (mfrc522.MIFARE_Ultralight_Write((byte)(block + p), page, 4) != MFRC522::STATUS_OK) ok = false;
    }
    detail = ok ? ("wrote ntag pages " + String(block) + "-" + String(block + 3)) : "ntag write failed";
    mfrc522.PICC_HaltA();
    return ok;
  }

  // Mifare Classic: authenticate the block with key A, then write (unchanged behaviour).
  MFRC522::StatusCode st = mfrc522.PCD_Authenticate(
      MFRC522::PICC_CMD_MF_AUTH_KEY_A, (byte)block, &mifareKey, &(mfrc522.uid));
  if (st != MFRC522::STATUS_OK) {
    detail = "auth failed";
    mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1();
    return false;
  }
  st = mfrc522.MIFARE_Write((byte)block, buf, 16);
  bool ok = (st == MFRC522::STATUS_OK);
  detail = ok ? ("wrote block " + String(block)) : "write failed";
  mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1();
  return ok;
}

// ================= NDEF (NTAG21x) writing for bin tags =================
// Build a well-formed NDEF message (optional URI record + optional Text record), wrap it in the NTAG
// NDEF Message TLV, and write it to user memory from page 4 with plain page writes (NTAG/Ultralight,
// SAK 0x00, has no Classic key auth). Then read the pages back and decode the text so a write can be
// verified end-to-end. The existing Classic write_tag path is untouched.

String jsonEscape(const String& s) {
  String o = "";
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else if ((unsigned char)c >= 0x20) o += c;   // drop other control chars
  }
  return o;
}

String tagTypeName(byte sak) {
  if (sak == 0x00) return "NTAG/Ultralight";
  if (sak == 0x08) return "MIFARE Classic 1K";
  if (sak == 0x18) return "MIFARE Classic 4K";
  if (sak == 0x09) return "MIFARE Mini";
  if (sak & 0x20)  return "ISO14443-4";
  return "SAK 0x" + String(sak, HEX);
}

// NFC Forum URI RTD prefix abbreviations - trims the matched prefix from uri and returns its code.
byte uriPrefixCode(String& uri) {
  const char* pfx[]  = {"https://www.", "http://www.", "https://", "http://"};
  const byte  code[] = {0x02,            0x01,          0x04,        0x03};
  for (int i = 0; i < 4; i++) {
    if (uri.startsWith(pfx[i])) { uri = uri.substring(strlen(pfx[i])); return code[i]; }
  }
  return 0x00;   // no abbreviation -> store the whole URI
}

// Assemble the NDEF message (records only, no TLV) into buf. Returns length, or -1 if empty/too big.
int buildNdefMessage(const String& url, const String& text, byte* buf, int cap) {
  bool hasUri  = url.length()  > 0;
  bool hasText = text.length() > 0;
  if (!hasUri && !hasText) return -1;
  int n = 0;
  if (hasUri) {                                        // URI record ('U')
    String u = url; byte c = uriPrefixCode(u);
    int payLen = 1 + u.length();
    if (payLen > 255 || n + 4 + payLen > cap) return -1;
    byte hdr = 0x80 | 0x10 | 0x01;                     // MB | SR | TNF=well-known
    if (!hasText) hdr |= 0x40;                          // ME if also the last record
    buf[n++] = hdr; buf[n++] = 1; buf[n++] = (byte)payLen; buf[n++] = 'U'; buf[n++] = c;
    for (unsigned int i = 0; i < u.length(); i++) buf[n++] = (byte)u[i];
  }
  if (hasText) {                                       // Text record ('T'), lang "en"
    int payLen = 1 + 2 + text.length();
    if (payLen > 255 || n + 4 + payLen > cap) return -1;
    byte hdr = 0x40 | 0x10 | 0x01;                     // ME | SR | TNF=well-known
    if (!hasUri) hdr |= 0x80;                           // MB if first record
    buf[n++] = hdr; buf[n++] = 1; buf[n++] = (byte)payLen; buf[n++] = 'T';
    buf[n++] = 0x02; buf[n++] = 'e'; buf[n++] = 'n';    // status (UTF-8, lang len 2) + "en"
    for (unsigned int i = 0; i < text.length(); i++) buf[n++] = (byte)text[i];
  }
  return n;
}

// Scan up to `ms` for a tag and select it (fills mfrc522.uid). WakeupA so a tag already sitting on the
// reader is caught, not only a freshly-arrived one. Returns true if a tag was selected.
bool selectTagWithin(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    byte atqa[2]; byte sz = sizeof(atqa);
    mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
    MFRC522::StatusCode s = mfrc522.PICC_WakeupA(atqa, &sz);
    if ((s == MFRC522::STATUS_OK || s == MFRC522::STATUS_COLLISION) && mfrc522.PICC_ReadCardSerial())
      return true;
    delay(50);
  }
  return false;
}

// Decode the first NDEF Text record from a page buffer; fall back to the URI record's URI. "" if none.
String decodeNdefText(const byte* d, int len) {
  int i = 0, msgLen = -1, msgStart = -1;
  while (i < len) {                                    // find the NDEF Message TLV (0x03)
    byte t = d[i++];
    if (t == 0x00) continue;                            // NULL TLV
    if (t == 0xFE) break;                               // terminator
    if (i >= len) break;
    int l = d[i++];
    if (l == 0xFF) { if (i + 1 >= len) break; l = (d[i] << 8) | d[i+1]; i += 2; }
    if (t == 0x03) { msgLen = l; msgStart = i; break; }
    i += l;
  }
  if (msgStart < 0) return "";
  int p = msgStart, end = msgStart + msgLen;
  if (end > len) end = len;
  String uriOut = "";
  while (p < end) {                                     // walk the records
    byte hdr = d[p++]; bool sr = hdr & 0x10; bool il = hdr & 0x08;
    if (p >= end) break;
    int typeLen = d[p++];
    long payLen;
    if (sr) payLen = d[p++];
    else { if (p + 3 >= end) break; payLen = ((long)d[p]<<24)|((long)d[p+1]<<16)|((long)d[p+2]<<8)|d[p+3]; p += 4; }
    int idLen = 0; if (il) { if (p >= end) break; idLen = d[p++]; }
    if (p + typeLen > end) break;
    char type = (typeLen >= 1) ? (char)d[p] : 0;
    p += typeLen + idLen;
    if (p + payLen > end) payLen = end - p;
    if (type == 'T' && payLen >= 1) {
      int langLen = d[p] & 0x3F;
      int txtStart = p + 1 + langLen, txtLen = payLen - 1 - langLen;
      String out = "";
      for (int k = 0; k < txtLen && txtStart + k < end; k++) out += (char)d[txtStart + k];
      return out;                                       // prefer the text record
    }
    if (type == 'U' && payLen >= 1 && uriOut.length() == 0) {
      const char* pfx = "";
      switch (d[p]) { case 0x01: pfx="http://www."; break; case 0x02: pfx="https://www."; break;
                      case 0x03: pfx="http://"; break; case 0x04: pfx="https://"; break; }
      String u = pfx;
      for (int k = 1; k < payLen && p + k < end; k++) u += (char)d[p + k];
      uriOut = u;
    }
    p += payLen;
  }
  return uriOut;
}

// Like decodeNdefText but returns BOTH the first TEXT record (lang byte skipped) and the first URI record
// via reference params, so a read can report them separately (read_back = text, detail = uri).
void decodeNdefRecords(const byte* d, int len, String& textOut, String& uriOut) {
  textOut = ""; uriOut = "";
  int i = 0, msgLen = -1, msgStart = -1;
  while (i < len) {
    byte t = d[i++];
    if (t == 0x00) continue;
    if (t == 0xFE) break;
    if (i >= len) break;
    int l = d[i++];
    if (l == 0xFF) { if (i + 1 >= len) break; l = (d[i] << 8) | d[i+1]; i += 2; }
    if (t == 0x03) { msgLen = l; msgStart = i; break; }
    i += l;
  }
  if (msgStart < 0) return;
  int p = msgStart, end = msgStart + msgLen;
  if (end > len) end = len;
  while (p < end) {
    byte hdr = d[p++]; bool sr = hdr & 0x10; bool il = hdr & 0x08;
    if (p >= end) break;
    int typeLen = d[p++];
    long payLen;
    if (sr) payLen = d[p++];
    else { if (p + 3 >= end) break; payLen = ((long)d[p]<<24)|((long)d[p+1]<<16)|((long)d[p+2]<<8)|d[p+3]; p += 4; }
    int idLen = 0; if (il) { if (p >= end) break; idLen = d[p++]; }
    if (p + typeLen > end) break;
    char type = (typeLen >= 1) ? (char)d[p] : 0;
    p += typeLen + idLen;
    if (p + payLen > end) payLen = end - p;
    if (type == 'T' && payLen >= 1 && textOut.length() == 0) {
      int langLen = d[p] & 0x3F;
      int txtStart = p + 1 + langLen, txtLen = payLen - 1 - langLen;
      for (int k = 0; k < txtLen && txtStart + k < end; k++) textOut += (char)d[txtStart + k];
    } else if (type == 'U' && payLen >= 1 && uriOut.length() == 0) {
      const char* pfx = "";
      switch (d[p]) { case 0x01: pfx="http://www."; break; case 0x02: pfx="https://www."; break;
                      case 0x03: pfx="http://"; break; case 0x04: pfx="https://"; break; }
      uriOut = pfx;
      for (int k = 1; k < payLen && p + k < end; k++) uriOut += (char)d[p + k];
    }
    p += payLen;
  }
}

// NTAG storage size from the CC size byte (page 3, byte 2).
String ntagTypeFromCCSize(byte sizeByte) {
  if (sizeByte == 0x12) return "ntag213";
  if (sizeByte == 0x3E) return "ntag215";
  if (sizeByte == 0x6D) return "ntag216";
  return "unknown";
}

// Shared antenna levels for tuning: TX drive AND RX gain stepped down together = "tag further away",
// which is how an over-coupled CLOSE tag is read/written. Level 0 = full power/max gain (a distance tag).
struct RfLevel { byte gsn, cwgsp, modgsp, rxgain; };
const RfLevel RF_LEVELS[] = {
  { 0xFF, 0x3F, 0x3F, 0x07 << 4 },   // full  TX + 48dB - distance tag
  { 0x88, 0x20, 0x20, 0x06 << 4 },   //             43dB
  { 0x44, 0x10, 0x10, 0x04 << 4 },   // low   TX + 33dB - close
  { 0x22, 0x08, 0x08, 0x01 << 4 },   // v.low  TX + 23dB - very close
  { 0x11, 0x04, 0x04, 0x00 << 4 },   // min    TX + 18dB - severe over-couple
  { 0x08, 0x02, 0x02, 0x00 << 4 },   // ultra  TX + 18dB - a STRONG reader over-coupling at fixed distance
  { 0x04, 0x01, 0x01, 0x00 << 4 },   // floor  TX + 18dB - strongest reader / tag almost on the coil
};
const int RF_NLEVELS = sizeof(RF_LEVELS) / sizeof(RF_LEVELS[0]);
void setRfLevel(int i) {
  if (i < 0) i = 0; if (i >= RF_NLEVELS) i = RF_NLEVELS - 1;
  setAntennaDrive(RF_LEVELS[i].gsn, RF_LEVELS[i].cwgsp, RF_LEVELS[i].modgsp);
  mfrc522.PCD_SetAntennaGain(RF_LEVELS[i].rxgain);
}
void setRfFull() { setRfLevel(0); }

// Re-grab a tag mid-write at a SPECIFIC tuned level (used between page-write retries so we stay at the
// level the tag couples well, instead of snapping back to full and over-coupling a close tag again).
bool reselectAtLevel(int level) {
  setRfLevel(level);
  return selectTagWithin(300);
}

// READY-TO-WRITE: get the coupling as steady as possible before the first page burn. Re-grab the tag a few
// times at the tuned level with short warm-ups so its capacitor is fully charged and the field is settled
// when the write hits - the write draws a current pulse a half-charged / unsteady tag can't deliver, which
// is why a marginal board fails at page 4 though the select succeeded. Leaves the tag selected.
void stabilizeForWrite(int level) {
  int good = 0;
  for (int i = 0; i < 8 && good < 3; i++) {              // up to 8 tries to see 3 clean selects in a row
    if (reselectAtLevel(level)) { good++; delay(25); }   // charge between confirms
    else { good = 0; delay(15); }
  }
  delay(40);                                             // final top-up charge before the first write
}

// Acquire a tag by SWEEPING RF levels (TX + RX gain), not just full power. An OVER-coupled close tag won't
// select at full power - which is why "no tag on reader (2s scan)" fired even with a tag sitting there.
// Tries full first (best for a distance tag), then progressively "further away". Leaves the winning level
// set and the tag selected; returns the level index, or -1 (restores full) if nothing selects within `ms`.
int selectTagSweep(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    for (int i = 0; i < RF_NLEVELS; i++) {
      setRfLevel(i);
      if (selectTagWithin(120)) return i;
    }
  }
  setRfFull();
  return -1;
}

bool writeNdefTag(const String& url, const String& text, String& detail, String& readBack) {
  if (!rc522Ok) { detail = "reader not detected"; return false; }
  // Acquire via the LEVEL SWEEP (not full power only) so an over-coupled close tag is found and the level
  // is tuned in one step - this is what "no tag on reader (2s scan)" was missing.
  int wlvl = selectTagSweep(5000);                       // 5s (was 2s): wait out a tag that flickers off the
  if (wlvl < 0) { detail = "no tag on reader (5s sweep)"; return false; }   // coil, grab it the instant it's back
  String ttype = tagTypeName(mfrc522.uid.sak);
  if (mfrc522.uid.sak != 0x00) {                        // Classic goes through write_tag, not here
    detail = "not an NTAG (type=" + ttype + ", sak=0x" + String(mfrc522.uid.sak, HEX) + ")";
    mfrc522.PICC_HaltA(); return false;
  }
  // wlvl (from selectTagSweep above) is already the tag's best coupling level - we write there. Full power
  // OVER-couples a close tag and its write fails at page 4; writing at the tuned level is the fix.
  // Quiet the WiFi radio for the duration of the write. WiFi TX bursts share the board's 3.3V with the
  // reader; silencing them frees supply headroom for the reader's current spike during the EEPROM burn -
  // the software stand-in for a decoupling cap. Restored on EVERY exit below (paired with setRfFull).
  WiFi.setSleep(true);
  stabilizeForWrite(wlvl);                               // charge the tag + settle the field before writing
  // Capability Container (page 3): a phone/tablet reads THIS FIRST to decide the tag is an NDEF tag at
  // all. If it isn't E1 10 xx 00, the tag is invisible to phones even though the NDEF is written - our own
  // read-back still decodes it, which is why "verified" tags wouldn't read on your tablet. A factory NTAG
  // already has a valid CC (starts 0xE1) and we leave it; a blank/wiped tag gets a standard CC written.
  {
    byte cc[18]; byte ccLen = sizeof(cc);
    bool haveCC = (mfrc522.MIFARE_Read(3, cc, &ccLen) == MFRC522::STATUS_OK && cc[0] == 0xE1);
    if (!haveCC) {
      byte ccPage[4] = { 0xE1, 0x10, 0x12, 0x00 };        // NDEF magic, v1.0, 144B (NTAG213), read/write
      bool ccWrote = false;
      for (int a = 0; a < 4 && !ccWrote; a++) {
        if (mfrc522.MIFARE_Ultralight_Write(3, ccPage, 4) == MFRC522::STATUS_OK) ccWrote = true;
        else { delay(10); reselectAtLevel(wlvl); }
      }
      if (!ccWrote) {
        detail = "capability container (page 3) write failed - tag locked? (type=" + ttype + ")";
        mfrc522.PICC_HaltA(); setRfFull(); WiFi.setSleep(false); return false;
      }
      delay(5);
    }
  }
  byte msg[240];
  int mlen = buildNdefMessage(url, text, msg, sizeof(msg));
  if (mlen < 0) { detail = "ndef empty or too large (type=" + ttype + ")"; mfrc522.PICC_HaltA(); return false; }
  byte tlv[248]; int tn = 0;
  tlv[tn++] = 0x03; tlv[tn++] = (byte)mlen;             // NDEF Message TLV + 1-byte length (<255 here)
  for (int i = 0; i < mlen; i++) tlv[tn++] = msg[i];
  tlv[tn++] = 0xFE;                                     // terminator TLV
  while (tn % 4 != 0) tlv[tn++] = 0x00;                 // pad to whole pages
  int pages = tn / 4;
  // WARM-UP: hold the field on briefly before the first write so a MARGINALLY-COUPLED tag charges its
  // internal capacitor fully - an EEPROM page write needs a current pulse the tag can only deliver when
  // charged, which is why a weak seat fails at the very first data page even though the select succeeded.
  delay(40);
  for (int pg = 0; pg < pages; pg++) {
    byte page[4] = { tlv[pg*4], tlv[pg*4+1], tlv[pg*4+2], tlv[pg*4+3] };
    // Retry each page: a tag that slips out of the field mid-write (weak coupling) fails one page even
    // though it's not locked. On failure re-grab the tag and retry this page before giving up, so a
    // marginal seat still completes instead of aborting the whole write.
    bool wrote = false;
    for (int attempt = 0; attempt < 8 && !wrote; attempt++) {   // 8 tries for a marginal seat
      if (mfrc522.MIFARE_Ultralight_Write((byte)(4 + pg), page, 4) == MFRC522::STATUS_OK) { wrote = true; break; }
      delay(10);
      reselectAtLevel(wlvl);                             // re-grab at the tuned level (don't snap back to full)
      delay(30);                                         // warm-up: let the re-grabbed tag charge before retrying
    }
    if (!wrote) {
      // Don't GUESS "locked" - read the STATIC LOCK BYTES (page 2, bytes 2-3) and report the truth. If
      // they're set, the tag is genuinely locked (irreversible - replace it). If they're CLEAR, a blank
      // read/write tag is NOT locked and the write failed for another reason (weak reader/coupling on this
      // board, or password protection) - so we don't tell the user to bin a good tag.
      byte lk[18]; byte lkl = sizeof(lk);
      bool lkRead = (mfrc522.MIFARE_Read(2, lk, &lkl) == MFRC522::STATUS_OK);   // did the page-2 read succeed?
      byte l0 = lkRead ? lk[2] : 0x00, l1 = lkRead ? lk[3] : 0x00;
      bool reallyLocked = lkRead && (l0 != 0x00 || l1 != 0x00);
      String hex = "0x" + String(l0, HEX) + " 0x" + String(l1, HEX);
      if (pg == 0 && !lkRead)
        // Could NOT even read the lock bytes -> the reader can't read this tag at all (coupling/wedge on
        // this board). NOT proof of a lock. Don't tell the user to replace a good tag.
        detail = "write failed AND lock bytes UNREADABLE - reader can't read this tag (coupling/wedge on this board), not necessarily locked - reposition tag / power-cycle & retry (type=" + ttype + ")";
      else if (pg == 0 && reallyLocked)
        detail = "tag LOCKED - static lock bytes " + hex + " set (irreversible; replace this tag) (type=" + ttype + ")";
      else if (pg == 0)
        detail = "page 4 write failed but lock bytes CLEAR (" + hex + ") - NOT locked; weak reader/coupling on this board or password - reposition tag / check reader (type=" + ttype + ")";
      else
        detail = "ntag write failed at page " + String(4 + pg) + " - tag slipped out of field mid-write, reposition/steady it (type=" + ttype + ")";
      mfrc522.PICC_HaltA(); setRfFull(); WiFi.setSleep(false); return false;
    }
    delay(5);                                            // let the NTAG commit the page before the next write
  }
  // Honest verify: a phone reads the tag ONLY if the CC magic (page 3 byte 0) is 0xE1 AND page 4 begins
  // with the NDEF TLV (0x03). Read both and report what a phone would actually see - not just whether our
  // own decoder found the text.
  byte cc3[18]; byte cc3Len = sizeof(cc3); byte ccMagic = 0x00;
  if (mfrc522.MIFARE_Read(3, cc3, &cc3Len) == MFRC522::STATUS_OK) ccMagic = cc3[0];
  byte back[256]; int backLen = 0;
  for (int pg = 0; pg < pages + 2 && backLen + 16 <= (int)sizeof(back); pg += 4) {
    byte b[18]; byte bl = sizeof(b);
    if (mfrc522.MIFARE_Read((byte)(4 + pg), b, &bl) != MFRC522::STATUS_OK) break;
    for (int k = 0; k < 16; k++) back[backLen++] = b[k];
  }
  mfrc522.PICC_HaltA();
  setRfFull(); WiFi.setSleep(false);                     // restore full drive + max gain + WiFi for normal ops
  readBack = decodeNdefText(back, backLen);
  bool phoneReadable = (ccMagic == 0xE1) && (backLen > 0) && (back[0] == 0x03);
  detail = String(phoneReadable ? "wrote+verified NDEF, phone-readable" : "wrote NDEF but NOT phone-readable")
           + " (" + String(pages) + " pages @ RF level " + String(wlvl) + ", CC=0x" + String(ccMagic, HEX) + ", type=" + ttype + ")";
  return phoneReadable;   // report real usability: a tag a phone can't read is a failed write
}

void postCommandResultNdef(const String& cmdId, const String& status, const String& detail, const String& readBack) {
  String resp;
  String body = "{\"command_id\":\"" + cmdId + "\",\"status\":\"" + status +
                "\",\"detail\":\"" + jsonEscape(detail) + "\",\"read_back\":\"" + jsonEscape(readBack) + "\"}";
  int code = postJson(CMD_RESULT_URL, body, resp);
  Serial.println("[cmd] ndef " + status + " read_back='" + readBack + "' -> code=" + String(code));
}

// Result poster for read_ndef: includes tag_uid + tag_type alongside read_back + detail.
void postCommandResultRead(const String& cmdId, const String& status, const String& tagUid,
                           const String& tagType, const String& readBack, const String& detail) {
  String resp;
  String body = "{\"command_id\":\"" + cmdId + "\",\"status\":\"" + status +
                "\",\"tag_uid\":\"" + jsonEscape(tagUid) + "\",\"tag_type\":\"" + jsonEscape(tagType) +
                "\",\"read_back\":\"" + jsonEscape(readBack) + "\",\"detail\":\"" + jsonEscape(detail) + "\"}";
  int code = postJson(CMD_RESULT_URL, body, resp);
  Serial.println("[cmd] read_ndef " + status + " uid=" + tagUid + " type=" + tagType +
                 " text='" + readBack + "' -> code=" + String(code));
}

// READ-ONLY: scan for a tag, decode its NDEF, and report it. Writes NOTHING (no page writes, no lock bytes).
void readNdefTag(const String& cmdId) {
  if (!rc522Ok) { postCommandResultRead(cmdId, "fail", "", "unknown", "", "reader not detected"); return; }
  // Sweep RF levels to acquire (an over-coupled close tag won't select at full power), then restore full.
  if (selectTagSweep(4000) < 0) { postCommandResult(cmdId, "fail", "no tag on reader (4s sweep)"); return; }
  String uid = uidToString(mfrc522.uid);
  byte sak = mfrc522.uid.sak;
  String tagType, textOut = "", uriOut = "";
  int bytesRead = 0;
  if (sak != 0x00) {                                   // MIFARE Classic: auth + read block 4
    tagType = "classic";
    byte buf[18]; byte bl = sizeof(buf);
    if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &mifareKey, &mfrc522.uid) == MFRC522::STATUS_OK
        && mfrc522.MIFARE_Read(4, buf, &bl) == MFRC522::STATUS_OK) {
      bytesRead = 16;
      decodeNdefRecords(buf, 16, textOut, uriOut);
    }
    mfrc522.PCD_StopCrypto1();
  } else {                                             // NTAG/Ultralight: read pages 4..39
    byte data[160]; int n = 0;
    for (int pg = 4; pg <= 39 && n + 16 <= (int)sizeof(data); pg += 4) {
      byte b[18]; byte bl = sizeof(b);
      if (mfrc522.MIFARE_Read((byte)pg, b, &bl) != MFRC522::STATUS_OK) break;
      for (int k = 0; k < 16; k++) data[n++] = b[k];
    }
    bytesRead = n;
    byte cc[18]; byte ccl = sizeof(cc); byte ccSize = 0x00;   // CC size byte (page 3, byte 2) -> tag model
    if (mfrc522.MIFARE_Read(3, cc, &ccl) == MFRC522::STATUS_OK) ccSize = cc[2];
    tagType = ntagTypeFromCCSize(ccSize);
    decodeNdefRecords(data, n, textOut, uriOut);
  }
  mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1();     // release the tag; normal polling (WUPA) resumes
  setRfFull();                                         // restore full drive + max gain after the level sweep
  String detail = uriOut.length() ? ("uri=" + uriOut) : ("read " + String(bytesRead) + " bytes");
  postCommandResultRead(cmdId, "ok", uid, tagType, textOut, detail);
}

void handleCommand(const String& resp) {
  String cmd = jsonStr(resp, "command");
  if (cmd == "" || cmd == "null") return;
  String cmdId = jsonStr(resp, "command_id");
  if (cmd == "write_tag") {
    int block = (int)jsonInt(resp, "block", 4);
    String data = jsonStr(resp, "data");
    Serial.println("[cmd] write_tag id=" + cmdId + " block=" + String(block) + " data=" + data);
    String detail;
    bool ok = writeTagBlock(block, data, detail);
    postCommandResult(cmdId, ok ? "ok" : "fail", detail);
  } else if (cmd == "write_ndef") {
    String url  = jsonStr(resp, "url");
    String text = jsonStr(resp, "text");
    Serial.println("[cmd] write_ndef id=" + cmdId + " url=" + url + " text=" + text);
    String detail, readBack;
    bool ok = writeNdefTag(url, text, detail, readBack);
    postCommandResultNdef(cmdId, ok ? "ok" : "fail", detail, readBack);
  } else if (cmd == "read_ndef") {
    Serial.println("[cmd] read_ndef id=" + cmdId);
    readNdefTag(cmdId);                                 // read-only: decodes + reports, writes nothing
  } else if (cmd == "recover_reader") {
    // Backend-triggered recovery: the dashboard can send this when a board looks stale. Force the full
    // escalation (longer field blink + reader re-init) and re-scan, so a stuck reader/tag is revived
    // on demand without anyone touching the hardware. Reports what it found.
    Serial.println("[cmd] recover_reader id=" + cmdId);
    cycleAntenna(250);
    startRC522();
    reprobeCount = 0; emptySince = 0;
    String found = readTagUid();
    String detail = rc522Ok ? ("reader re-inited (v0x" + String(lastReaderVersion, HEX) + ")")
                            : "reader still not detected";
    if (found != "" && found != currentUID) { adoptTag(found); }
    else if (found != "") { lastSeenAt = millis(); }
    postCommandResult(cmdId, rc522Ok ? "ok" : "fail",
                      detail + (found != "" ? (", tag=" + found) : ", no tag"));
  } else if (cmd == "clear_tag") {
    // Backend is the authority on removal: it says the tag is genuinely gone, so clear it. Together
    // with recover_reader this splits the ambiguity the board can't resolve alone - the server decides
    // "really removed" (clear_tag) vs "stuck, heal yourself" (recover_reader). Optionally scoped to a
    // uid so a stale command can't clear a tag that has since changed.
    String want = jsonStr(resp, "uid");                 // optional: only clear if it matches
    Serial.println("[cmd] clear_tag id=" + cmdId + " uid=" + want + " (holding " + currentUID + ")");
    if (currentUID == "") { postCommandResult(cmdId, "ok", "no tag to clear"); }
    else if (want != "" && want != currentUID) { postCommandResult(cmdId, "ok", "held " + currentUID + " != " + want + ", not cleared"); }
    else {
      String had = currentUID;
      postTagEvent(currentUID, "removed");
      currentUID = ""; emptySince = 0; everSawTag = false;   // human says bin is gone -> don't reboot-recover
      postCommandResult(cmdId, "ok", "cleared " + had);
    }
  } else {
    postCommandResult(cmdId, "fail", "unknown command");
  }
}

void sendHeartbeat() {
  if (deviceId == "" || deviceKey == "") {
    Serial.println("[id] MISSING IDENTITY - flash a board with DEVICE_ID/DEVICE_KEY - heartbeat disabled");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) return;
  String ip = WiFi.localIP().toString();
  long rssi = WiFi.RSSI();
  // Accurate OTA schedule. Single source of truth = OTA_CHECK_INTERVAL, the SAME timer that drives the OTA
  // task; lastOtaAt is stamped when a check actually RETURNS (see loop). ota_in_s counts down to the next
  // real check, and is 0 while a download/flash is in progress. last_ota_result/age let the server see the
  // true state (never a placeholder).
  long otaAgeS = (long)((millis() - lastOtaAt) / 1000);
  long otaInS  = otaInProgress ? 0 : ((long)(OTA_CHECK_INTERVAL / 1000) - otaAgeS);
  if (otaInS < 0) otaInS = 0;
  // reader_ok = is the CHIP ANSWERING, not "is it fully re-configured". A real version byte (anything but
  // 0x00/0xFF, which mean genuinely dead/absent SPI) OR a tag currently held proves the reader is alive and
  // talking. This is deliberately decoupled from the internal rc522Ok gate: rc522Ok also requires the antenna
  // config to have "stuck" and flaps false during a self-heal re-init, which on a busy machine board made the
  // heartbeat report reader_ok=false even while the reader answered 0x92 and read tags (the 1.2.0 regression).
  // A genuinely dead reader still reads 0x00/0xFF -> reader_ok false, so the real SPI-dead alarm is preserved.
  bool readerAnswering = (lastReaderVersion != 0x00 && lastReaderVersion != 0xFF) || currentUID != "";
  String body = String("{\"firmware_version\":\"") + FW_VERSION +
                "\",\"ip\":\"" + ip +
                "\",\"rssi\":" + String(rssi) +
                ",\"board_code\":\"" + boardCode + "\"" +
                ",\"reader_ok\":" + (readerAnswering ? "true" : "false") +   // chip answering (real ver) or tag held
                ",\"reader_ver\":\"0x" + String(lastReaderVersion, HEX) + "\"" +  // RC522 version byte
                ",\"tag\":\"" + currentUID + "\"" +                          // UID currently held ("" = none)
                ",\"ota_in_s\":" + String(otaInS) +                          // seconds to next OTA check (0 = in progress)
                ",\"ota_interval_s\":" + String(OTA_CHECK_INTERVAL / 1000) + // real OTA check period
                ",\"last_ota_result\":\"" + lastOtaResult + "\"" +           // ok|no_update|http_404|flash_fail|wifi_down|pending
                ",\"last_ota_check_age_s\":" + String(otaAgeS) +            // seconds since the last check returned
                ",\"boot\":" + (firstBeat ? "true" : "false") + "}";
  String resp;
  int code = postJson(HEARTBEAT_URL, body, resp);
  String shortResp = resp.length() > 120 ? resp.substring(0, 120) : resp;
  Serial.println("[hb] board=" + String(boardNum) + " code=" + String(code) +
                 " rssi=" + String(rssi) + " ip=" + ip + " resp=" + shortResp);
  if (code == 401) Serial.println("[hb] BAD DEVICE KEY - check X-Device-Key");
  if (code <= 0)   Serial.println("[hb] NO CONNECTION code=" + String(code));
  if (code == 200) { firstBeat = false; everConnected = true; lastGoodContactAt = millis(); handleCommand(resp); }
}

// Both-edge interrupt (one per input, channel index passed as arg). Minimal and in IRAM: timestamp the edge
// with esp_timer (us) and hand it to the capture task. Interrupts preempt everything, so an edge is captured
// even while the loop or sender is mid-HTTP. All debounce/state/shot logic lives in captureTask on core 0.
void IRAM_ATTR machineSignalISR(void* arg) {
  BaseType_t hpw = pdFALSE;
  RawEdge e;
  e.ch    = (uint8_t)(int)(intptr_t)arg;
  e.level = (uint8_t)digitalRead(machineSignals[e.ch].pin);      // level AT the edge
  e.tUs   = esp_timer_get_time();                                // microsecond stamp of the edge
  if (rawEdgeQ) xQueueSendFromISR(rawEdgeQ, &e, &hpw);           // ALL debounce/state logic is in the core-0 task
  if (hpw) portYIELD_FROM_ISR();
}

// POST one machine signal state change to Supabase (machine 7). Reuses the SAME X-Device-Id/
// X-Device-Key auth as the heartbeat. Server stamps the time (created_at); we send which signal,
// its new state, the running shot count and uptime.
// ISO 8601 UTC with milliseconds from an epoch-ms value.
// Arduino String has no 64-bit constructor; format epoch-ms (and any uint64) via snprintf.
String u64str(uint64_t v) { char b[21]; snprintf(b, sizeof(b), "%llu", (unsigned long long)v); return String(b); }

String isoFromEpochMs(uint64_t epochMs) {
  time_t s = (time_t)(epochMs / 1000ULL);
  int ms = (int)(epochMs % 1000ULL);
  struct tm tmv;
  gmtime_r(&s, &tmv);
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
  return String(buf);
}

// device_ts for an event captured at edge uptime `uptimeMs`: boot_epoch + uptime (boot-anchored, so an
// event captured BEFORE the clock was set still gets an accurate time once NTP syncs). "" until synced.
String deviceTsFor(unsigned long uptimeMs) {
  if (!timeSynced) return "";
  return isoFromEpochMs(bootEpochMs + (uint64_t)uptimeMs);
}

// Non-blocking NTP: detect the first valid time and set the boot anchor; re-sync every few hours. configTime
// kicks off SNTP in the background - this only polls time(nullptr), so it never blocks the main loop.
void ntpService() {
  unsigned long nowMs = millis();
  if (nowMs - lastNtpCheck >= NTP_CHECK_INTERVAL) {
    lastNtpCheck = nowMs;
    time_t s = time(nullptr);
    if (s > 1700000000) {                                             // a real epoch (after 2023) -> clock is set
      bootEpochMs = (uint64_t)s * 1000ULL - (uint64_t)millis();       // re-anchor each check = drift-corrected
      if (!timeSynced) { timeSynced = true; Serial.println("[ntp] synced; boot_epoch anchored"); }
    }
  }
  if (nowMs - lastNtpResync >= NTP_RESYNC_INTERVAL) {
    lastNtpResync = nowMs;
    configTime(0, 0, "pool.ntp.org", "time.google.com");              // re-trigger SNTP (UTC)
  }
}

// Add a finished event to the RAM ring (under the spinlock). Assigns the next seq here so the counter is
// serialized across the capture and sender tasks. If the ring is full, drop the OLDEST.
void pushEvt(Evt& ev) {
  portENTER_CRITICAL(&evtMux);
  ev.seq = ++gSeq; rtcSeq = gSeq;                                     // seq survives reboot via RTC
  int idx = (evtHead + evtCount) % EVT_CAP;
  evtRing[idx] = ev;
  if (evtCount < EVT_CAP) evtCount++;
  else evtHead = (evtHead + 1) % EVT_CAP;
  portEXIT_CRITICAL(&evtMux);
}

// Current raw pin levels as an H/L string in machineSignals order (8th char = mould_closing).
void rawLevelsStr(char* out, int cap) {
  int n = 0;
  for (int i = 0; i < NUM_SIGNALS && n < cap - 1; i++)
    out[n++] = (digitalRead(machineSignals[i].pin) == HIGH) ? 'H' : 'L';
  out[n] = '\0';
}

// CORE 0: drain raw edges, debounce 20 ms per input (commit stamped at the FIRST transition), emit one
// event per committed change, maintain rising-edge counters + noise. Never touches Wi-Fi/HTTP.
void captureTask(void* pv) {
  esp_task_wdt_add(NULL);
  RawEdge e;
  for (;;) {
    esp_task_wdt_reset();
    while (xQueueReceive(rawEdgeQ, &e, pdMS_TO_TICKS(4)) == pdTRUE) {
      int i = e.ch;
      if (i < 0 || i >= NUM_SIGNALS) continue;
      bool active = (e.level == (sigActiveHigh ? HIGH : LOW));
      if (active == capCommitted[i]) {                               // returned to committed level = bounce
        if (capPend[i] != -1) { capNoise[i]++; capPend[i] = -1; }
        continue;
      }
      if (capPend[i] != (int8_t)active) {                            // new pending change -> stamp FIRST transition
        if (capPend[i] != -1) capNoise[i]++;
        capPend[i]   = (int8_t)active;
        capPendUs[i] = e.tUs;
      }
    }
    int64_t nowUs = esp_timer_get_time();
    for (int i = 0; i < NUM_SIGNALS; i++) {
      if (capPend[i] == -1) continue;
      if (nowUs - capPendUs[i] < (int64_t)SIG_DEBOUNCE_US) continue; // not yet stable 20 ms
      bool active = (capPend[i] != 0);
      capPend[i] = -1;
      capCommitted[i] = active;
      capLastChangeMs[i] = millis();
      Evt ev; memset(&ev, 0, sizeof(ev));
      ev.pin      = (uint8_t)i;
      ev.active   = active ? 1 : 0;
      ev.uptimeMs = (uint32_t)(capPendUs[i] / 1000);
      ev.tsMs     = timeSynced ? (bootEpochMs + (uint64_t)(capPendUs[i] / 1000)) : 0;
      if (active) {
        capActiveSinceUs[i] = capPendUs[i];
        capActiveStartMs[i] = millis();
        capCounter[i]++;                                             // rising edge
        if (machineSignals[i].isInjection) injectionShotCount++;
        ev.pulseMs = 0;
      } else {
        ev.pulseMs = capActiveSinceUs[i] ? (uint32_t)((capPendUs[i] - capActiveSinceUs[i]) / 1000) : 0;
      }
      pushEvt(ev);
    }
  }
}

const char* evtSignalName(const Evt& ev) {
  return (ev.pin == 0xFF) ? "raw_levels" : machineSignals[ev.pin].name;
}

// Build the _status object: last_seq, buffered, per-input counters + noise, stuck/silent (core signals only,
// never carriage), pinmap (once per boot), fw. Called from the sender.
String buildStatusJson() {
  unsigned long now = millis();
  unsigned long recentChange = 0;
  for (int i = 0; i < NUM_SIGNALS; i++) if (capLastChangeMs[i] > recentChange) recentChange = capLastChangeMs[i];
  bool othersCycling = (recentChange != 0) && (now - recentChange < 600000UL);
  String s = "{\"last_seq\":" + String(gSeq) + ",\"buffered\":" + String(evtCount) + ",\"fw\":\"" + FW_VERSION + "\"";
  s += ",\"counters\":{"; for (int i = 0; i < NUM_SIGNALS; i++) { if (i) s += ","; s += "\"" + String(machineSignals[i].name) + "\":" + String(capCounter[i]); } s += "}";
  s += ",\"noise\":{";    for (int i = 0; i < NUM_SIGNALS; i++) { if (i) s += ","; s += "\"" + String(machineSignals[i].name) + "\":" + String(capNoise[i]); }    s += "}";
  String stuck = "", silent = "";
  for (int i = 0; i < NUM_SIGNALS; i++) {
    if (!isCoreSignal(machineSignals[i].name)) continue;            // never judge carriage/ejectors
    if (capCommitted[i] && othersCycling && capActiveStartMs[i] && (now - capActiveStartMs[i]) > 600000UL) {
      if (stuck.length()) stuck += ","; stuck += "\"" + String(machineSignals[i].name) + "\"";
    }
    if (othersCycling && capLastChangeMs[i] && (now - capLastChangeMs[i]) > 86400000UL) {
      if (silent.length()) silent += ","; silent += "\"" + String(machineSignals[i].name) + "\"";
    }
  }
  s += ",\"stuck\":[" + stuck + "],\"silent\":[" + silent + "]";
  if (!pinmapSent) {
    s += ",\"pinmap\":{";
    for (int i = 0; i < NUM_SIGNALS; i++) { if (i) s += ",";
      s += "\"" + String(machineSignals[i].name) + "\":{\"gpio\":" + String(machineSignals[i].pin) +
           ",\"active_high\":" + (sigActiveHigh ? "true" : "false") + "}"; }
    s += "}";
  }
  s += "}";
  return s;
}

// POST up to 500 oldest events + _status to the device_events_push RPC (anon key in headers, device creds in
// body). On ok:true the server has durably stored everything it received, so `ackedOut` = events sent.
bool postEventsBatch(int& ackedOut) {
  ackedOut = 0;
  if (WiFi.status() != WL_CONNECTED) return false;
  String events = "";
  int sent = 0;
  if (timeSynced) {                                                  // hold events until the clock is real
    portENTER_CRITICAL(&evtMux);
    int n = evtCount; if (n > 500) n = 500;
    int idx = evtHead;
    portEXIT_CRITICAL(&evtMux);
    for (int k = 0; k < n; k++) {
      Evt ev = evtRing[idx];
      idx = (idx + 1) % EVT_CAP;
      uint64_t ts = ev.tsMs ? ev.tsMs : (bootEpochMs + (uint64_t)ev.uptimeMs);  // backfill pre-sync events
      if (events.length()) events += ",";
      events += "{\"seq\":" + String(ev.seq) +
                ",\"signal\":\"" + String(evtSignalName(ev)) + "\"" +
                ",\"state\":\"" + (ev.pin == 0xFF ? String(ev.rawlv) : String(ev.active ? "active" : "idle")) + "\"" +
                ",\"shot_count\":" + String(injectionShotCount) +
                ",\"uptime_ms\":" + String(ev.uptimeMs) +
                ",\"ts\":" + u64str(ts) +
                ",\"pulse_ms\":" + String(ev.pulseMs) + "}";
      sent++;
    }
  }
  String body = String("{\"_device_id\":\"") + deviceId + "\",\"_key\":\"" + deviceKey +
                "\",\"_events\":[" + events + "],\"_status\":" + buildStatusJson() + "}";
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(5000); http.setTimeout(8000);
  if (!http.begin(client, EVENTS_RPC_URL)) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  int code = http.POST(body);
  String resp = http.getString();
  http.end();
  bool ok = (code >= 200 && code < 300) && resp.indexOf("\"ok\":true") >= 0;
  if (ok) { lastGoodContactAt = millis(); firstOkPost = true; pinmapSent = true; ackedOut = sent; }
  else Serial.println("[rpc] POST code=" + String(code) + " ok=0 resp=" + resp.substring(0, 80));
  return ok;
}

// CORE 1: every 1 s (or at >=50 queued) POST a batch and delete only what the server acked. Pushes a
// raw_levels snapshot into the ring every 5 s, and forces a _status at least every 60 s.
void senderTask(void* pv) {
  esp_task_wdt_add(NULL);
  unsigned long lastSend = 0, lastRaw = 0, lastStatus = 0;
  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();
    if (!otaInProgress && injectionArmed && now - lastRaw >= 5000) {         // raw_levels every 5 s (deduped/acked like any event)
      lastRaw = now;
      Evt ev; memset(&ev, 0, sizeof(ev));
      ev.pin = 0xFF; ev.active = 0;
      ev.uptimeMs = (uint32_t)(esp_timer_get_time() / 1000);
      ev.tsMs = timeSynced ? (bootEpochMs + ev.uptimeMs) : 0;
      rawLevelsStr(ev.rawlv, sizeof(ev.rawlv));
      pushEvt(ev);
    }
    bool due = (now - lastSend >= 1000) || (evtCount >= 50) || (now - lastStatus >= 60000);
    if (!otaInProgress && due) {
      int acked = 0;
      if (postEventsBatch(acked)) {
        lastSend = now; lastStatus = now;
        if (acked > 0) {
          portENTER_CRITICAL(&evtMux);
          int rem = acked; if (rem > evtCount) rem = evtCount;
          evtHead = (evtHead + rem) % EVT_CAP; evtCount -= rem;
          portEXIT_CRITICAL(&evtMux);
        }
      } else {
        lastSend = now;                                                      // retry next tick; events stay buffered
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Persist the injection shot count across reboots (NVS), throttled so we only touch flash when the
// count actually changed - injection cycles are seconds+ apart, so this is well within flash wear.
void saveShotCount() {
  unsigned long shots = injectionShotCount;
  if (shots == lastShotSaved) return;
  prefs.begin("machine", false);
  prefs.putULong("shots", shots);
  prefs.end();
  lastShotSaved = shots;
}

void checkOTA() {
  if (WiFi.status() != WL_CONNECTED) { lastOtaResult = "wifi_down"; return; }
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  if (!http.begin(client, FW_VERSION_URL)) { lastOtaResult = "begin_fail"; return; }
  int code = http.GET();
  if (code != 200) {                                             // version file unreachable (e.g. 404)
    http.end();
    lastOtaResult = "http_" + String(code);
    return;
  }
  String body = http.getString(); body.trim();
  http.end();
  int nl = body.indexOf('\n');
  String remoteVer = (nl < 0) ? body : body.substring(0, nl);
  String target    = (nl < 0) ? "all" : body.substring(nl + 1);
  remoteVer.trim(); target.trim();
  bool targeted = (target == "all" || target == String(boardNum) || target == deviceId);
  if (!(targeted && versionNewer(remoteVer, FW_VERSION))) {      // nothing newer for this board
    lastOtaResult = "no_update";
    return;
  }
  // A newer, targeted build exists -> download + flash. On success httpUpdate auto-reboots (we won't
  // return); on failure we record why.
  Serial.println("[ota] " + String(FW_VERSION) + " -> " + remoteVer + " ... downloading");
  otaInProgress = true;                                          // sender/raw_levels pause on this flag
  if (captureTaskH) vTaskSuspend(captureTaskH);                  // freeze capture + sender during the flash write
  if (senderTaskH)  vTaskSuspend(senderTaskH);
  if (injectionArmed)                                            // no ISR during flash write
    for (int i = 0; i < NUM_SIGNALS; i++) detachInterrupt(digitalPinToInterrupt(machineSignals[i].pin));
  WiFiClientSecure up; up.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  t_httpUpdate_return r = httpUpdate.update(up, FIRMWARE_URL);
  otaInProgress = false;
  if (r != HTTP_UPDATE_OK) {                                     // success reboots; on failure resume capture
    if (injectionArmed)
      for (int i = 0; i < NUM_SIGNALS; i++)
        attachInterruptArg(digitalPinToInterrupt(machineSignals[i].pin), machineSignalISR, (void*)(intptr_t)i, CHANGE);
    if (captureTaskH) vTaskResume(captureTaskH);
    if (senderTaskH)  vTaskResume(senderTaskH);
  }
  if (r == HTTP_UPDATE_FAILED) {
    int err = httpUpdate.getLastError();
    lastOtaResult = (err == HTTP_UE_SERVER_FILE_NOT_FOUND || err == HTTP_UE_SERVER_FORBIDDEN) ? "http_404" : "flash_fail";
    Serial.println("[ota] FAILED: " + String(err) + " " + httpUpdate.getLastErrorString());
  } else if (r == HTTP_UPDATE_NO_UPDATES) {
    lastOtaResult = "no_update";
  } else {
    lastOtaResult = "ok";                                        // reached only if reboot is somehow deferred
  }
}

// ---- OTA validation + rollback (app-level) ----
// A freshly OTA'd image must PROVE itself: Wi-Fi up, one ok:true POST to device_events_push, and 60 s with
// no crash/watchdog reboot. Only then is it marked valid. If it crash-loops (boots 3x without validating),
// revert to the previous working image. True bootloader auto-rollback needs a custom bootloader config the
// Arduino build doesn't ship, so this is the app-level equivalent: it catches runtime faults that boot far
// enough to run this code (the realistic risk); a binary that faults before setup() still needs a USB flash.
void otaValidateInit() {
  otaBootMs = millis();
  prefs.begin("ota", false);
  String validated = prefs.getString("valid_ver", "");
  if (validated == FW_VERSION) {
    prefs.putUInt("boot_try", 0);
    otaPendingValidation = false;
  } else {
    uint32_t tries = prefs.getUInt("boot_try", 0) + 1;
    prefs.putUInt("boot_try", tries);
    otaPendingValidation = true;
    Serial.println("[ota] new image " + String(FW_VERSION) + " pending validation (boot try " + String(tries) + ")");
    if (tries >= 3) {
      Serial.println("[ota] crash-looping unvalidated -> rolling back to previous image");
      prefs.end();
      esp_ota_mark_app_invalid_rollback_and_reboot();            // reverts + reboots if a prior app exists
      prefs.begin("ota", false);                                 // (only reached if rollback isn't possible)
    }
  }
  prefs.end();
}

void otaValidateTick() {
  if (!otaPendingValidation) return;
  if (WiFi.status() == WL_CONNECTED && firstOkPost && (millis() - otaBootMs >= 60000)) {
    prefs.begin("ota", false);
    prefs.putString("valid_ver", FW_VERSION);
    prefs.putUInt("boot_try", 0);
    prefs.end();
    esp_ota_mark_app_valid_cancel_rollback();                    // harmless no-op if not in pending-verify
    otaPendingValidation = false;
    Serial.println("[ota] image validated (wifi + ok:true post + 60s) - locked in");
  }
}

// A KNOWN-GOOD RC522 version byte. Real MFRC522s report 0x91/0x92 (NXP) or 0x88/0x90/0xB2/0x12 (clones).
// Anything else - 0x00, 0xFF, or garbage like 0xEE - means the SPI link is returning junk (loose/miswired/
// noisy reader), NOT a working reader. The old check only rejected 0x00/0xFF, so a garbage 0xEE was wrongly
// accepted as "healthy" and the reader was never recovered. Reject anything not on this list.
bool isValidReaderVersion(byte v) {
  return v == 0x91 || v == 0x92 || v == 0x88 || v == 0x90 || v == 0xB2 || v == 0x12;
}

// Read the RC522 version register with retries (long cables give occasional bad reads). Retry until we get
// a VALID version (not just non-zero), so one garbage read doesn't get latched as the reader's identity.
byte readReaderVersion() {
  byte v = 0;
  for (int i = 0; i < 6; i++) {
    v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    if (isValidReaderVersion(v)) break;
    delay(5);
  }
  return v;
}

void startRC522() {
  for (byte i = 0; i < 6; i++) mifareKey.keyByte[i] = 0xFF;
  // Try progressively SLOWER SPI clocks. A reader that reads GARBAGE (0xEE) or NOTHING (0x0) is usually a
  // marginal / electrically-noisy SPI link over a long cable - NOT a dead module. A slower clock very often
  // gets it talking again with zero hardware changes, so we don't give up (or tell you to buy a reader)
  // until every speed has failed. A healthy reader answers at the first (fastest) speed, so it's unaffected.
  const uint32_t spiSpeeds[] = { 50000, 25000, 10000, 5000 };
  byte v = 0;
  for (int s = 0; s < 4; s++) {
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    SPI.setFrequency(spiSpeeds[s]);
    pinMode(RST_PIN, OUTPUT);               // hardware reset pulse (un-sticks a wedged reader)
    digitalWrite(RST_PIN, LOW);  delay(10);
    digitalWrite(RST_PIN, HIGH); delay(50);
    mfrc522.PCD_Init();
    delay(50);
    v = readReaderVersion();
    if (isValidReaderVersion(v)) {
      if (s > 0) Serial.println("[rc522] recovered at " + String(spiSpeeds[s]) + " Hz SPI (noisy cable)");
      break;                                // clean version at this speed -> keep it
    }
  }
  lastReaderVersion = v;
  rc522Ok = isValidReaderVersion(v);
  if (rc522Ok) {
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);        // max receive sensitivity (48 dB)
    mfrc522.PCD_AntennaOn();
    // Drive the antenna as hard as the chip allows -> strongest field / best range.
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x27 << 1), 0xFF); // GsNReg  (CW+Mod N-driver = max)
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x28 << 1), 0x3F); // CWGsPReg  (P-driver CW = max)
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x29 << 1), 0x3F); // ModGsPReg (P-driver Mod = max)
    // Verify the config actually stuck. On a hot-swap the DB9 contacts can be bouncing at the
    // instant we init, leaving a reader that answers on SPI (looks "detected") but is only
    // half-configured and won't read tags - the intermittent "sometimes it doesn't work after
    // I plug a reader in". If the write didn't take, do NOT mark it ready: leave rc522Ok false
    // so serviceReader re-inits on the next check, and keeps retrying until the contact is
    // solid and the config lands cleanly.
    byte gsnBack = mfrc522.PCD_ReadRegister((MFRC522::PCD_Register)(0x27 << 1));
    if (gsnBack != 0xFF) {
      rc522Ok = false;
      Serial.println("[rc522] init didn't stick (v0x" + String(v, HEX) +
                     " GsN=0x" + String(gsnBack, HEX) + ") - retrying (reseat the reader/DB9)");
    } else {
      Serial.println("[rc522] ready (v0x" + String(v, HEX) + ") - max gain + max drive");
    }
  } else {
    Serial.println("[rc522] NOT DETECTED (v0x" + String(v, HEX) + ") - heartbeat continues anyway");
  }
}

// Hot-plug + recovery: detect a reader connected after boot, AND recover one that wedges.
void serviceReader() {
  static byte missCount = 0;
  byte v = readReaderVersion();
  lastReaderVersion = v;
  bool present = isValidReaderVersion(v);       // garbage version (0xEE etc.) = bad SPI, treat as NOT present
  if (present) {
    missCount = 0;
    readerDownSince = 0;                        // reader is back/alive -> clear the disconnected timer
    // Re-init when either (a) we don't think a reader is up, or (b) a reader IS present but
    // is NOT carrying our configuration. Case (b) is the HOT-SWAP fix: if you pull one reader
    // and plug another in between version checks, rc522Ok never goes false, so the new reader
    // would sit uninitialised (antenna off, wrong mode) and read nothing until a manual RST.
    // GsNReg (0x27) is set to 0xFF ONLY by startRC522() and is never touched by the read path,
    // so a freshly powered reader (default GsN != 0xFF) reliably trips this and gets init'd.
    byte gsn = mfrc522.PCD_ReadRegister((MFRC522::PCD_Register)(0x27 << 1));
    if (!rc522Ok || gsn != 0xFF) {
      startRC522();                       // new/uninitialised reader -> configure it, no RST needed
    }
  } else {
    // Reader answered with nothing (v0x0). One glitch is normal on a long cable, so wait for a
    // couple of consecutive misses before acting (never disturb a working reader over one blip).
    // Then actively RE-INIT to recover a reader that has wedged in place - the old code only
    // recovered a reader that came back on its own, so one stuck reading v0x0 stayed dead until
    // a manual reset (exactly "tag presence randomly goes down and stays down"). The RST pulse
    // in startRC522 un-sticks it, so it recovers by itself.
    if (++missCount >= 2) {
      if (readerDownSince == 0) readerDownSince = millis();   // stamp when the reader first went down
      if (rc522Ok) {
        rc522Ok = false;
        Serial.println("[rc522] reader lost - attempting recovery");
      }
      startRC522();
    }
  }
}

// A single, human-readable line printed a couple times a second so you can tell exactly
// WHICH board you're looking at (its permanent MAC + board number) and whether its reader
// harness is actually alive. Uses only cached values -> it does NOT touch the RF/read path.
void printStatus() {
  String reader = rc522Ok
      ? ("OK  (v0x" + String(lastReaderVersion, HEX) + ")")
      : ("FAULT - no reader (v0x" + String(lastReaderVersion, HEX) + ") - check DB9 wiring/3V3");
  String tag  = (currentUID != "") ? currentUID : "-- none --";
  String wifi = (WiFi.status() == WL_CONNECTED) ? ("up " + WiFi.localIP().toString()) : "DOWN";
  Serial.println("[STATUS] board #" + String(boardNum) +
                 "  code=" + (boardCode != "" ? boardCode : "(unset)") +
                 "  MAC=" + WiFi.macAddress() +
                 "  READER=" + reader +
                 "  tag=" + tag +
                 "  wifi=" + wifi);
  // Live machine view (machine 7 only): raw GPIO level per channel, so each pin can be watched
  // changing while the machine runs - the fastest way to tell wiring from logic. Shot count last.
  if (injectionArmed) {
    String line = "[mach]";
    for (int i = 0; i < NUM_SIGNALS; i++) {
      line += " " + String(machineSignals[i].name) + "(gpio" + String(machineSignals[i].pin) + ")=" +
              String(digitalRead(machineSignals[i].pin) == (sigActiveHigh ? HIGH : LOW) ? "ACT" : "idle");
    }
    line += "  shots=" + String(injectionShotCount);
    Serial.println(line);
  }
}

// Set the antenna P/N driver strength (field power). Only the low-power "close tag" probe
// changes this; normal reads leave it at the full 0xFF set in startRC522.
void setAntennaDrive(byte gsn, byte cwgsp, byte modgsp) {
  mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x27 << 1), gsn);    // GsNReg
  mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x28 << 1), cwgsp);  // CWGsPReg
  mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x29 << 1), modgsp); // ModGsPReg
}

// One read attempt at the CURRENT power. Returns the tag UID, or "" if nothing read. Does not
// touch the drive registers, so at full power the steady field a distance read needs is untouched.
String tryReadOnce() {
  byte atqa[2];
  byte atqaSize = sizeof(atqa);
  mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
  MFRC522::StatusCode s = mfrc522.PICC_WakeupA(atqa, &atqaSize);
  if ((s == MFRC522::STATUS_OK || s == MFRC522::STATUS_COLLISION) && mfrc522.PICC_ReadCardSerial()) {
    String u = uidToString(mfrc522.uid);
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    return u;
  }
  return "";
}

// Lightweight "is a tag still answering the field?" ping - a bare WakeupA, no full select.
// A weakly-coupled tag (small round NTAG bin tags) will answer this even when it can't complete
// a full re-read. Returns true if anything responds.
bool tagAnswersField() {
  for (int attempt = 0; attempt < 3; attempt++) {
    byte atqa[2];
    byte atqaSize = sizeof(atqa);
    mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
    MFRC522::StatusCode r = mfrc522.PICC_WakeupA(atqa, &atqaSize);
    if (r == MFRC522::STATUS_OK || r == MFRC522::STATUS_COLLISION) return true;
  }
  return false;
}

bool isTagStillPresent() {
  // Tiered so it holds BOTH strong tags and weakly-coupled small ones without going stale:
  // 1. FULL re-read at full power (5 tries) - a strong tag confirms its exact UID -> honest,
  //    no stale. Field stays at the steady 0xFF from startRC522, so distance reads are undisturbed.
  for (int attempt = 0; attempt < 5; attempt++) {
    if (tryReadOnce() == currentUID) return true;
  }
  // 2. FULL re-read at low power - a CLOSE tag that over-couples the strong field.
  setAntennaDrive(0x44, 0x10, 0x10);
  bool close = (tryReadOnce() == currentUID);
  setAntennaDrive(0xFF, 0x3F, 0x3F);
  if (close) return true;
  // 3. Lenient WakeupA ping - a WEAKLY-COUPLED tag (the small round NTAG bin tags) answers the
  //    field but can't finish a full select. This is what the old code did, and it's what held
  //    the small tags before the stale-fix made the check strict. A genuinely REMOVED tag answers
  //    none of the three -> it still clears after REMOVE_TIMEOUT. A wedged reader (spurious
  //    answers) is handled separately by serviceReader's auto-recovery.
  return tagAnswersField();
}

// "Software re-seat": drop the RF field for a moment then bring it back. A passive tag is powered by
// the field, so this forces it to lose power and reset to its power-on (IDLE) state - exactly like
// physically lifting the tag off the reader and putting it back. It fixes a tag that has gone
// unresponsive after sitting in a continuous field (won't answer WUPA until the field is cycled). The
// antenna drive registers are untouched, so the field returns at full strength. No RST pin needed.
void cycleAntenna(unsigned long offMs) {
  mfrc522.PCD_AntennaOff();
  delay(offMs);
  mfrc522.PCD_AntennaOn();
  delay(8);
}

// Switch the tracked tag to a newly-seen UID (a swap): report the old one removed and the new present.
void adoptTag(const String& newUid) {
  Serial.println("Tag swapped: " + currentUID + " -> " + newUid);
  postTagEvent(currentUID, "removed");
  currentUID = newUid; lastSeenAt = millis(); emptySince = 0; everSawTag = true;
  postTagEvent(currentUID, "present");
}

// Read the tag UID with a few tries; "" if nothing answered.
String readTagUid() {
  String u = "";
  for (int a = 0; a < 5 && u == ""; a++) u = tryReadOnce();
  return u;
}

// Machine 7 ONLY: its strong reader OVER-couples a close tag. Proven by the user: moving the reader AWAY
// lets it read. Over-coupling has TWO parts - too much TX field AND the receiver saturating at max gain on
// a close tag's huge load-modulation signal. Lowering ONLY TX (my earlier sweep) didn't fix it; we must
// drop TX power AND RX gain together, which electronically emulates "the tag is further away" - the thing
// that actually worked. Full/max first (a distance tag reads unchanged), then progressively "further away".
String sweepReadLevels() {
  // Use the SHARED RF_LEVELS table (same one the write path uses) so detection sweeps exactly as low as
  // writing does - down to the ultra-low/floor levels for the strongest over-coupling readers. These were
  // added to RF_LEVELS for writes but the read sweep had its own shorter table and only went to 0x11, so a
  // very strong reader could WRITE a close tag but never DETECT it. One table now fixes that fleet-wide.
  String u = "";
  for (int L = 0; L < RF_NLEVELS && u == ""; L++) {
    setRfLevel(L);                                         // steps BOTH TX drive and RX gain down together
    for (int a = 0; a < 3 && u == ""; a++) u = tryReadOnce();
  }
  setRfFull();                                             // restore full drive + max gain for the steady field
  return u;
}

void pollRfid() {
  // Over-coupling (a strong RC522 can't read a CLOSE tag at full power) affects MANY boards in the fleet,
  // not just M7 - Mixer, the production boards, etc. The sweep (drop BOTH TX power and RX gain to emulate
  // "tag further away") only runs when full power finds nothing, so a distance tag reads unchanged and this
  // only ADDS the ability to find a close over-coupled tag. So run it on EVERY board's detection - the write
  // path already sweeps fleet-wide, and detection must too so a close tag is always found automatically.
  bool strongReader = true;
  // ---- Holding a tracked tag: bias HEAVILY toward "still present" ----
  // A tag physically on the reader must NEVER falsely report "removed", even after months. Removal is
  // accepted ONLY when a HEALTHY reader confirms a genuinely empty field, continuously, for
  // REMOVE_CONFIRM_MS. A wedged/down reader can't judge presence, so it holds the tag and lets
  // serviceReader recover it. A weakly-coupled tag that only answers a bare field ping still counts as
  // present. A different UID appearing is handled as a swap (removed old + present new).
  if (currentUID != "") {
    if (!rc522Ok) {                                    // reader down
      // Hold the tag through a BRIEF glitch (don't false-remove). But if the reader has been dead/
      // disconnected for READER_DOWN_CLEAR_MS (genuinely unplugged, not a glitch), stop asserting a
      // phantom tag - there is no reader to confirm it, so reporting "present" is just wrong (this is the
      // "says a tag is in place when the reader isn't even connected"). Clear it; it re-detects if the
      // reader comes back.
      if (readerDownSince && millis() - readerDownSince > READER_DOWN_CLEAR_MS) {
        Serial.println("Reader disconnected >" + String(READER_DOWN_CLEAR_MS / 1000) +
                       "s - clearing phantom tag: " + currentUID);
        postTagEvent(currentUID, "removed");
        currentUID = ""; emptySince = 0; everSawTag = false;
      } else {
        emptySince = 0;
      }
      return;
    }

    // Read the tag at FULL power: confirms the exact UID and catches a swap to a different tag.
    String seen = readTagUid();
    if (seen == currentUID) { lastSeenAt = millis(); emptySince = 0; reprobeCount = 0; return; }  // present
    if (seen != "") { adoptTag(seen); reprobeCount = 0; return; }                 // a different tag -> swap
    // LOW-POWER dip: a STRONG reader OVER-couples a CLOSE tag at full power and can't read it - it needs
    // a weaker field. This is the probe-style sweep applied to the PRESENCE check (not just detection),
    // so an over-coupling close tag is confirmed and HELD instead of going stale. This is the piece the
    // tag-hold rewrite had dropped.
    setAntennaDrive(0x44, 0x10, 0x10);
    String low = readTagUid();
    setAntennaDrive(0xFF, 0x3F, 0x3F);
    if (low == currentUID) { lastSeenAt = millis(); emptySince = 0; reprobeCount = 0; return; }  // present (close)
    if (low != "") { adoptTag(low); reprobeCount = 0; return; }                   // a different close tag -> swap
    // Machine 7: full multi-level sweep before giving up this pass, so an over-coupled tag sitting at a
    // level between full and the single low dip is still confirmed present and HELD (not dropped).
    if (strongReader) {
      String sw = sweepReadLevels();
      if (sw == currentUID) { lastSeenAt = millis(); emptySince = 0; reprobeCount = 0; return; }
      if (sw != "") { adoptTag(sw); reprobeCount = 0; return; }
    }
    // A weakly-coupled tag may still answer a bare field ping even when it can't complete a select - that
    // holds a genuinely-present weak tag. BUT on the strong reader (M7) we must NOT trust a bare field
    // answer to hold the UID: when you SWAP in a new tag it answers the field too, and holding the OLD
    // UID here masks the swap and stops us re-reading/re-sweeping. So M7 requires a real UID (from the
    // sweep above); if it can't get one, we DON'T hold on a bare answer - the removal window runs, the tag
    // clears, and detect re-sweeps every poll to identify whatever tag is actually on the reader now.
    if (tagAnswersField()) { lastSeenAt = millis(); emptySince = 0; reprobeCount = 0; return; }

    // No answer this pass. Before letting the removal window run, ESCALATE recovery: short antenna
    // blink -> longer blink -> full reader re-init. A sitting tag that has gone unresponsive answers
    // again after the field is dropped for long enough (this is why lifting it off and back on works).
    if (emptySince == 0) emptySince = millis();
    if (millis() - lastReprobeAt > REPROBE_AFTER_MS) {
      lastReprobeAt = millis();
      reprobeCount++;
      if      (reprobeCount <= 2) cycleAntenna(60);     // quick blink
      else if (reprobeCount <= 5) cycleAntenna(250);    // longer field-off -> fully drains a stubborn tag
      else { startRC522(); reprobeCount = 0; }          // heaviest: full reader re-init, then start over
      String again = readTagUid();
      if (again == currentUID || (again == "" && tagAnswersField())) { lastSeenAt = millis(); emptySince = 0; reprobeCount = 0; return; }
      if (again != "") { adoptTag(again); reprobeCount = 0; return; }
    }
    if (millis() - emptySince > REMOVE_CONFIRM_MS) {
      Serial.println("Tag removed (field empty " + String(REMOVE_CONFIRM_MS / 1000) + "s): " + currentUID);
      postTagEvent(currentUID, "removed");
      currentUID = ""; emptySince = 0;
    }
    return;
  }

  if (!rc522Ok) return;                                 // no tag tracked and reader down -> nothing to do
  // Detect a tag with WakeupA (WUPA, 0x52) - NOT PICC_IsNewCardPresent/REQA (0x26).
  // REQA only answers a tag that has JUST entered the field (IDLE state). A tag already
  // sitting on the reader, or one the reader previously halted, ignores REQA -> you would
  // only ever get "present" after physically removing + replacing the tag, or resetting the
  // board with it in place (and even then only sometimes). WUPA wakes those tags too, so a
  // tag that is already there registers as present immediately.
  // FULL POWER first (5 tries): the field is the steady 0xFF from startRC522, never rewritten,
  // so a DISTANCE tag reads here exactly as before - this is what the last sweep broke, fixed.
  String uid = "";
  for (int attempt = 0; attempt < 5 && uid == ""; attempt++) uid = tryReadOnce();
  // Machine 7: if full power found nothing, sweep all drive levels THIS poll (no throttle) so a close/
  // over-coupled tag is caught in one pass instead of only when it's moved/turned. Full-power-first above
  // means a distance tag still reads exactly as before; this only adds the lower levels on top.
  if (uid == "" && strongReader) uid = sweepReadLevels();
  // Only if full power found NOTHING (so there's no distance read to disturb), occasionally dip
  // to low power to catch a tag close enough to over-couple the strong field. Throttled to
  // CLOSE_PROBE_INTERVAL and snapped straight back to full, so the steady field is barely touched.
  if (uid == "" && millis() - lastCloseProbeAt >= CLOSE_PROBE_INTERVAL) {
    lastCloseProbeAt = millis();
    setAntennaDrive(0x44, 0x10, 0x10);
    for (int attempt = 0; attempt < 2 && uid == ""; attempt++) uid = tryReadOnce();
    setAntennaDrive(0xFF, 0x3F, 0x3F);
    // Still nothing: a tag may be sitting there but gone unresponsive. Blink the field to re-seat it,
    // then re-read - this re-finds a tag that had wrongly gone "removed" without anyone touching it.
    if (uid == "") { cycleAntenna(120); uid = readTagUid(); }
    // STILL nothing: periodically do a FULL reader re-init - the same reset a reboot does. This is what
    // recovers a reader that answers on SPI but has stopped reading (the wedge that stays "removed" and
    // never self-heals), so the board recovers on its own instead of needing a replug.
    if (uid == "" && millis() - lastReinitAt >= REINIT_INTERVAL) {
      lastReinitAt = millis();
      startRC522();
      uid = readTagUid();
    }
  }
  if (uid != "") {
    currentUID = uid;
    lastSeenAt = millis();
    everSawTag = true;
    Serial.println("Tag present: " + currentUID);
    postTagEvent(currentUID, "present");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  resolveIdentity();
  otaValidateInit();                     // start the OTA rollback gate: this image must prove itself in 60 s

  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  wifiMulti.addAP("Esp32Test",            "12345678");
  wifiMulti.addAP("Cottages",             "C0ttage@37");
  wifiMulti.addAP("ABSolutely Connected", "Wifi@613");

  Serial.println("========================================");
  Serial.println("FW " + String(FW_VERSION) + "  board #" + String(boardNum) + "  code=" + boardCode);
  Serial.println(">>> THIS BOARD's PERMANENT ID (MAC): " + WiFi.macAddress() + " <<<");
  Serial.print("Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { wifiMulti.run(); delay(20); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiUp = true;
    Serial.println("SSID " + WiFi.SSID());
    Serial.println("IP   " + WiFi.localIP().toString());
    Serial.println("MAC " + WiFi.macAddress());
  } else {
    Serial.println("WiFi not up yet - will keep retrying");
  }

  // Start SNTP (UTC) in the background. Non-blocking: ntpService() in loop() detects the first valid time
  // and sets the boot anchor; events captured before then are buffered and backfilled.
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  lastNtpResync = millis();

  if (deviceId != "" && deviceKey != "")
    Serial.println("[id] board=" + String(boardNum) + " code=" + boardCode + " id=" + deviceId + " keyLen=" + String(deviceKey.length()));
  else
    Serial.println("[id] MISSING IDENTITY - flash with DEVICE_ID/DEVICE_KEY set once");

  startRC522();

  // Machine monitoring: ARM only on machine 7 (matched by board number OR device id). Seven PC817
  // optocoupler taps of the moulding machine's signals, all ACTIVE-LOW with INPUT_PULLUP: an idle or
  // unconnected input rests HIGH (no false events), a real signal sinks it LOW = active. Edge-triggered
  // so an edge is caught even while the loop is busy in a blocking HTTP post. Arm by BOARD NUMBER *or*
  // device id so a device-id that doesn't match identities.txt can't leave monitoring silently dormant.
  bool isM7 = (boardNum == 7 || deviceId == INJECTION_DEVICE_ID);
  // Every other MATERIAL board (machines 1-6, 8) uses the machine-6 wiring standard (same colours/GPIOs,
  // sinking outputs, active-HIGH). Production boards (11+) and the mixer reader are never armed. An unwired
  // board just reads all-HIGH with no edges: no events, the dashboard shows it as "not wired yet".
  bool isM6 = (boardNum == 6 || deviceId == MACHINE6_DEVICE_ID);
  bool isStd = !isM7 && (isM6 || (boardNum >= 1 && boardNum <= 8));
  static char labelBuf[16];
  if (isStd) {
    machineSignals = machine6Signals;
    NUM_SIGNALS    = sizeof(machine6Signals) / sizeof(machine6Signals[0]);
    sigActiveHigh  = true;
    snprintf(labelBuf, sizeof(labelBuf), "machine %d", isM6 ? 6 : boardNum);
    machineLabel   = labelBuf;
  }
  if (isM7 || isStd) {
    // Restore the persisted shot count so a reboot doesn't reset it.
    prefs.begin("machine", true);
    injectionShotCount = prefs.getULong("shots", 0);
    prefs.end();
    lastShotSaved = injectionShotCount;
    // seq: prefer RTC (survives reboot); on a cold boot (RTC lost) restore from NVS. Monotonic across both.
    if (rtcSeqMagic != RTC_SEQ_MAGIC) {
      prefs.begin("seqs", true); gSeq = prefs.getUInt("seq", 0); prefs.end();
      rtcSeqMagic = RTC_SEQ_MAGIC;
    } else {
      gSeq = rtcSeq;
    }
    rtcSeq = gSeq; lastSeqSaved = gSeq;
    // Event ring: prefer PSRAM (big), fall back to internal RAM (clamped - the real need over a 2 min outage
    // is only a few hundred events, so even the fallback is ample).
    if (!evtRing) {
      evtRing = (Evt*)heap_caps_malloc((size_t)EVT_CAP * sizeof(Evt), MALLOC_CAP_SPIRAM);
      if (!evtRing) { EVT_CAP = 1500; evtRing = (Evt*)heap_caps_malloc((size_t)EVT_CAP * sizeof(Evt), MALLOC_CAP_8BIT); }
      if (!evtRing) { EVT_CAP = 600;  evtRing = (Evt*)malloc((size_t)EVT_CAP * sizeof(Evt)); }
    }
    rawEdgeQ = xQueueCreate(256, sizeof(RawEdge));
    Serial.println("[mach] GPIO MAP (" + String(machineLabel) + ")  ring=" + String(EVT_CAP) + " seq=" + String(gSeq));
    for (int i = 0; i < NUM_SIGNALS; i++) {
      // Pull toward IDLE so an unconnected/idle input can't float: active-HIGH boards pull DOWN (idle=LOW);
      // machine 7 (active-LOW) pulls UP (idle=HIGH). Kills the M1/M7 floating-input phantom edges.
      pinMode(machineSignals[i].pin, sigActiveHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
      bool active = (digitalRead(machineSignals[i].pin) == (sigActiveHigh ? HIGH : LOW));
      capCommitted[i]     = active;
      capPend[i]          = -1;
      capCounter[i]       = 0;
      capNoise[i]         = 0;
      capActiveSinceUs[i] = active ? esp_timer_get_time() : 0;
      capActiveStartMs[i] = active ? millis() : 0;
      capLastChangeMs[i]  = 0;
      attachInterruptArg(digitalPinToInterrupt(machineSignals[i].pin),
                         machineSignalISR, (void*)(intptr_t)i, CHANGE);
      Serial.println("   [mach]   " + String(i + 1) + "   " + machineSignals[i].name +
                     "  gpio" + String(machineSignals[i].pin) + (sigActiveHigh ? "  HIGH(active)  now=" : "  LOW(active)  now=") +
                     String(active ? "ACTIVE" : "idle"));
    }
    injectionArmed = true;
    // Task watchdog longer than any legit POST (8 s) so it catches true hangs, not slow servers.
    esp_task_wdt_config_t twdt = { .timeout_ms = 30000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&twdt);
    xTaskCreatePinnedToCore(captureTask, "capture", 4096,  NULL, 10, &captureTaskH, 0);  // core 0
    xTaskCreatePinnedToCore(senderTask,  "sender",  12288, NULL, 5,  &senderTaskH,  1);  // core 1
    Serial.println("[mach] ARMED " + String(machineLabel) + " " + String(NUM_SIGNALS) + (sigActiveHigh ? " active-high" : " active-low") + " inputs, shots=" + String(injectionShotCount) + " ring=" + String(EVT_CAP));
  } else {
    Serial.println("[mach] NOT armed - board #" + String(boardNum) + " is not a monitored machine (dormant)");
  }

  Serial.println("========================================");

  lastHeartbeatAt = millis() - HEARTBEAT_INTERVAL;
  lastGoodContactAt = millis();          // grace period before the offline watchdog can fire
}

void loop() {
  unsigned long now = millis();

  // Offline watchdog: if the board can't reach the server for OFFLINE_REBOOT_TIMEOUT,
  // it's stuck (wedged WiFi/TCP) -> hard reboot to force a clean reconnect.
  // Only reboot if we HAD contact this boot and then lost it for 60s (a wedge).
  // If we never connected (no network here), don't reboot-loop - just keep trying.
  if (everConnected && now - lastGoodContactAt > OFFLINE_REBOOT_TIMEOUT) {
    Serial.println("[watchdog] lost server contact for 60s - rebooting to recover");
    delay(50);
    ESP.restart();
  }

  // Reader-stuck watchdog: if this board HAS read a tag this boot but the reader has now read nothing at
  // all for READER_STUCK_REBOOT_MS (no UID, not even a bare field answer -> lastSeenAt stops advancing),
  // the reader has wedged in the way that only a reboot clears - the same recovery you see when "it works
  // again after a software update". Reboot to do that automatically. everSawTag gates it (a spare/empty
  // reader never triggers it) and clears on reboot, so a genuinely-removed tag causes at most ONE reboot.
  if (everSawTag && now - lastSeenAt > READER_STUCK_REBOOT_MS) {
    Serial.println("[watchdog] reader read nothing for " + String(READER_STUCK_REBOOT_MS / 1000) +
                   "s after holding a tag - rebooting to recover the reader");
    delay(50);
    ESP.restart();
  }

  if (now - lastWifiCheckAt >= WIFI_CHECK_INTERVAL) {
    lastWifiCheckAt = now;
    if (WiFi.status() != WL_CONNECTED) {
      wifiUp = false;
      Serial.println("[wifi] reconnecting...");
      wifiMulti.run();
    } else if (!wifiUp) {
      wifiUp = true;
      Serial.println("[wifi] back up - IP " + WiFi.localIP().toString());
      lastHeartbeatAt = now - HEARTBEAT_INTERVAL;
    }
  }

  ntpService();                       // non-blocking: set/maintain the real UTC clock + boot anchor

  if (now - lastHeartbeatAt >= HEARTBEAT_INTERVAL) {
    lastHeartbeatAt = now;
    sendHeartbeat();
  }

  if (now - lastReaderCheckAt >= READER_CHECK_INTERVAL) {
    lastReaderCheckAt = now;
    serviceReader();
  }

  if (now - lastOtaAt >= OTA_CHECK_INTERVAL) {
    checkOTA();                 // run the check...
    lastOtaAt = millis();       // ...then stamp the timer on RETURN (success or failure), per the OTA spec
  }

  if (now - lastRfidPollAt >= RFID_POLL_INTERVAL) {
    lastRfidPollAt = now;
    pollRfid();
  }

  if (now - lastStatusAt >= STATUS_INTERVAL) {
    lastStatusAt = now;
    printStatus();
  }

  // Machine capture + sending now run in dedicated tasks (captureTask core 0, senderTask core 1); the loop
  // never touches the event path. Here we only persist seq + shot count (so they survive a power cut) and
  // run the OTA validation/rollback gate.
  if (injectionArmed && gSeq - lastSeqSaved >= 50) {
    prefs.begin("seqs", false); prefs.putUInt("seq", gSeq); prefs.end();
    lastSeqSaved = gSeq;
  }
  if (injectionArmed) saveShotCount();
  otaValidateTick();
}
