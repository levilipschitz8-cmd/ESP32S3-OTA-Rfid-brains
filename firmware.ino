#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Preferences.h>

// ====== IDENTITY ======
// OTA / generic build: leave these BLANK so each board keeps the identity
// already stored in its flash. Only fill them in for a first USB provisioning flash.
#define BOARD_NUM 0
const char* BOARD_CODE = "";
const char* DEVICE_ID  = "";
const char* DEVICE_KEY = "";
// ======================

#define FW_VERSION "1.0.70"

const char* HEARTBEAT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-heartbeat";
const char* TAG_EVENT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-tag-event";
const char* CMD_RESULT_URL  = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-command-result";
const char* FW_VERSION_URL  = "https://raw.githubusercontent.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/main/fw-version.txt";
const char* FIRMWARE_URL    = "https://github.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/releases/latest/download/firmware.bin";
const char* MACHINE_EVENT_URL = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-machine-event";
const char* INJECTION_DEVICE_ID = "ec089dbd-7269-4c1d-86b8-e425400f80bf";  // machine 7 (board 7): injection monitor

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
// ---- Injection-machine signal monitoring (machine 7 ONLY; dormant on every other board) ----
// Seven PC817-isolated signals from the moulding machine. Confirmed ACTIVE-LOW (GPIO 6 proved it:
// pulled LOW when the mould opened, released HIGH). So: INPUT_PULLUP on every channel, and a pin is
// "active" when it reads LOW, "idle" when it reads HIGH. An unconnected/idle input rests HIGH via the
// pull-up = idle (no false events); a real signal sinks it LOW = active. The seven struct entries are
// the SINGLE source of truth for BOTH the event path and the raw_levels string, so a signal name can
// never drift relative to its GPIO. GPIO 5 (terminal 112) is a wired spare, deliberately not listed.
bool injectionArmed = false;
struct MachineSignal { byte pin; const char* name; bool isInjection; };  // isInjection drives the shot counter
MachineSignal machineSignals[] = {
  {  4, "injection",        true  },   // raw_levels pos 1 - terminal 105
  {  6, "mould_opening",    false },   // raw_levels pos 2 - terminal 109  (confirmed working)
  {  7, "ejecting_forward", false },   // raw_levels pos 3 - terminal 110
  { 15, "carriage_forward", false },   // raw_levels pos 4 - terminal 104
  { 21, "charge",           false },   // raw_levels pos 5 - terminal 106  (plasticizing / screw recovery); moved off GPIO16
  { 40, "carriage_back",    false },   // raw_levels pos 6 - terminal 108
  { 41, "ejecting_back",    false },   // raw_levels pos 7 - terminal 111
};
const int NUM_SIGNALS = sizeof(machineSignals) / sizeof(machineSignals[0]);
volatile bool          sigState[7];                 // current debounced state per channel (true = active = LOW)
volatile bool          sigChanged[7];               // edge occurred -> loop posts and clears
volatile unsigned long sigLastEdgeUs[7];            // last accepted edge time (us) for per-channel debounce
volatile unsigned long injectionShotCount = 0;      // shots (injection idle->active edges), persisted to NVS
unsigned long          lastShotSaved = 0;           // last shot value written to NVS (throttle flash writes)
const unsigned long INJECTION_DEBOUNCE_US = 40000;  // 40 ms debounce per channel
String currentUID = "";
byte lastReaderVersion = 0;             // last RC522 version byte read (for the status line)
unsigned long lastSeenAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastRfidPollAt = 0;
unsigned long lastWifiCheckAt = 0;
unsigned long lastOtaAt = 0;
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

const unsigned long HEARTBEAT_INTERVAL   = 5000;
const unsigned long RFID_POLL_INTERVAL   = 250;
const unsigned long WIFI_CHECK_INTERVAL  = 5000;
const unsigned long LEVEL_SNAP_INTERVAL  = 15000;    // machine 7: raw pin-level snapshot period (diagnostic)
const unsigned long OTA_CHECK_INTERVAL   = 60000;    // 60s
const unsigned long READER_CHECK_INTERVAL = 3000;    // hot-plug re-check
const unsigned long STATUS_INTERVAL      = 2000;     // print a clear per-board status line this often
const unsigned long CLOSE_PROBE_INTERVAL = 1000;     // how often (max) to dip to low power for a close tag
const unsigned long FIELD_REFRESH_INTERVAL = 20000;  // while a tag is present, proactively blink the field
                                                     // this often so it never sits in a continuous field
                                                     // long enough to go unresponsive (prevents sticking).
const unsigned long OFFLINE_REBOOT_TIMEOUT = 60000;  // no server contact for 60s -> self-reboot
const unsigned long REMOVE_CONFIRM_MS    = 20000;    // A HEALTHY reader must see a genuinely EMPTY field
                                                     // this long, CONTINUOUSLY, before a tracked tag is
                                                     // declared removed. A tag physically on the reader
                                                     // answers the field ping every poll and resets this,
                                                     // so it stays "present" indefinitely (months). A
                                                     // wedged/down reader never counts toward removal - it
                                                     // holds the tag and recovers. Real removal still
                                                     // clears within ~20s of the field actually going empty.
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

bool writeNdefTag(const String& url, const String& text, String& detail, String& readBack) {
  if (!rc522Ok) { detail = "reader not detected"; return false; }
  if (!selectTagWithin(2000)) { detail = "no tag on reader (2s scan)"; return false; }
  String ttype = tagTypeName(mfrc522.uid.sak);
  if (mfrc522.uid.sak != 0x00) {                        // Classic goes through write_tag, not here
    detail = "not an NTAG (type=" + ttype + ", sak=0x" + String(mfrc522.uid.sak, HEX) + ")";
    mfrc522.PICC_HaltA(); return false;
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
  for (int pg = 0; pg < pages; pg++) {
    byte page[4] = { tlv[pg*4], tlv[pg*4+1], tlv[pg*4+2], tlv[pg*4+3] };
    if (mfrc522.MIFARE_Ultralight_Write((byte)(4 + pg), page, 4) != MFRC522::STATUS_OK) {
      detail = "ntag write failed at page " + String(4 + pg) + " (type=" + ttype + ")";
      mfrc522.PICC_HaltA(); return false;
    }
  }
  byte back[256]; int backLen = 0;
  for (int pg = 0; pg < pages + 2 && backLen + 16 <= (int)sizeof(back); pg += 4) {
    byte b[18]; byte bl = sizeof(b);
    if (mfrc522.MIFARE_Read((byte)(4 + pg), b, &bl) != MFRC522::STATUS_OK) break;
    for (int k = 0; k < 16; k++) back[backLen++] = b[k];
  }
  mfrc522.PICC_HaltA();
  readBack = decodeNdefText(back, backLen);
  detail = "wrote+verified NDEF (" + String(pages) + " pages, type=" + ttype + ")";
  return true;
}

void postCommandResultNdef(const String& cmdId, const String& status, const String& detail, const String& readBack) {
  String resp;
  String body = "{\"command_id\":\"" + cmdId + "\",\"status\":\"" + status +
                "\",\"detail\":\"" + jsonEscape(detail) + "\",\"read_back\":\"" + jsonEscape(readBack) + "\"}";
  int code = postJson(CMD_RESULT_URL, body, resp);
  Serial.println("[cmd] ndef " + status + " read_back='" + readBack + "' -> code=" + String(code));
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
  String body = String("{\"firmware_version\":\"") + FW_VERSION +
                "\",\"ip\":\"" + ip +
                "\",\"rssi\":" + String(rssi) +
                ",\"board_code\":\"" + boardCode + "\"" +
                ",\"reader_ok\":" + (rc522Ok ? "true" : "false") +           // reader detected & configured
                ",\"reader_ver\":\"0x" + String(lastReaderVersion, HEX) + "\"" +  // RC522 version byte
                ",\"tag\":\"" + currentUID + "\"" +                          // UID currently held ("" = none)
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

// Per-channel edge interrupt (machine 7 only): ACTIVE-LOW, debounced, minimal, in IRAM. The channel
// index is passed as the ISR arg so all seven signals share one handler. Interrupt (not 10 ms polling)
// so an edge is still captured even while the loop is stalled in a blocking HTTP post - strictly better
// than polling, which would miss an edge that falls inside a network call. On the injection channel a
// clean idle(HIGH)->active(LOW) edge counts exactly one shot (never repeats while it stays LOW).
void IRAM_ATTR machineSignalISR(void* arg) {
  int i = (int)(intptr_t)arg;
  unsigned long now = micros();
  if (now - sigLastEdgeUs[i] < INJECTION_DEBOUNCE_US) return;     // debounce contact bounce
  sigLastEdgeUs[i] = now;
  bool active = (digitalRead(machineSignals[i].pin) == LOW);      // ACTIVE-LOW: LOW = active, HIGH = idle
  if (active != sigState[i]) {
    sigState[i] = active;
    if (active && machineSignals[i].isInjection) injectionShotCount++;  // idle->active edge = one shot
    sigChanged[i] = true;
  }
}

// POST one machine signal state change to Supabase (machine 7). Reuses the SAME X-Device-Id/
// X-Device-Key auth as the heartbeat. Server stamps the time (created_at); we send which signal,
// its new state, the running shot count and uptime.
void postMachineSignal(const char* name, bool active, unsigned long shots) {
  String resp;
  String body = String("{\"signal\":\"") + name +
                "\",\"state\":\"" + (active ? "active" : "idle") +
                "\",\"shot_count\":" + String(shots) +
                ",\"uptime_ms\":" + String(millis()) + "}";
  int code = postJson(MACHINE_EVENT_URL, body, resp);
  Serial.println("[mach] " + String(name) + " " + String(active ? "ACTIVE" : "idle") +
                 " shot#" + String(shots) + " -> code=" + String(code));
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
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  if (!http.begin(client, FW_VERSION_URL)) return;
  int code = http.GET();
  if (code == 200) {
    String body = http.getString(); body.trim();
    int nl = body.indexOf('\n');
    String remoteVer = (nl < 0) ? body : body.substring(0, nl);
    String target    = (nl < 0) ? "all" : body.substring(nl + 1);
    remoteVer.trim(); target.trim();
    bool targeted = (target == "all" || target == String(boardNum) || target == deviceId);
    if (targeted && versionNewer(remoteVer, FW_VERSION)) {
      Serial.println("[ota] " + String(FW_VERSION) + " -> " + remoteVer + " ... downloading");
      http.end();
      if (injectionArmed)                                          // no ISR during flash write
        for (int i = 0; i < NUM_SIGNALS; i++) detachInterrupt(digitalPinToInterrupt(machineSignals[i].pin));
      WiFiClientSecure up; up.setInsecure();
      httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
      t_httpUpdate_return r = httpUpdate.update(up, FIRMWARE_URL);
      if (r == HTTP_UPDATE_FAILED)
        Serial.println("[ota] FAILED: " + String(httpUpdate.getLastError()) + " " + httpUpdate.getLastErrorString());
      return;
    }
  }
  http.end();
}

// Read the RC522 version register with retries (long cables give occasional bad reads).
byte readReaderVersion() {
  byte v = 0;
  for (int i = 0; i < 4; i++) {
    v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    if (v != 0x00 && v != 0xFF) break;
    delay(5);
  }
  return v;
}

void startRC522() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  SPI.setFrequency(50000);               // 50 kHz: maximum tolerance for very long (~3 m) cables
  for (byte i = 0; i < 6; i++) mifareKey.keyByte[i] = 0xFF;
  // Hardware reset pulse on RST before init. A reader that has WEDGED (locked up, reads v0x0)
  // is un-stuck far more reliably by a real power-on reset than by a soft init - this is what
  // lets a wedged reader recover on re-init instead of staying dead until a manual replug.
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(10);
  digitalWrite(RST_PIN, HIGH);
  delay(50);
  mfrc522.PCD_Init();
  delay(50);
  byte v = readReaderVersion();
  lastReaderVersion = v;
  rc522Ok = (v != 0x00 && v != 0xFF);
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
  bool present = (v != 0x00 && v != 0xFF);
  if (present) {
    missCount = 0;
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
              String(digitalRead(machineSignals[i].pin) == LOW ? "ACT" : "idle");  // active-LOW
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
  currentUID = newUid; lastSeenAt = millis(); emptySince = 0;
  postTagEvent(currentUID, "present");
}

// Read the tag UID with a few tries; "" if nothing answered.
String readTagUid() {
  String u = "";
  for (int a = 0; a < 5 && u == ""; a++) u = tryReadOnce();
  return u;
}

void pollRfid() {
  // ---- Holding a tracked tag: bias HEAVILY toward "still present" ----
  // A tag physically on the reader must NEVER falsely report "removed", even after months. Removal is
  // accepted ONLY when a HEALTHY reader confirms a genuinely empty field, continuously, for
  // REMOVE_CONFIRM_MS. A wedged/down reader can't judge presence, so it holds the tag and lets
  // serviceReader recover it. A weakly-coupled tag that only answers a bare field ping still counts as
  // present. A different UID appearing is handled as a swap (removed old + present new).
  if (currentUID != "") {
    if (!rc522Ok) { emptySince = 0; return; }          // reader down -> hold, recover in background

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
    // A weakly-coupled tag may still answer a bare field ping even when it can't complete a select.
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
  }
  if (uid != "") {
    currentUID = uid;
    lastSeenAt = millis();
    Serial.println("Tag present: " + currentUID);
    postTagEvent(currentUID, "present");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  resolveIdentity();

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
  if (boardNum == 7 || deviceId == INJECTION_DEVICE_ID) {
    // Restore the persisted shot count so a reboot doesn't reset it.
    prefs.begin("machine", true);
    injectionShotCount = prefs.getULong("shots", 0);
    prefs.end();
    lastShotSaved = injectionShotCount;
    // Startup self-test: print the GPIO map only. It does NOT post events or touch the shot count.
    Serial.println("[mach] GPIO MAP (machine 7)  pos  signal            gpio  active-level");
    for (int i = 0; i < NUM_SIGNALS; i++) {
      pinMode(machineSignals[i].pin, INPUT_PULLUP);
      sigState[i]      = (digitalRead(machineSignals[i].pin) == LOW);   // seed with the live level (LOW=active)
      sigChanged[i]    = false;
      sigLastEdgeUs[i] = 0;
      attachInterruptArg(digitalPinToInterrupt(machineSignals[i].pin),
                         machineSignalISR, (void*)(intptr_t)i, CHANGE);
      Serial.println("   [mach]   " + String(i + 1) + "   " + machineSignals[i].name +
                     "  gpio" + String(machineSignals[i].pin) + "  LOW(active)  now=" +
                     String(digitalRead(machineSignals[i].pin) == LOW ? "ACTIVE" : "idle"));
    }
    injectionArmed = true;
    Serial.println("[mach] ARMED " + String(NUM_SIGNALS) + " active-low inputs, shots restored=" + String(injectionShotCount));
  } else {
    Serial.println("[mach] NOT armed - board #" + String(boardNum) + " is not machine 7 (dormant)");
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

  if (now - lastHeartbeatAt >= HEARTBEAT_INTERVAL) {
    lastHeartbeatAt = now;
    sendHeartbeat();
  }

  if (now - lastReaderCheckAt >= READER_CHECK_INTERVAL) {
    lastReaderCheckAt = now;
    serviceReader();
  }

  if (now - lastOtaAt >= OTA_CHECK_INTERVAL) {
    lastOtaAt = now;
    checkOTA();
  }

  if (now - lastRfidPollAt >= RFID_POLL_INTERVAL) {
    lastRfidPollAt = now;
    pollRfid();
  }

  if (now - lastStatusAt >= STATUS_INTERVAL) {
    lastStatusAt = now;
    printStatus();
  }

  // Machine signal changes (machine 7 only): each channel's ISR flags it; post each promptly and
  // persist the shot count. The ISR captured the edge even if this post blocks, so none are lost.
  if (injectionArmed) {
    for (int i = 0; i < NUM_SIGNALS; i++) {
      if (!sigChanged[i]) continue;
      noInterrupts();
      bool active = sigState[i];
      unsigned long shots = injectionShotCount;
      sigChanged[i] = false;
      interrupts();
      Serial.println("[edge] " + String(machineSignals[i].name) + " gpio" + String(machineSignals[i].pin) +
                     " -> " + String(active ? "ACTIVE(LOW)" : "idle(HIGH)") + " t=" + String(millis()));
      postMachineSignal(machineSignals[i].name, active, shots);
      if (machineSignals[i].isInjection) saveShotCount();
    }
  }

  // 1 Hz RAW serial diagnostic (machine 7): the literal level of all 7 pins, once a second, to serial.
  if (injectionArmed && now - lastRawPrintAt >= 1000) {
    lastRawPrintAt = now;
    String lv = "";
    for (int i = 0; i < NUM_SIGNALS; i++) lv += (digitalRead(machineSignals[i].pin) == HIGH) ? "H" : "L";
    Serial.println("RAW " + lv + "  (inj,mould,ejF,carF,charge,carB,ejB)");
  }

  // Raw pin-level snapshot to the backend (machine 7): every LEVEL_SNAP_INTERVAL post the ACTUAL HIGH/
  // LOW level of all 7 pins straight from digitalRead - bypassing the interrupt/edge path entirely.
  // One row: signal="raw_levels", state = 7 chars (H/L) in machineSignals order. If a char flips while
  // the machine cycles but no active/idle event fires -> detection bug; if a char never flips while its
  // function cycles -> that pin is electrically static (tap point / wiring). Ground truth on the tablet.
  if (injectionArmed && now - lastLevelSnapAt >= LEVEL_SNAP_INTERVAL) {
    lastLevelSnapAt = now;
    String lv = "";
    for (int i = 0; i < NUM_SIGNALS; i++)
      lv += (digitalRead(machineSignals[i].pin) == HIGH) ? "H" : "L";
    String resp;
    String body = String("{\"signal\":\"raw_levels\",\"state\":\"") + lv +
                  "\",\"shot_count\":" + String(injectionShotCount) +
                  ",\"uptime_ms\":" + String(millis()) + "}";
    postJson(MACHINE_EVENT_URL, body, resp);
  }
}
