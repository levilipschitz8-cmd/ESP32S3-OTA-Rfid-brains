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

#define FW_VERSION "1.0.16"

const char* HEARTBEAT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-heartbeat";
const char* TAG_EVENT_URL   = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-tag-event";
const char* CMD_RESULT_URL  = "https://cofrgojpwdyzfhfqnlch.supabase.co/functions/v1/device-command-result";
const char* FW_VERSION_URL  = "https://raw.githubusercontent.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/main/fw-version.txt";
const char* FIRMWARE_URL    = "https://github.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/releases/latest/download/firmware.bin";

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
String currentUID = "";
unsigned long lastSeenAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastRfidPollAt = 0;
unsigned long lastWifiCheckAt = 0;
unsigned long lastOtaAt = 0;
unsigned long lastReaderCheckAt = 0;
unsigned long lastGoodContactAt = 0;   // last successful (200) heartbeat

const unsigned long HEARTBEAT_INTERVAL   = 5000;
const unsigned long RFID_POLL_INTERVAL   = 250;
const unsigned long WIFI_CHECK_INTERVAL  = 5000;
const unsigned long OTA_CHECK_INTERVAL   = 60000;    // 60s
const unsigned long READER_CHECK_INTERVAL = 3000;    // hot-plug re-check
const unsigned long OFFLINE_REBOOT_TIMEOUT = 60000;  // no server contact for 60s -> self-reboot
const unsigned long REMOVE_TIMEOUT       = 1000;     // ~1s: report tag removed quickly (glitch-filtered)

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

  // Explicit hardware reset pulse on RST. Some ESP32-S3 boards leave the RC522 in
  // a half-initialised state where it answers over SPI (so it looks "detected")
  // but its RF receiver never comes up -> the reader can't actually read a tag.
  // A clean low->high pulse forces a full power-on reset before we init. This is
  // the difference that makes one board read and an identically-wired one not.
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(10);
  digitalWrite(RST_PIN, HIGH);
  delay(50);

  mfrc522.PCD_Init();
  delay(50);
  byte v = readReaderVersion();
  rc522Ok = (v != 0x00 && v != 0xFF);
  if (rc522Ok) {
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);        // max receive sensitivity (48 dB) - proven on working board
    mfrc522.PCD_AntennaOn();
    // Restore the strong antenna drive that the working board reads with.
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x27 << 1), 0xFF); // GsNReg  (CW+Mod N-driver = max)
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x28 << 1), 0x3F); // CWGsPReg  (P-driver CW = max)
    mfrc522.PCD_WriteRegister((MFRC522::PCD_Register)(0x29 << 1), 0x3F); // ModGsPReg (P-driver Mod = max)
    Serial.println("[rc522] ready (v0x" + String(v, HEX) + ") - hw-reset + max RF");
  } else {
    Serial.println("[rc522] NOT DETECTED (v0x" + String(v, HEX) + ") - heartbeat continues anyway");
  }
}

// Hot-plug: detect an RC522 connected AFTER boot, and recover from cable glitches.
void serviceReader() {
  byte v = readReaderVersion();
  bool present = (v != 0x00 && v != 0xFF);
  if (present && !rc522Ok) {
    startRC522();                         // reader just appeared -> initialize it
  } else if (!present && rc522Ok) {
    rc522Ok = false;
    Serial.println("[rc522] reader lost");
  }
}

// These ESP32-S3 boards drop the RC522 antenna (TxControlReg RF-enable bits) back to
// OFF after every RF operation, so the reader goes blind between ops (reads a tag, then
// can't see it -> false "removed"). Force the field on before each and every op.
void ensureAntennaOn() {
  byte tx = mfrc522.PCD_ReadRegister(mfrc522.TxControlReg);
  if ((tx & 0x03) == 0x03) return;                       // already on
  mfrc522.PCD_WriteRegister(mfrc522.TxControlReg, tx | 0x03);
  if ((mfrc522.PCD_ReadRegister(mfrc522.TxControlReg) & 0x03) == 0x03) return;  // stuck on
  // Write refused -> reader wedged. Full reset recovery, then force on.
  mfrc522.PCD_Reset();
  delay(50);
  mfrc522.PCD_Init();
  mfrc522.PCD_AntennaOn();
  mfrc522.PCD_WriteRegister(mfrc522.TxControlReg, 0x83);
}

bool isTagStillPresent() {
  byte bufferATQA[2];
  byte bufferSize;
  // Retry: on a long/noisy cable a single check can glitch even with the tag present.
  // Only if all attempts fail do we treat the tag as gone -> no false "removed".
  for (int attempt = 0; attempt < 3; attempt++) {
    ensureAntennaOn();                       // reader drops the field between ops -> re-force
    mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
    bufferSize = sizeof(bufferATQA);
    MFRC522::StatusCode result = mfrc522.PICC_WakeupA(bufferATQA, &bufferSize);
    if (result == MFRC522::STATUS_OK || result == MFRC522::STATUS_COLLISION) return true;
  }
  return false;
}

void pollRfid() {
  if (!rc522Ok) return;
  if (currentUID != "") {
    if (isTagStillPresent()) {
      lastSeenAt = millis();
    } else if (millis() - lastSeenAt > REMOVE_TIMEOUT) {
      Serial.println("Tag removed: " + currentUID);
      postTagEvent(currentUID, "removed");
      currentUID = "";
      mfrc522.PCD_Init();
    }
    return;
  }
  // Retry a few times per scan: over a long cable a faint tag often fails the first
  // attempt. This board drops the antenna after each op, so re-force it every attempt.
  bool found = false;
  for (int attempt = 0; attempt < 5 && !found; attempt++) {
    ensureAntennaOn();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) found = true;
  }
  if (found) {
    currentUID = uidToString(mfrc522.uid);
    lastSeenAt = millis();
    Serial.println("Tag present: " + currentUID);
    postTagEvent(currentUID, "present");
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
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
}
