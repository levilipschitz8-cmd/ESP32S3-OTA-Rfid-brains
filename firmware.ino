#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <time.h>

// ====== FIRST USB FLASH OF A BOARD: set 1, 2, 3...
// ====== OTA RELEASE BUILDS: set 0 (each board keeps its stored number)
#define BOARD_NUM 1
// ===============================================================

#define FW_VERSION "1.0.1"

const char* SUPABASE_URL      = "https://cofrgojpwdyzfhfqnlch.supabase.co";
const char* SUPABASE_ANON_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImNvZnJnb2pwd2R5emZoZnFubGNoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Nzc0Mzg0MjIsImV4cCI6MjA5MzAxNDQyMn0.fc1dOwvHJEhRDDnfOzj4EpqhoN9qE2CWf9T8995yoBM";

const char* IDENTITY_URL = "https://raw.githubusercontent.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/main/identities.txt";
const char* VERSION_URL  = "https://raw.githubusercontent.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/main/version.txt";
const char* FIRMWARE_URL = "https://github.com/levilipschitz8-cmd/ESP32S3-OTA-Rfid-brains/releases/latest/download/firmware.bin";

#define SCK_PIN  12
#define MISO_PIN 13
#define MOSI_PIN 11
#define SS_PIN   18
#define RST_PIN  17

WiFiMulti wifiMulti;
MFRC522 mfrc522(SS_PIN, RST_PIN);
Preferences prefs;

int boardNum = 0;
String deviceId = "";
String currentUID = "";
unsigned long lastSeenAt = 0;
unsigned long lastPresentPostAt = 0;
unsigned long lastHeartbeatAt = 0;
unsigned long lastOtaCheckAt = 0;
unsigned long lastIdentityCheckAt = 0;

const unsigned long REMOVE_TIMEOUT      = 15000;
const unsigned long PRESENT_REFRESH     = 8000;
const unsigned long HEARTBEAT_INTERVAL  = 6000;
const unsigned long OTA_CHECK_INTERVAL  = 300000;   // 5 min
const unsigned long IDENTITY_INTERVAL   = 600000;   // 10 min

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

void fetchIdentity() {
  if (boardNum == 0) return;
  if (wifiMulti.run() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(IDENTITY_URL);
  if (http.GET() == 200) {
    String body = http.getString();
    String prefix = String(boardNum) + "=";
    int start = 0;
    while (start < body.length()) {
      int end = body.indexOf('\n', start);
      if (end < 0) end = body.length();
      String line = body.substring(start, end);
      line.trim();
      if (line.startsWith(prefix)) {
        String newId = line.substring(prefix.length());
        newId.trim();
        if (newId.length() > 0 && newId != deviceId) {
          deviceId = newId;
          prefs.putString("devid", deviceId);
          Serial.println("Identity updated: " + deviceId);
        }
        break;
      }
      start = end + 1;
    }
  }
  http.end();
}

void checkOTA() {
  if (wifiMulti.run() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(VERSION_URL);
  if (http.GET() == 200) {
    String body = http.getString();
    body.trim();
    int nl = body.indexOf('\n');
    String remoteVer = (nl < 0) ? body : body.substring(0, nl);
    String target    = (nl < 0) ? "all" : body.substring(nl + 1);
    remoteVer.trim(); target.trim();
    if (remoteVer != FW_VERSION &&
        (target == "all" || target == String(boardNum) || target == deviceId)) {
      Serial.println("OTA: " + String(FW_VERSION) + " -> " + remoteVer);
      http.end();
      WiFiClientSecure client;
      client.setInsecure();
      httpUpdate.update(client, FIRMWARE_URL);
      return;
    }
  }
  http.end();
}

void postEvent(const String& uid, const String& eventType) {
  if (deviceId == "") return;
  int retries = 0;
  while (wifiMulti.run() != WL_CONNECTED && retries < 10) {
    delay(300);
    retries++;
  }
  if (wifiMulti.run() != WL_CONNECTED) return;
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/tag_events";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  String body = "{\"device_id\":\"" + deviceId + "\","
                "\"tag_uid\":\"" + uid + "\","
                "\"event_type\":\"" + eventType + "\"}";
  int code = http.POST(body);
  Serial.println("[" + eventType + "] " + uid + " -> HTTP " + String(code));
  http.end();
}

bool timeSynced() {
  return time(nullptr) > 1600000000;   // sanity: after 2020 == NTP has synced
}

String isoNow() {
  time_t now = time(nullptr);
  struct tm t;
  gmtime_r(&now, &t);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &t);
  return String(buf);
}

void postHeartbeat() {
  if (deviceId == "" || wifiMulti.run() != WL_CONNECTED) return;
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/devices?id=eq." + deviceId;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
  http.addHeader("Prefer", "return=minimal");
  // Send the board's own flashed number so Supabase always shows the true device,
  // plus a real ISO timestamp (the old "now()" string was rejected by Postgres,
  // so last_seen never updated and every board looked offline).
  String body = "{\"board_num\":" + String(boardNum);
  if (timeSynced()) body += ",\"last_seen\":\"" + isoNow() + "\"";
  body += "}";
  int code = http.PATCH(body);
  Serial.println("[heartbeat] board " + String(boardNum) + " -> HTTP " + String(code));
  http.end();
}

void startRC522() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  SPI.setFrequency(1000000);
  delay(200);
  mfrc522.PCD_Init();
  delay(200);
  mfrc522.PCD_DumpVersionToSerial();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  Serial.println("RC522 ready");
}

bool isTagStillPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
  MFRC522::StatusCode result = mfrc522.PICC_WakeupA(bufferATQA, &bufferSize);
  return (result == MFRC522::STATUS_OK || result == MFRC522::STATUS_COLLISION);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  prefs.begin("rfid", false);
  if (BOARD_NUM > 0) prefs.putInt("boardnum", BOARD_NUM);
  boardNum = prefs.getInt("boardnum", 0);
  deviceId = prefs.getString("devid", "");
  Serial.println("Board #" + String(boardNum) + " FW " + FW_VERSION);
  Serial.println("Stored identity: " + (deviceId == "" ? "NONE" : deviceId));

  wifiMulti.addAP("Esp32Test",            "12345678");
  wifiMulti.addAP("Cottages",             "C0ttage@37");
  wifiMulti.addAP("ABSolutely Connected", "Wifi@613");

  Serial.print("Connecting to WiFi");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected: " + WiFi.SSID());
  Serial.println("IP: " + WiFi.localIP().toString());

  // Get real UTC time from NTP so heartbeats carry a valid timestamp.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  for (int i = 0; i < 20 && !timeSynced(); i++) {
    delay(250);
  }
  Serial.println(timeSynced() ? "Time synced: " + isoNow() : "Time NOT synced");

  fetchIdentity();

  delay(500);
  Serial.println("Initializing RC522...");
  startRC522();
}

void loop() {
  if (millis() - lastHeartbeatAt > HEARTBEAT_INTERVAL) {
    postHeartbeat();
    lastHeartbeatAt = millis();
  }
  if (millis() - lastOtaCheckAt > OTA_CHECK_INTERVAL) {
    checkOTA();
    lastOtaCheckAt = millis();
  }
  if (millis() - lastIdentityCheckAt > IDENTITY_INTERVAL) {
    fetchIdentity();
    lastIdentityCheckAt = millis();
  }

  if (currentUID != "") {
    if (isTagStillPresent()) {
      lastSeenAt = millis();
      if (millis() - lastPresentPostAt > PRESENT_REFRESH) {
        postEvent(currentUID, "present");
        lastPresentPostAt = millis();
      }
    } else if (millis() - lastSeenAt > REMOVE_TIMEOUT) {
      Serial.println("Tag removed: " + currentUID);
      postEvent(currentUID, "removed");
      currentUID = "";
      mfrc522.PCD_Init();
    }
    return;
  }

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = uidToString(mfrc522.uid);
    currentUID = uid;
    lastSeenAt = millis();
    lastPresentPostAt = millis();
    postEvent(uid, "present");
    Serial.println("Tag present: " + uid);
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }

  delay(200);
}
