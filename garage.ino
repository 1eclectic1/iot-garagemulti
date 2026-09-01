// garage.ino – jvlib (ESP8266 D1 mini)
// Door + relay + DHT + OneWire  (BME removed – hardware dead)
// Air: 2 min  |  Door: on change + 10 min retained heartbeat

#define mainver "0.04"
#define me "GD01"
#define LOG_LEVEL LOG_INFO

// Sensors (no BME)
#define DHTPIN D3
#define OW_PIN1 D5
#define i2cdata D6
#define i2cclock D7

#define MQTT_PUBLISH_TOPIC "garage/SN01"

#include <jvlib.h>
#include <ArduinoJson.h>

static const int PIN_RELAY = D1;
static const int PIN_REED  = D2;

static const unsigned long RELAY_PULSE_MS     = 1000;
static const unsigned long DOOR_SAMPLE_MS    = 250;
static const unsigned long DOOR_DEBOUNCE_MS  = 40;
static const unsigned long DOOR_HEARTBEAT_MS = 10UL * 60UL * 1000UL;
static const unsigned long AIR_CYCLE_MS      = 2UL * 60UL * 1000UL;

static int doorRaw = HIGH;
static int doorStable = HIGH;
static int doorPublished = -1;
static unsigned long doorLastChangeMs = 0;
static unsigned long doorLastPubMs = 0;
static unsigned long relayBusyUntil = 0;

static int doorStateCode() {
  // INPUT_PULLUP: LOW = closed (magnet), HIGH = open — flip if inverted
  return (doorStable == LOW) ? 1 : 2;
}

static void pulseRelay() {
  if (millis() < relayBusyUntil) {
    LOG_INFO("Relay busy, ignore activate");
    return;
  }
  LOG_INFO("Door activate pulse %lu ms", RELAY_PULSE_MS);
  digitalWrite(PIN_RELAY, HIGH);
  delay(RELAY_PULSE_MS);
  digitalWrite(PIN_RELAY, LOW);
  relayBusyUntil = millis() + RELAY_PULSE_MS + 500;
}

static void publishDoor(bool /*heartbeat*/) {
  StaticJsonDocument<256> doc;
  doc["name"] = me;
  doc["id"] = jv::deviceId();
  doc["01DoorState"] = doorStateCode();
  doc["IP"] = jv::ip();
  doc["Ver"] = jv::version();

  char buf[256];
  serializeJson(doc, buf, sizeof(buf));
  bool ok = jv::publishRaw("garage/GD01", buf, true);
  LOG_INFO("Door publish state=%d → %s", doorStateCode(), ok ? "OK" : "FAIL");
  doorPublished = doorStateCode();
  doorLastPubMs = millis();
}

static void serviceDoor() {
  static unsigned long lastSample = 0;
  unsigned long now = millis();
  if (now - lastSample < DOOR_SAMPLE_MS) return;
  lastSample = now;

  int raw = digitalRead(PIN_REED);
  if (raw != doorRaw) {
    doorRaw = raw;
    doorLastChangeMs = now;
  }
  if ((now - doorLastChangeMs) >= DOOR_DEBOUNCE_MS && doorStable != doorRaw) {
    doorStable = doorRaw;
    LOG_INFO("Door changed → code %d", doorStateCode());
    publishDoor(false);
  }

  if (now - doorLastPubMs >= DOOR_HEARTBEAT_MS) {
    publishDoor(true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "garage/GD02") != 0) return;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) {
    LOG_WARN("GD02 JSON parse failed");
    return;
  }

  const char* n = doc["name"] | "";
  const char* act = doc["activate"] | "";
  if (strcmp(n, "GD02") == 0 && strcmp(act, "YES") == 0) {
    pulseRelay();
    delay(50);
    serviceDoor();
  }
}

void publishAir() {
  jv::readSensors();

  StaticJsonDocument<2048> doc;
  doc["name"] = "SN01";
  doc["id"]   = jv::deviceId();

#if OW_PIN1 >= 0
  for (int i = 0; i < publishedSensorCount; i++) {
    if (!isnan(publishedTemps[i])) {
      char key[20];
      sprintf(key, "%02X%02X%02X%02X%02X%02X%02X%02X",
              publishedAddrs[i][0], publishedAddrs[i][1],
              publishedAddrs[i][2], publishedAddrs[i][3],
              publishedAddrs[i][4], publishedAddrs[i][5],
              publishedAddrs[i][6], publishedAddrs[i][7]);
      doc[key] = publishedTemps[i];
    }
  }
#endif

  // Primary garage air keys from DHT (BME removed)
#if DHTPIN >= 0
  if (dhttemp > -900.0) {
    doc["01temp"]     = dhttemp;
    doc["01humidity"] = dhthum;
    doc["01dewpoint"] = dhtdew;
    doc["01dhtt"]     = dhttemp;
    doc["01dhth"]     = dhthum;
    doc["01dhtd"]     = dhtdew;
    doc["01dhthi"]    = dhthi;
    if (dhthi > -900.0 && dhthum >= 30.0 && dhttemp >= 70.0) {
      doc["01heatindex"] = dhthi;
    }
  }
#endif

  doc["IP"]   = jv::ip();
  doc["RSSI"] = jv::rssi();
  doc["Ver"]  = jv::version();

  char buf[1536];
  serializeJson(doc, buf, sizeof(buf));
  bool ok = jv::publishRaw("garage/SN01", buf, false);
  LOG_INFO("Air publish → %s", ok ? "OK" : "FAIL");
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_REED, INPUT_PULLUP);

  jv::begin();
  jvSensorsBegin();
  jv::setCallback(mqttCallback);
  jv::subscribe("garage/GD02");
  jvWatchdogSetup();
  jvDailyRebootSetup();

  doorRaw = doorStable = digitalRead(PIN_REED);
  LOG_INFO("Garage GD01/SN01 starting (DHT+OW, no BME)");

  publishDoor(true);
  publishAir();
}

void loop() {
  jv::loop();
  jvWatchdogFeed();
  jvCheckDailyReboot();
  serviceDoor();

  static unsigned long lastAir = 0;
  if (millis() - lastAir >= AIR_CYCLE_MS) {
    lastAir = millis();
    LOG_INFO("Air measurement cycle");
    publishAir();
  }

  delay(10);
}
