// garage.ino – migrated to jvlib (ESP8266 D1 mini)
// Door state + relay + BME + DHT + OneWire
// Air sensors: 2 min  |  Door: on change + 10 min retained heartbeat

#define mainver "0.03"
#define me "GD01"
#define LOG_LEVEL LOG_INFO

// Sensors
#define BME
#define DHTPIN D3
#define OW_PIN1 D5
#define i2cdata D6
#define i2cclock D7

// BME SLP – ~6 m below SN02 (133 m)
#define JV_ALTITUDE_M 127.0
#define JV_BME_PRES_BIAS 0.0

#define MQTT_PUBLISH_TOPIC "garage/SN01"   // environmental (non-retain)

#include <jvlib.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Door / relay hardware
// ---------------------------------------------------------------------------
static const int PIN_RELAY = D1;
static const int PIN_REED  = D2;

static const unsigned long RELAY_PULSE_MS     = 1000;
static const unsigned long DOOR_SAMPLE_MS    = 250;
static const unsigned long DOOR_DEBOUNCE_MS  = 40;
static const unsigned long DOOR_HEARTBEAT_MS = 10UL * 60UL * 1000UL;  // 10 min
static const unsigned long AIR_CYCLE_MS      = 2UL * 60UL * 1000UL;   // 2 min network-wide

static int doorRaw = HIGH;           // INPUT_PULLUP: HIGH = open circuit
static int doorStable = HIGH;
static int doorPublished = -1;       // last value sent to MQTT
static unsigned long doorLastChangeMs = 0;
static unsigned long doorLastPubMs = 0;
static unsigned long relayBusyUntil = 0;

// Old sketch published sensorVal+1 (1 or 2). Keep for Influx compatibility.
static int doorStateCode() {
  // reed with pullup: LOW = magnet present (closed), HIGH = open — verify on your wiring
  // Matching old: digitalRead value used as 0/1 then +1 → 1 or 2
  return (doorStable == LOW) ? 1 : 2;   // adjust if your open/closed is inverted
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

static void publishDoor(bool forceRetainHeartbeat) {
  StaticJsonDocument<256> doc;
  doc["name"] = me;
  doc["id"] = jv::deviceId();
  doc["01DoorState"] = doorStateCode();
  doc["IP"] = jv::ip();
  doc["Ver"] = jv::version();

  char buf[256];
  serializeJson(doc, buf, sizeof(buf));
  bool ok = jv::publishRaw("garage/GD01", buf, true);  // always retain door state
  LOG_INFO("Door publish state=%d retain → %s", doorStateCode(), ok ? "OK" : "FAIL");
  doorPublished = doorStateCode();
  doorLastPubMs = millis();
  (void)forceRetainHeartbeat;
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

// ---------------------------------------------------------------------------
// MQTT command: garage/GD02  {"name":"GD02","activate":"YES"}
// ---------------------------------------------------------------------------
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
    // optional: re-publish door state after pulse
    delay(50);
    serviceDoor();
  }
}

// ---------------------------------------------------------------------------
// Environmental publish – classic SN01 keys, 2 min, no retain
// ---------------------------------------------------------------------------
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

#ifdef BME
  if (bmetemp > -900.0) {
    doc["01bmet"]  = bmetemp;
    doc["01bmeh"]  = bmehum;
    doc["01bmep"]  = bmepres;
    doc["01bmed"]  = bmedew;
    doc["01bmehi"] = bmehi;
    // also the short keys some flows used
    doc["01temp"]     = bmetemp;
    doc["01humidity"] = bmehum;
    doc["01pressure"] = bmepres;
    doc["01dewpoint"] = bmedew;
  }
#endif

#if DHTPIN >= 0
  if (dhttemp > -900.0) {
    doc["01dhtt"]  = dhttemp;
    doc["01dhth"]  = dhthum;
    doc["01dhtd"]  = dhtdew;
    doc["01dhthi"] = dhthi;
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

// ---------------------------------------------------------------------------
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
  LOG_INFO("Garage GD01/SN01 starting (alt %.1f m)", (double)JV_ALTITUDE_M);

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
