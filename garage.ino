#define me "GD01"
#define ONE
#define BME
#define DHT
#define DHTpin D3
#define OneWpin D5
#define i2cdata D6
#define i2cclock D7
#include <jvcommon.h>

const char* xformat = "%02X%02X%02X%02X%02X%02X%02X%02X";

String version = "0.02b2";
// 08/16/2021 version 0.01b1 begin tracking -complete code reorg and cleanup
// 08/20/2021 version 0.01b2 added sensor based conditional build #defines and logic
// 08/21/2021 version 0.01b3 fixed MQTT buffersize too small issue
// 08/21/2021 version 0.01b4 add IP address to MQTT message
// 08/21/2021 version 0.01b5 remote enable/disable OTA listen
// 05/13/2024 version 0.01b6 change IP of broker and add 3 digit preecision
// 07/22/2024 version 0.02b1 fix json prob,remove OTA crap & complete cleanup
// 09/07/2024 version 0.02b2 move common modules to separate header file/reorg code

const char* name;
char msg[400];
int sensorVal, sensorOld = 0, loopcount = 0;

// task manager variables
unsigned long t1old = 0, t1interval = 120 * 1000;  // Publish door state once every 120 seconds
unsigned long t2old = 0, t2interval = 500;         // read door state every 500 ms
unsigned long t3old = 0, t3interval = 1000;        // check for incoming every 1000 ms
unsigned long t4old = 0, t4interval = 60 * 1000;   // read DHT every 1 min
unsigned long t5old = 0, t5interval = 60 * 1000;   // read onewire evert 1 minute
unsigned long t6old = 0, t6interval = 125 * 1000;  // publish garage readings every 2 minutes
unsigned long t7old = 0, t7interval = 60 * 1000;   // read BME every 1 min

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] len = ");
  Serial.println(length);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
  payload[length] = 0;
  doc.clear();
  deserializeJson(doc, payload);
  name = doc["name"];
  if (strcmp(name, "GD02") == 0) {
    name = doc["activate"];
    if (strcmp(name, "YES") == 0) {
      digitalWrite(D1, HIGH);
      delay(1000);
      digitalWrite(D1, LOW);
    }
  }
}

void reconnect() {
  if (WiFi.status() != WL_CONNECTED) { JVWIFISetUp(); }
  // Loop until we're reconnected
  client.setBufferSize(512);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("GD01Client", "admin", "rebels1")) {
      Serial.println("connected");
      client.subscribe("garage/GD02");
      Serial.println("Subscribed");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void pubdoor() {
  reconnect();
  doc.clear();
  doc["name"] = hostname;  // Begin to build the JSON for MQTT
  client.setCallback(callback);
  doc["01DoorState"] = sensorVal + 1;
  serializeJson(doc, msg);
  client.publish("garage/GD01", msg);
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.flush();
  doc.clear();
  client.subscribe("garage/GD02");
}

void readpin() {
  sensorVal = digitalRead(D2);
  if (sensorVal != sensorOld) {
    sensorOld = sensorVal;
    loopcount = 1000;
  }
  if (loopcount > 120) {
    pubdoor();
    loopcount = 0;
  }
  loopcount++;
}

void pubth() {                          // publish data to MQTT
  if (WiFi.status() != WL_CONNECTED) {  // reconnect WiFi if necessary
    JVWIFISetUp();
  }
  String clientId = hostname;
  clientId += String(random(0xffff), HEX);
  while (!client.connected()) {
    Serial.println("MQTT connecting");
    // Try to connect
    if (client.connect(clientId.c_str(), mqttID, mqttPASS)) {
      client.setBufferSize(500);
    } else {
      delay(2000);
    }
  }
  Serial.print("MQTT connected. ClientId: ");
  Serial.println(clientId.c_str());

  doc.clear();
  doc["name"] = "SN01";  // Begin to build the JSON for MQTT
  double thi = -999.9;
  double dew = -999.9;
  double rh = 0;
  double baro = 0;
  dew = dhtdew;
  thi = dhttemp;
  rh = dhthum;
  double heatindex = feels(thi, rh);
  if (thi > -999.9) {
    thi = round(thi);
    doc["01temp"] = thi;
    doc["01humidity"] = rh;
  }
  if (rh >= 30.0 && thi >= 70.0) {
    doc["01heatindex"] = heatindex;
  }
  if (baro > 0.0) { doc["01pressure"] = baro; }
  if (dew > -999.9) { doc["01dewpoint"] = dew; }

  for (int i = 0; i < sensors; i++) {
    sprintf(msg, xformat, tsn[i][0], tsn[i][1], tsn[i][2], tsn[i][3], tsn[i][4], tsn[i][5], tsn[i][6], tsn[i][7]);
    doc[msg] = t[i];
  }
  doc["IP"] = myip;
  doc["01bmet"] = bmetemp;
  doc["01bmeh"] = bmehum;
  doc["01bmep"] = bmepres;
  doc["01bmed"] = bmedew;
  doc["01bmehi"] = bmehi;
  doc["01dhtt"] = dhttemp;
  doc["01dhth"] = dhthum;
  doc["01dhtd"] = dhtdew;
  doc["01dhthi"] = dhthi;
  doc["RSSI"] = rssi;
  doc["Ver"] = version;
  serializeJson(doc, msg);
  bool rc = client.publish("garage/SN01", msg);

  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.flush();
  doc.clear();
  client.disconnect();
  return;
}

void loop() {
  Wire.begin(); /* data, clock */
  reconnect();
  pinMode(D1, OUTPUT);
  pinMode(D2, INPUT_PULLUP);
  readdht();
  readbme();
  readone();
  readpin();
  pubth();
  while (1 == 1) {
    if ((unsigned long)(millis() - t2old) >= t2interval) {
      t2old = millis();
      readpin();
    }
    if ((unsigned long)(millis() - t3old) >= t3interval) {
      t3old = millis();
      reconnect();
      client.loop();
    }
    if ((unsigned long)(millis() - t1old) >= t1interval) {
      t1old = millis();
      pubdoor();
    }
    if ((unsigned long)(millis() - t4old) >= t4interval) {
      t4old = millis();
      readdht();
    }
    if ((unsigned long)(millis() - t5old) >= t5interval) {
      t5old = millis();
      readone();
    }
    if ((unsigned long)(millis() - t6old) >= t6interval) {
      t6old = millis();
      pubth();
    }
    if ((unsigned long)(millis() - t7old) >= t7interval) {
      t7old = millis();
      readbme();
    }
    delay(50);
  }
}
