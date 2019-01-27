#include "ESP8266WiFi.h"
#include "Wire.h"

#include <bsec_integration.h>
#include <bme680.h>
#include <DFRobot_BME680.h>
#include <bsec_interface.h>
#include <bsec_datatypes.h>
#include <DFRobot_BME680_I2C.h>
#include <bme680_defs.h>
#include <PubSubClient.h>

/* use an accurate altitude to calibrate sea level air pressure */
#define CALIBRATE_PRESSURE

DFRobot_BME680_I2C bme(0x77);  //0x77 I2C address

const char* sensor = "unbound";

const char* ssid = "IoT";
const char* password = "<wifi password>";

const char* mqttServer = "192.168.1.55";
const char* mqttUser = "envorino";
const char* mqttPassword = "<mqtt password>";

const float berlin_seaLevel = 48.0;
const int sleepTimeS = 10;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp2", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


float seaLevel; 
void setup()
{
  uint8_t rslt = 1;
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  Serial.println();
  while(rslt != 0) {
    rslt = bme.begin();
    if(rslt != 0) {
      Serial.println("bme begin faild");
      delay(2000);
    }
  }
  Serial.println("bme begin successful");
  bme.supportIAQ();

  // Sensor name from MAC address
  String eth_mac = WiFi.macAddress();
  Serial.println("MAC address: " + eth_mac);

  if (eth_mac.equals("5C:C7:7F:CC:25:C2")) {
    sensor = "jesper_office";
  }

  setup_wifi();
  client.setServer(mqttServer, 1883);
  
}

void report_mem_usage() {
  Serial.print(millis());
  Serial.print(" -> Memory usage, free_heap=");
  Serial.println(ESP.getFreeHeap());
}

void publish(char* measure, float val) {
  char valBuf[8];
  char payload[100];
  char topic[100];

  dtostrf(val, 3, 2, valBuf);
  
  sprintf(payload, "envorino,sensor=%s %s=%s", sensor, measure, valBuf);
  sprintf(topic, "sensors/%s/%s", sensor, measure);
  
  client.publish(topic, payload);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static uint8_t firstCalibrate = 0;
  
  #ifdef CALIBRATE_PRESSURE
  if(firstCalibrate == 0) {
    if(bme.iaqUpdate() == 0) {
      seaLevel = bme.readSeaLevel(berlin_seaLevel);
      firstCalibrate = 1;
    }
  }
  #else
    firstCalibrate = 1;
  #endif
  
  if(firstCalibrate) {
    uint8_t rslt = bme.iaqUpdate();

    if(rslt == 0) {
      report_mem_usage();

      publish("temperature", bme.readTemperature());
      publish("pressure", bme.readPressure());
      publish("humidity", bme.readHumidity());
      publish("altitude", bme.readAltitude());
      publish("gas_resistance", bme.readGasResistance());

      if(bme.isIAQReady()) {
        publish("iaq", bme.readIAQ());
      } else {
        Serial.println("IAQ not ready, please wait about 5 minutes");
      }
    
      #ifdef CALIBRATE_PRESSURE
        publish("calibrated_altitude", bme.readCalibratedAltitude(seaLevel));
      #endif
    }
  }
}
