/*******************************************
  Mini sensor package running on a NodeMCU ESP8266; publishes sensor data (packaged in JSON format) over WiFi using MQTT.

  Author: Dave Parsons
  Last Updated: 2021-09-02

  Sensors currently supported:
  - BME680 temp/humidity/pressure/gas sensor (I2C)
  - VEML7700 lux sensor (I2C)

  Features not currently present (but could easily be added):
  - inputs to control some of the various sensor settings
  - actions based on MQTT messages received / topics subscribed to
*******************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_VEML7700.h>

/**** START SETTINGS ****/
const char* SSID = "MonkeyNet"; //WiFi SSID
const char* PASSWD = "aR^sKmTOR%$6*#x0G9wF5Cn7"; //WiFi password

// MQTT broker connection
const char* MQTT_SERVER = "10.0.0.39";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "monkeybroker";
const char* MQTT_PASSWD = "monkeybroker";

/*
 * MQTT client ID and basic state topics / topic publish frequency (ms)
 * By default the client ID is used in all the topics so if you're keeping this type of topic format
 * you'll need to change each topic string when customizing.
 * 
 * Default topic structure: sensors/{MQTT_CLIENTID}/{TOPIC}
 */
const char *MQTT_CLIENTID = "sensor002";

// State topic
const char *MQTT_TOPIC_STATE = "sensors/sensor002/STATE";
const int MQTT_TOPIC_STATE_FREQ = 60000; //ms

// LWT topic
const char *MQTT_TOPIC_LWT = "sensors/sensor002/LWT";

// BME680 settings
bool hasBME = true; // set false if not using this sensor

/* 
 * The gas sensor uses a heater that can throw off temp measurements in some cases,
 * set to true if you want gas measurement (may also require setting the offset for temps)
 */
bool bmeUseGas = false;

/* 
 *  Offset added to the temperature value returned to account for additional board or enclosure
 *  heat - calibrate with a known accurate temperature source first. This value is ADDED to the temperature,
 *  so use a negative for offsets that lower the returned value.
 */
float bmeTempOffset = -1.00; 

// MQTT topic/publish frequency for BME sensor data; data is combined under one topic using a JSON package
const char *MQTT_TOPIC_CLIMATE = "sensors/sensor002/CLIMATE";
const int MQTT_TOPIC_CLIMATE_FREQ = 60000; //ms

// VEML7700 settings
bool hasVEML = true; // set false if not using this sensor

// MQTT topic/publish frequency for VEML sensor data; data is combined under one topic using a JSON package
const char *MQTT_TOPIC_LUX = "sensors/sensor002/LUX";
const int MQTT_TOPIC_LUX_FREQ = 60000; //ms
/**** END SETTINGS ****/

// mqtt
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long nowState = 0;
unsigned long lastMsgState = 0;

// temp/humidity/pressure/gas sensor
Adafruit_BME680 bme;
unsigned long nowBme = 0;
unsigned long lastMsgBme = 0;

// lux sensor
Adafruit_VEML7700 veml = Adafruit_VEML7700();
unsigned long nowLux = 0;
unsigned long lastMsgLux = 0;

// function to connect WiFi on setup
void setup_wifi() {
  delay(10);
  Serial.println();

  Serial.print("Connecting to ");
  Serial.print(SSID);

  WiFi.begin(SSID, PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(BUILTIN_LED, LOW);
    delay(500);
    digitalWrite(BUILTIN_LED, HIGH);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println();
  Serial.print("Connected; IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(BUILTIN_LED, HIGH);
}

// function to connect/re-connect to MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    digitalWrite(BUILTIN_LED, LOW);
    // Attempt to connect
    if (client.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWD, MQTT_TOPIC_STATE, 0, true, "OFFLINE")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(MQTT_TOPIC_STATE, "ONLINE");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" - trying again in 5 seconds");
      // Wait 5 seconds before retrying
      digitalWrite(BUILTIN_LED, HIGH);
      delay(5000);
    }
  }

  digitalWrite(BUILTIN_LED, HIGH);
}

/*
 * Code in this function executes when an MQTT message is received; this can be
 * used if your device is receiving data back from your home automation controller
 */
void callback(char* topic, byte* payload, unsigned int length) {
  // Replace the serial output with payload parsing and whatever actions are needed based on that
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(9600);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  // set up BME sensor
  if (hasBME) {
    // execution hangs and built-in LED will fast blink if BME sensor is enabled
    // but no sensor is detected
    if (!bme.begin()) {
      Serial.println("Could not find a BME680 sensor; check the wiring");
      while (1) {
        digitalWrite(BUILTIN_LED, LOW);
        delay(1000);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(1000);
      }
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    if (bmeUseGas) {
      bme.setGasHeater(320, 150);
    } else {
      bme.setGasHeater(0,0);
    }
  }

  // set up VEML sensor
  if (hasVEML) {
    // execution hangs and built-in LED will slow blink if VEML sensor is enabled
    // but no sensor is detected
    if (!veml.begin()) {
      Serial.println("Could not find a VEML7700 sensor; check the wiring");
      while (1) {
        digitalWrite(BUILTIN_LED, LOW);
        delay(3000);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(3000);
      }
    }

    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_800MS);
    veml.setLowThreshold(10000);
    veml.setHighThreshold(20000);
    veml.interruptEnable(true);
  }

  // turn light off if everything is good
  digitalWrite(BUILTIN_LED, HIGH);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop()) {
    client.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWD, MQTT_TOPIC_LWT, 0, true, "OFFLINE");
  }

  // publish state message every 5 minutes
  nowState = millis();
  if (nowState - lastMsgState > MQTT_TOPIC_STATE_FREQ) {
    lastMsgState = nowState;
    client.publish(MQTT_TOPIC_STATE, "ONLINE");
  }

  // publish BME sensor data every XS minutes
  if (hasBME) {
    nowBme = millis();
    if (nowBme - lastMsgBme > MQTT_TOPIC_CLIMATE_FREQ) {
      lastMsgBme = nowBme;
      if (!bme.performReading()) {
        Serial.println("Failed to perform BME680 sensor reading");
      } else {
        char bmePayload[85] = "{\"Temperature\":\"";
        dtostrf((bme.temperature + bmeTempOffset), 4, 2, &bmePayload[strlen(bmePayload)]);
        strcat(bmePayload, "\",\"Humidity\":\"");
        dtostrf(bme.humidity, 4, 2, &bmePayload[strlen(bmePayload)]);
        strcat(bmePayload, "\",\"Pressure\":\"");
        dtostrf(bme.pressure / 100.0, 4, 2, &bmePayload[strlen(bmePayload)]);
        strcat(bmePayload, "\",\"Gas\":\"");
        dtostrf(bme.gas_resistance / 1000.0, 4, 2, &bmePayload[strlen(bmePayload)]);
        strcat(bmePayload, "\"}");

        client.publish(MQTT_TOPIC_CLIMATE, bmePayload);
        //Serial.println(bmePayload);
      }
    }
  }

  // publish lux data every X minutes
  if (hasVEML) {
    nowLux = millis();
    if (nowLux - lastMsgLux > MQTT_TOPIC_LUX_FREQ) {
      lastMsgLux = nowLux;

      char luxPayload[75] = "{\"Lux\":\"";
      dtostrf(veml.readLux(), 4, 2, &luxPayload[strlen(luxPayload)]);
      strcat(luxPayload, "\",\"White\":\"");
      dtostrf(veml.readWhite(), 4, 2, &luxPayload[strlen(luxPayload)]);
      strcat(luxPayload, "\",\"ALS\":\"");
      dtostrf(veml.readALS(), 1, 0, &luxPayload[strlen(luxPayload)]);
      strcat(luxPayload, "\",\"Threshold\":\"");

      uint16_t irq = veml.interruptStatus();
      if (irq & VEML7700_INTERRUPT_LOW) {
        strcat(luxPayload, "0");
      } else if (irq & VEML7700_INTERRUPT_HIGH) {
        strcat(luxPayload, "1");
      }

      strcat(luxPayload, "\"}");

      client.publish(MQTT_TOPIC_LUX, luxPayload);
      //Serial.println(luxPayload);
    }
  }
}
