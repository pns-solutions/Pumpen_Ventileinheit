#include <ArduinoMqttClient.h>
#include <Wire.h>                     // enable I2C.
#include "arduino_secrets.h"
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
#include <ESP8266WiFi.h>
#endif

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

IPAddress broker = {192,168,178,74};
int        port     = 8883;

const long interval = 1000;
unsigned long previousMillis = 0;


// PUMPEN
unsigned int pump_1_pin = 13;
unsigned long pump_1_timer = 0;

// VENTILE
unsigned int valve_1_pin = 12;
unsigned long valve_1_timer = 0;



void setup() {
  // put your setup code here, to run once:
//Initialize serial and wait for port to open:
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // attempt to connect to WiFi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        // failed, retry
        Serial.print(".");
        delay(5000);
    }

    Serial.println("You're connected to the network");
    Serial.println();

    // You can provide a unique client ID, if not set the library uses Arduino-millis()
    // Each client must have a unique client ID
    // mqttClient.setId("clientId");

    // You can provide a username and password for authentication
    // mqttClient.setUsernamePassword("username", "password");

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);

    if (!mqttClient.connect(broker, port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1);
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

    mqttClient.onMessage(onMqttMessage);

    // TOPICS DER ANGESCHLOSSENEN HARDWARE ABONNIEREN
    mqttClient.subscribe("pump_1");
    mqttClient.subscribe("valve_1");

    // PINS setzen
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  mqttClient.poll();

  stopActions();
   
}

void stopActions() {
  unsigned long currentMillis = millis();

  if(digitalRead(pump_1_pin) == HIGH && currentMillis >= pump_1_timer) {
    digitalWrite(pump_1_pin, LOW);
    sendActionStopMessage(1, true);
  }
  if(digitalRead(valve_1_pin) == HIGH && currentMillis >= valve_1_timer) {
    digitalWrite(valve_1_pin, LOW);
    sendActionStopMessage(1, false);
  }
}

void startAction(int actor, int duration) {
  switch(actor) {
    case 1:
      // -Pumpe 1
      // PinX on
      digitalWrite(pump_1_pin, HIGH);
      // setze Timer
      pump_1_timer = millis() + (1000 * duration);
      break;
    case 2:
    // -Ventil 1
      // PinX on
      digitalWrite(valve_1_pin, HIGH);
      // setze Timer
      valve_1_timer = millis() + (1000 * duration);
      break;
  }
}

void sendActionStopMessage(int actor, bool isPump) {
  String topic = "pumpActionStop";
  if(!isPump) topic = "valveActionStop";
  mqttClient.beginMessage(topic);
  mqttClient.print(actor);
  mqttClient.endMessage();
}

void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(topic);
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");
  // use the Stream interface to print the contents
  String payload = "";
  while (mqttClient.available()) {
    payload += (char)mqttClient.read();
  }
  Serial.print(payload);
  Serial.println();

  if(topic.equals("pump_1")) {
    startAction(1, payload.toInt());
  } else if(topic.equals("valve_1")) {
    startAction(2, payload.toInt());
  }
}
