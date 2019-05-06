#include "config.h"
#include <DHTesp.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include "homie.hpp"

#define CLIENTID "ESP8266Client-"
#define SLEEPSECONDS 60

#define SERIAL false

void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void meassurement();
void sleep();

WiFiClient espClient;
PubSubClient client(MQTT_IP, MQTT_PORT, callback, espClient);
Homie homieCTRL = Homie(&client);
DHTesp dht;
Ticker dhtMeasure;
Ticker deepSleep;

unsigned long lastReconnectAttempt = 0;
uint8_t flag_measure = 0;
uint8_t flag_sleep = 0;
String ClientID;
const float divider = 0.0092998; // Divider for Vcc meassurement

void setup() {

        Serial.begin(115200);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                if(SERIAL) Serial.print(".");
        }
        while (!Serial)
                ;
        if(SERIAL) Serial.println("");
        if(SERIAL) Serial.print("Connected, IP address: ");
        if(SERIAL) Serial.println(WiFi.localIP());
        if(SERIAL) Serial.println();

        HomieDevice homieDevice = HomieDevice(DEVICE_NAME, "Sensorbrick", WiFi.localIP().toString().c_str(),
                                              WiFi.macAddress().c_str(), FW_NAME, FW_VERSION,
                                              "esp8266", "");
        HomieNode dhtNode = HomieNode("air-sensors", "Air Sensors", "DHT22");
        HomieProperties temperature = HomieProperties("temperature", "Temperature",
                                                      false, true, "°C",
                                                      homie::float_t);
        HomieProperties humidity = HomieProperties("humidity", "Humidity", false,
                                                   true, "%", homie::float_t);

        // if((float) analogRead(A0) * divider <= 6.5) ESP.deepSleep(1000000 * 5);
        dhtNode.addProp(temperature);
        dhtNode.addProp(humidity);
        homieDevice.addNode(dhtNode);
        homieCTRL.setDevice(homieDevice);
        if(SERIAL) Serial.println("Status\tHumidity (%)\tTemperature (C)\tHeatIndex (C)");
        dht.setup(D1, DHTesp::DHT22); // DHT22 on Pin1
}

void loop() {
        if (!homieCTRL.connected()) {
                unsigned long now = millis();
                if (now - lastReconnectAttempt > 5000) {
                        lastReconnectAttempt = now;
                        // Attempt to reconnect
                        if (reconnect()) {
                                lastReconnectAttempt = 0;
                                delay(dht.getMinimumSamplingPeriod());
                                meassurement();
                        }
                }
        }
        if (flag_measure == 1) {
                float humidity = dht.getHumidity();
                float temperature = dht.getTemperature();

                if(SERIAL) Serial.print(dht.getStatusString());
                if(SERIAL) Serial.print("\t");
                if(SERIAL) Serial.print(humidity, 1);
                if(SERIAL) Serial.print("\t\t");
                if(SERIAL) Serial.println(temperature, 1);

                char buffer[6]; //3 char + point + 2 char
                sprintf(buffer, "%.2f", temperature);
                string topic = "homie/" + string(DEVICE_NAME) + "/air-sensors/temperature";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%.2f", humidity);
                topic = "homie/" + string(DEVICE_NAME) + "/air-sensors/humidity";
                client.publish(topic.c_str(),buffer,true);

                dhtMeasure.detach();
                flag_measure = 0;
                deepSleep.attach(1.0, sleep);
        }
        if (flag_sleep == 1) {
                flag_sleep = 0;
                homieCTRL.sleep();
                deepSleep.detach();
                if(SERIAL) Serial.println("deepSleep");
                ESP.deepSleep(1000000 * SLEEPSECONDS);
        }
        homieCTRL.loop();
}

void meassurement() {
        flag_measure = 1;
}

void sleep() {
        flag_sleep = 1;
}

boolean reconnect() {
        // Loop until we're reconnected
        return homieCTRL.connect(ClientID.c_str(), MQTT_USR, MQTT_PW);
}

void callback(char *topic, byte *payload, unsigned int length) {
}
