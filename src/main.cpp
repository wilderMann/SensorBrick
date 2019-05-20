#include "config.h"
#include <DHTesp.h>
#include "Adafruit_TCS34725.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include "homie.hpp"

#define CLIENTID "ESP8266Client-"
#define SLEEPSECONDS 300
#define SERIAL true
#define HOMIE_SERIAL false
#define BAT_CORRECTION 4665

#define SDA_TCS D5
#define SCL_TCS D6
#define ONEWIRE_DHT D1


typedef struct dht_s {
        float temp;
        float humidity;
        float heatIndex;
}dht_t;

typedef struct bat_s {
        float volt;
        float percentage;
}bat_t;

typedef struct rgbL_s {
        float red;
        float green;
        float blue;
        uint16_t temp;
        uint16_t lux;
} rgbL_t;

void callback (char *topic, byte *payload, unsigned int length);
boolean reconnect();
void meassurement();
dht_t getDhtData();
bat_t getBatData();
rgbL_t getLightData();

WiFiClient espClient;
PubSubClient client(MQTT_IP, MQTT_PORT, callback, espClient);
Homie homieCTRL = Homie(&client);
DHTesp dht;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

unsigned long lastReconnectAttempt = 0;
uint8_t flag_measure = 0;
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
        while (!Serial);
        if(SERIAL) Serial.println("");
        if(SERIAL) Serial.print("Connected, IP address: ");
        if(SERIAL) Serial.println(WiFi.localIP());
        if(SERIAL) Serial.println();

        Wire.begin(SDA_TCS,SCL_TCS);
        if (tcs.begin()) {
                if(SERIAL) Serial.println("Found sensor");
        } else {
                if(SERIAL) Serial.println("No TCS34725 found ... check your connections");
        }


        HomieDevice homieDevice = HomieDevice(DEVICE_NAME, "Sensorbrick", WiFi.localIP().toString().c_str(),
                                              WiFi.macAddress().c_str(), FW_NAME, FW_VERSION,
                                              "esp8266", "");
        HomieNode dhtNode = HomieNode("air-sensors", "Air Sensors", "DHT22");
        HomieProperties temperature = HomieProperties("temperature", "Temperature",
                                                      false, true, "°C",
                                                      homie::float_t);
        HomieProperties humidity = HomieProperties("humidity", "Humidity", false,
                                                   true, "%", homie::float_t);
        HomieProperties heatIndex = HomieProperties("heat-index", "Heat Index",
                                                    false, true, "°C",
                                                    homie::float_t);

        HomieNode battery = HomieNode("battery-sensor", "Battery Sensor", "LiPo");
        HomieProperties voltage = HomieProperties("voltage", "Voltage",
                                                  false, true, "V",
                                                  homie::float_t);
        HomieProperties percentage = HomieProperties("percentage", "Battery Percentage",
                                                     false, true, "%",
                                                     homie::float_t);

        HomieNode lighting = HomieNode("light-sensor", "Light Sensor", "TCS34725");
        HomieProperties rgb = HomieProperties("rgb", "rgb",
                                              false, true, "",
                                              homie::color_t,"rgb");
        HomieProperties lux = HomieProperties("lux", "Lux",
                                              false, true, "lx",
                                              homie::integer_t);
        HomieProperties colorTemp = HomieProperties("color-temp", "Color Temperature",
                                                    false, true, "K",
                                                    homie::integer_t);

        // if((float) analogRead(A0) * divider <= 6.5) ESP.deepSleep(1000000 * 5);
        dhtNode.addProp(temperature);
        dhtNode.addProp(humidity);
        dhtNode.addProp(heatIndex);
        battery.addProp(voltage);
        battery.addProp(percentage);
        lighting.addProp(rgb);
        lighting.addProp(lux);
        lighting.addProp(colorTemp);
        homieDevice.addNode(dhtNode);
        homieDevice.addNode(battery);
        homieDevice.addNode(lighting);
        homieCTRL.setDevice(homieDevice);
        if(SERIAL) Serial.println("Status\tHumidity (%)\tTemperature (C)\tHeatIndex (C)");
        dht.setup(ONEWIRE_DHT, DHTesp::DHT22); // DHT22 on Pin1
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
                dht_t dhtData = getDhtData();
                bat_t batData = getBatData();
                rgbL_t lightData = getLightData();

                if(SERIAL) Serial.print(dht.getStatusString());
                if(SERIAL) Serial.print("\t");
                if(SERIAL) Serial.print(dhtData.humidity, 1);
                if(SERIAL) Serial.print("\t\t");
                if(SERIAL) Serial.println(dhtData.temp, 1);

                char buffer[15]; //3 char + point + 2 char
                sprintf(buffer, "%.2f", dhtData.temp);
                string topic = "homie/" + string(DEVICE_NAME) + "/air-sensors/temperature";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%.2f", dhtData.humidity);
                topic = "homie/" + string(DEVICE_NAME) + "/air-sensors/humidity";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%.2f", dhtData.heatIndex);
                topic = "homie/" + string(DEVICE_NAME) + "/air-sensors/heat-index";
                client.publish(topic.c_str(),buffer,true);

                sprintf(buffer, "%.2f", batData.volt);
                topic = "homie/" + string(DEVICE_NAME) + "/battery-sensor/voltage";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%.2f", batData.percentage);
                topic = "homie/" + string(DEVICE_NAME) + "/battery-sensor/percentage";
                client.publish(topic.c_str(),buffer,true);

                sprintf(buffer, "%.0f,%.0f,%.0f", lightData.red, lightData.green, lightData.blue);
                topic = "homie/" + string(DEVICE_NAME) + "/light-sensor/rgb";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%d", lightData.lux);
                topic = "homie/" + string(DEVICE_NAME) + "/light-sensor/lux";
                client.publish(topic.c_str(),buffer,true);
                sprintf(buffer, "%d", lightData.temp);
                topic = "homie/" + string(DEVICE_NAME) + "/light-sensor/color-temp";
                client.publish(topic.c_str(),buffer,true);

                flag_measure = 0;
                delay(1);
                homieCTRL.sleep();
                if(SERIAL) Serial.print("deepSleep for ");
                long sleepTime = 1000000 * SLEEPSECONDS;
                if(batData.percentage < 15) {
                        sleepTime = sleepTime * 4;
                } else {
                        if(batData.percentage < 30) {
                                sleepTime = sleepTime * 2;
                        }
                }
                if(SERIAL) Serial.println(sleepTime);
                delay(1.0);
                ESP.deepSleep(sleepTime);
        }
        homieCTRL.loop();
}

dht_t getDhtData(){
        dht_t data;
        data.temp= dht.getTemperature();
        data.humidity = dht.getHumidity();
        data.heatIndex = dht.computeHeatIndex(data.temp, data.humidity, false);
        return data;
}

bat_t getBatData(){
        bat_t data;
        float correction = (float)BAT_CORRECTION / (float)1000000;
        long vbat = analogRead(A0);
        data.volt = (float)vbat * correction;
        data.percentage = (data.volt - 2.9) / (4.2 - 2.9) * (float) 100;
        return data;
}

rgbL_t getLightData(){
        rgbL_t data;
        uint16_t r,g,b,c;
        tcs.getRawData(&r, &g, &b, &c);
        tcs.getRGB(&data.red, &data.green, &data.blue);
        data.lux = tcs.calculateLux(r,g,b);
        data.temp = tcs.calculateColorTemperature(r,g,b);
        if(SERIAL) {
                Serial.print("red: "); Serial.print(data.red, DEC);
                Serial.print(" r: "); Serial.println(r, DEC);
                Serial.print("green: "); Serial.print(data.green, DEC);
                Serial.print(" g: "); Serial.println(g, DEC);
                Serial.print("blue: "); Serial.print(data.blue, DEC);
                Serial.print(" b: "); Serial.println(b, DEC);
                Serial.print("lux: "); Serial.println(data.lux);
                Serial.print("temp: "); Serial.println(data.temp);
        }
        return data;
}

void meassurement() {
        flag_measure = 1;
}

boolean reconnect() {
        // Loop until we're reconnected
        return homieCTRL.connect(ClientID.c_str(), MQTT_USR, MQTT_PW);
}

void callback(char *topic, byte *payload, unsigned int length) {
}
