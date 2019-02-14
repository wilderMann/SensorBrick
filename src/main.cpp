#include <DHTesp.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include "config.h"

#define CLIENTID "ESP8266Client-"

void callback(char* topic, byte* payload, unsigned int length);
boolean reconnect();
void meassurement();
void sleep();


WiFiClient espClient;
PubSubClient client(MQTT_IP,MQTT_PORT,callback,espClient);
DHTesp dht;
Ticker dhtMeasure;
Ticker deepSleep;

unsigned long lastReconnectAttempt = 0;
uint8_t flag_measure = 0;
uint8_t flag_sleep = 0;
String ClientID;
const float divider = 0.0092998;    //Divider for Vcc meassurement

void setup(){

        Serial.begin(115200);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID,WIFI_PASS);
        while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
        }
        while(!Serial);
        Serial.println("");
        Serial.print("Connected, IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println();

        //if((float) analogRead(A0) * divider <= 6.5) ESP.deepSleep(1000000 * 5);

        Serial.println("Status\tHumidity (%)\tTemperature (C)\tHeatIndex (C)");
        dht.setup(D1, DHTesp::DHT22); //DHT22 on Pin1

}

void loop(){
        if (!client.connected()) {
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
        if(flag_measure == 1) {
                float humidity = dht.getHumidity();
                float temperature = dht.getTemperature();

                Serial.print(dht.getStatusString());
                Serial.print("\t");
                Serial.print(humidity, 1);
                Serial.print("\t\t");
                Serial.print(temperature, 1);
                Serial.print("\t\t");
                Serial.println(dht.computeHeatIndex(temperature, humidity, false), 1);

                const int capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4);
                const int capacityData = JSON_OBJECT_SIZE(3);
                StaticJsonBuffer<capacity> jb;
                StaticJsonBuffer<capacityData> jbData;
                JsonObject& root = jb.createObject();
                JsonObject& data = jbData.createObject();
                data["temperature"].set(temperature);
                data["humidity"].set(humidity);
                data["heatIndex"].set(dht.computeHeatIndex(temperature, humidity, false));

                root["device"].set(DEVICE_NAME);
                //root["Vin"].set((float) analogRead(A0) * divider);
                root["sensor"].set("DHT22");
                root["data"].set(data);

                String topic = "/" + String(DEVICE_NAME);
                char* buffer = (char*) malloc(topic.length()+1);
                topic.toCharArray(buffer, topic.length()+1);
                String output;
                root.printTo(output);
                uint8_t* buffer2 = (uint8_t*) malloc(output.length()+1);
                output.getBytes(buffer2, output.length()+1);
                client.publish(buffer, buffer2,output.length()+1,true);
                free(buffer);
                free(buffer2);

                dhtMeasure.detach();
                flag_measure = 0;
                deepSleep.attach(1.0,sleep);
        }
        if(flag_sleep == 1) {
                flag_sleep = 0;
                deepSleep.detach();
                Serial.println("deepSleep");
                ESP.deepSleep(1000000 * 5);
        }
        client.loop();


}

void meassurement(){
        flag_measure = 1;
}

void sleep(){
        flag_sleep = 1;
}

boolean reconnect() {
        // Loop until we're reconnected
        if (client.connect(ClientID.c_str(),MQTT_USR,MQTT_PW)) {
                Serial.println(" MQTT Connected");

        }else{
                Serial.print("MQTT conncetion failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
        }
        return client.connected();
}

void callback(char* topic, byte* payload, unsigned int length){
}
