#include "config.h"
#include <ESP8266WiFi.h>

#define SERIAL true




WiFiClient espClient;

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


}

void loop() {

}
