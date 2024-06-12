/*********************************************************************************
 * ESP32-Serial-WiFi-Client
 *
 * Simple ESP32 TCP or UDP client for use with ESP-Serial-Bridge
 *
 *   -- Yuri - Apr 2023
 *
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
 *********************************************************************************/

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "client_config.h"

#include <WiFiClient.h>

uint8_t buf[BUFFERSIZE];
uint16_t num = 0;

WiFiClient client;
void connect_to_host() {
    Serial.printf("Connecting to %s:%d...", HOST_IP.toString(), HOST_PORT);
    while (!client.connect(HOST_IP, HOST_PORT)) {
        delay(500);
        Serial.print('.');
    }
    Serial.println("connected\n");
    delay(1000);
}


void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.print("WiFi disconnected: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    Serial.println("Trying to reconnect..");
    WiFi.begin(SSID, PASSWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup() {
    delay(500);

    Serial.begin(115200);
    Serial1.begin(CLIENT_BAUD, CLIENT_PARAM, CLIENT_RXPIN, CLIENT_TXPIN);

    Serial.print("\n\nWiFi serial bridge client ");
    Serial.println(VERSION);

    Serial.println("Open ESP Station Mode");
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.begin(SSID, PASSWD);
    Serial.print("Connecting to: ");
    Serial.print(SSID);
    Serial.print("..");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    if (!client.connected()) connect_to_host();

    while (client.available()) {
        buf[num] = client.read();
        num++;
        if (num == BUFFERSIZE - 1) break;
    }
    if (num > 0) Serial1.write(buf, num);
    if (num > 0) {
      Serial.print("Received: ");
      Serial.write(buf, num);
      Serial.println("");
    }
    num = 0;

    while (Serial1.available()) {
        buf[num] = Serial1.read();
        num++;
        if (num == BUFFERSIZE - 1) break;
    }
    if (num > 0) client.write(buf, num);
    if (num > 0) {
      Serial.print("Responding: ");
      Serial.write(buf, num);
      Serial.println("");
    }
    num = 0;
}
