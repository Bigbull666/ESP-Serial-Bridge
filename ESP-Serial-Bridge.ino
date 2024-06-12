/*********************************************************************************
 * ESP-Serial-Bridge
 *
 * Simple WiFi Serial Bridge for Espressif microcontrollers
 *
 * Forked from https://github.com/AlphaLima/ESP32-Serial-Bridge
 *
 * Added compatibility for ESP8266, WiFi reconnect on failure, and mDNS
 *discovery.
 *
 * Note: ESP8266 is limited to 115200 baud and may be somewhat unreliable in
 *       this application.
 *
 *   -- Yuri - Aug 2021
 *
 * Fixed compatibility with Arduino framework 2.0 -- Yuri - Apr 2023
 * Implemented UDP + bug fixes                    -- Yuri - Apr 2023
 *
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
 *********************************************************************************/

#include <Arduino.h>

#ifdef ESP32
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>  // use SoftwareSerial for ESP8266
#endif

#include "config.h"

HardwareSerial Serial_one(1);
HardwareSerial* COM[NUM_COM] = {&Serial_one};

#include <WiFiClient.h>
WiFiServer server_0(SERIAL1_TCP_PORT);

WiFiServer* server[NUM_COM] = {&server_0};
WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];

uint8_t buf1[NUM_COM][BUFFERSIZE];
uint8_t buf2[NUM_COM][BUFFERSIZE];

uint16_t i1[NUM_COM] = {0};
uint16_t i2[NUM_COM] = {0};

uint8_t BTbuf[BUFFERSIZE];
uint16_t iBT = 0;


    void setup() {
        delay(500);
        Serial.begin(115200);

        COM[0]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
//        COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
//        COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
//        COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);

        Serial.print("\n\nWiFi serial bridge ");
        Serial.println(VERSION);

        Serial.println("Open ESP Access Point Mode");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(SSID, PASSWD);  // configure SSID and password for softAP
        delay(2000);            // VERY IMPORTANT
        WiFi.softAPConfig(STATIC_IP, STATIC_IP, NETMASK);  // configure ip address for softAP

        for (int num = 0; num < NUM_COM; num++) {
            Serial.print("Starting TCP Server ");
            Serial.println(num + 1);
            server[num]->begin();  // start TCP server
            server[num]->setNoDelay(true);
        }


#ifdef BATTERY_SAVER
        esp_err_t esp_wifi_set_max_tx_power(50);  // lower WiFi Power
#endif
    }

    void loop() {


        for (int num = 0; num < NUM_COM; num++) {
            if (server[num]->hasClient()) {
                byte i = 0;
                for (i = 0; i < MAX_NMEA_CLIENTS; i++) {
                    // find free/disconnected spot
                    if (!TCPClient[num][i] || !TCPClient[num][i].connected()) {
                        if (TCPClient[num][i]) TCPClient[num][i].stop();
                        TCPClient[num][i] = server[num]->accept();
                        if(!TCPClient[num][i]) {
                          Serial.println("available broken");
                        }
                        //else {
                          Serial.print("New client for COM");
                          Serial.print(num);
                          Serial.print(" #");
                          Serial.println(i);
                        //}
                        break;
                    }
                }
                if (i >= MAX_NMEA_CLIENTS) {
                  //no free/disconnected spot so reject
                  server[num]->accept().stop();
                }
            }
        }

        for (int num = 0; num < NUM_COM; num++) {
            if (COM[num] != NULL) {
                for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
                    if (TCPClient[num][cln]) {
                        while (TCPClient[num][cln].available()) {
                            buf1[num][i1[num]] = TCPClient[num][cln].read();  // read char from client
                            i1[num]++;
                            if (i1[num] == BUFFERSIZE - 1) break;
                        }

                        COM[num]->write(buf1[num], i1[num]);  // now send to UART(num):
                        
                        Serial.print("Octopus: ");
                        Serial.write(buf1[num], i1[num]);  // now send to UART(num):
                        i1[num] = 0;
                    }
                }

                if (COM[num]->available()) {
                    while (COM[num]->available()) {
                        buf2[num][i2[num]] = COM[num]->read();  // read char from UART(num)
                        i2[num]++;
                        if (i2[num] == BUFFERSIZE - 1) break;
                    }
                    Serial.print("Raspbeyry: ");
                    Serial.write(buf2[num], i2[num]);
                    for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
                        if (TCPClient[num][cln])
                            TCPClient[num][cln].write(buf2[num], i2[num]);
                    }

                    i2[num] = 0;
                }
            }
        }
    }
