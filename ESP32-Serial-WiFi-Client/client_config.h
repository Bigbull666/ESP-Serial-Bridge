/*********************************************************************************
 * client_config.h
 * 
 * Added ESP32-Serial-WiFi-Client   -- Yuri - Apr 2023
 * 
*********************************************************************************/
#ifndef CLIENT_CONFIG_H
#define CLIENT_CONFIG_H

#include <WiFi.h>


#define SSID   "wiser"      // SSID to join
#define PASSWD "wisersioko"  // wiFi password

#define BUFFERSIZE 1024

#define VERSION "2.0-ESP32"

#define PROTOCOL_TCP                       // PROTOCOL_TCP or PROTOCOL_UDP
#define HOST_IP IPAddress(192, 168, 4, 1)  // only used for PROTOCOL_TCP
#define HOST_PORT 8881                    // TCP or UDP port
#define CLIENT_BAUD 115200
#define CLIENT_PARAM SERIAL_8N1
#define CLIENT_TXPIN 17
#define CLIENT_RXPIN 16

#endif  // CLIENT_CONFIG_H
