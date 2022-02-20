/*

*/

#ifndef ESP8266_OTA_H
#define ESP8266_OTAP_H

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

bool InitOTA() {
  // ArduinoOTA.setPort(8266);                                            // Puerto por defecto 8266
  ArduinoOTA.setHostname("Enphase_Termo");                                // Nombre para el módulo ESP8266 al descubrirlo por el Ide de Arduino
  ArduinoOTA.setPassword("********");                                     // 

  ArduinoOTA.onStart([]() {
  });

  // ArduinoOTA.onEnd([]() {});

  // ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { });

  ArduinoOTA.onError([](ota_error_t error) {
    return 0;
  });

  ArduinoOTA.begin();
  return 1;
}

#endif
