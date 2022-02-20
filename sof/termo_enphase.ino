/*
 * Control de termo eléctrico a través de dimmer SSR sincrono
 * cuando tengamos excedentes de energía fotovoltaica, leyendo del 
 * envoy de Enphase
 * Se gestiona la recirculación del agua del termo "Enphase" al termo de principal 
 * Pantalla OLED SDD1306 para mostar información
 * 
http://envoy.local/production.json

  respuesta Json:
  {"production":[{"type":"inverters","activeCount":4,"readingTime":16387103,
  "wNow":0,"whLifetime":1446},{"type":"eim","activeCount":1,"measurementType":"production",
  "readingTime":1637350,"wNow":0.0,"whLifetime":272.12,"varhLeadLifetime":0.097,
  "varhLagLifetime":1162.537,"vahLifetime":407882.147,"rmsCurrent":0.44,"rmsVoltage":236.878,
  "reactPwr":62.257,"apprntPwr":103.938,"pwrFactor":0.0,"whToday":1476.12,"whLastSevenDays":19091.12,
  "vahToday":3327.0,"varhLeadToday":0.097,"varhLagToday":1409.537}],
  "consumption":[{"type":"eim","activeCount":1,"measurementType":"total-consumption",
  "readingTime":1639087,"wNow":27.18,"whLifetime":11113.247,"varhLeadLifetime":801057.219,
  "varhLagLifetime":1157.647,"vahLifetime":15155.735,"rmsCurrent":2.304,
  "rmsVoltage":236.819,"reactPwr":-341.499,"apprntPwr":545.723,"pwrFactor":0.47,
  "whToday":746.247,"whLastSevenDays":66584.247,"vahToday":10416.0,"varhLeadToday":6143.219,
  "varhLagToday":17.647},{"type":"eim","activeCount":1,"measurementType":"net-consumption",
  "readingTime":16390850,"wNow":257.18,"whLifetime":963148.409,"varhLeadLifetime":1057.121,
  "varhLagLifetime":2874.11,"vahLifetime":15455.735,"rmsCurrent":1.865,"rmsVoltage":236.76,
  "reactPwr":-279.242,"apprntPwr":437.428,"pwrFactor":0.58,"whToday":0,"whLastSevenDays":0,
  "vahToday":0,"varhLeadToday":0,"varhLagToday":0}],"storage":[{"type":"acb","activeCount":0,
  "readingTime":0,"wNow":0,"whNow":0,"state":"idle"}]}

1216 Bytes necesarios para almacenar los objetos JSON y las matrices en memoria
Strings 336 Bytes necesarios para almacenar las cadenas en memoria
Total (mínimo) 1552 Capacidad mínima del JsonDocument.
Total (recomendado) 2048 Incluyendo algo de holgura en caso de que las 
cadenas cambien, y redondeado a una potencia de dos

--Regulación carga resistiva con SSR sincrono:
Control SSR de activación por paso por cero y regulación por disparo de ráfaga (Brust Fire Control):
El Control de Ráfaga es una técnica de proporción de tiempo que utiliza ciclos completos de CA,
controlando con precisión el número de ciclos de CA encendidos dentro de una serie global fija de ciclos.
Se utiliza una base de 10 ciclos (200ms) y varía el número de ciclos de encendido para variar la potencia media
suministrada a una carga. El disparo de cruce por cero solo utiliza los ciclos de CA completos y,
al encenderse sólo en el punto de cruce por cero, minimiza el ruido eléctrico EMI.
 1-9%  ->pulso de 15ms ON 185ms OFF (ON 20ms)  ->timer1 TIM_DIV16 =75000
 10-19%->pulso de 35ms ON 165ms OFF (ON 40ms)  ->timer1 TIM_DIV16 =175000
 20-29%->pulso de 55ms ON 145ms OFF (ON 60ms)  ->timer1 TIM_DIV16 =275000
 30-39%->pulso de 75ms ON 125ms OFF (ON 80ms)  ->timer1 TIM_DIV16 =375000
 40-49%->pulso de 95ms ON 105ms OFF (ON 100ms) ->timer1 TIM_DIV16 =475000
 50-59%->pulso de 115ms ON 85ms OFF (ON 120ms) ->timer1 TIM_DIV16 =575000
 60-69%->pulso de 135ms ON 65ms OFF (ON 140ms) ->timer1 TIM_DIV16 =675000
 70-79%->pulso de 155ms ON 45ms OFF (ON 160ms) ->timer1 TIM_DIV16 =775000
 80-89%->pulso de 175ms ON 25ms OFF (ON 180ms) ->timer1 TIM_DIV16 =875000
 90-99%->pulso de 195ms ON  5ms OFF (ON 180ms) ->timer1 TIM_DIV16 =975000
--------------------------------------------------------------------------------------
La ganancia de la entrada ADC se pueden cambiar a través de la siguientes
funciones, pero se tiene que tener cuidado de no superar nunca VDD + 0,3 V,
La configuración de estos valores de forma incorrecta puede destruir el ADC!
                                                           ADS1015  ADS1115
                                                           -------  -------
ads.setGain(GAIN_TWOTHIRDS);  2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
ads.setGain(GAIN_ONE);        1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
ads.setGain(GAIN_TWO);        2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
ads.setGain(GAIN_FOUR);       4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
ads.setGain(GAIN_EIGHT);      8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
ads.setGain(GAIN_SIXTEEN);    16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    8/1/22

*/
#define ARDUINOJSON_USE_DOUBLE 1

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>                                          // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266HTTPClient
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>                                                // https://github.com/bblanchon/ArduinoJson v. 6
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>                                           // Modificado linea 132 cd Adafruit_ADS1X15.cpp por "config |= RATE_ADS1115_860SPS;"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define EEPROM_SIZE 10

#define DebugSerial 2

#include "ESP8266_OTA.h"

// pin detector de flujo D0
#define detecFlowPin 16
// pin I2C para OLED SDD1306 y ADS11115 D1
// SCL GPIO 5
// pin I2C para OLED SDD1306 y ADS11115 D2
// SDA GPIO 4
// pin datos sensores temperatura D3
#define oneWireBusPin 0
// pin led estado Dimmer (D4)
#define LedPin 2
// pin relé bomba recirculación D5
#define relayPumpPin 14
// pin salida SSR D6
#define outputPin 12
// pin paso por cero D7
#define zerocrossPin 13
// pin relé auxiliar (NO usado) S8
//#define relayAuxPin 15  

// OLED SDD1306 0,96 pulgadas, I2C dirección I2C 0x3C (para 128x64),. dual color amarillo en las dos primeras lineas, resto azul
#define SCREEN_WIDTH 128 												// Ancho de pantalla OLED, en píxeles
#define SCREEN_HEIGHT 64 												// Altura de la pantalla OLED, en píxeles
// Declaración para pantalla SSD1306 conectada a I2C (pines SDA, SCL)
#define OLED_RESET -1 													// NO se usa pin de Reset pin

#define TEMPERATURE_PRECISION 9

bool SSD1306 = false;
bool DS1820 = false;
bool ADS1115 = false;
float TempEnphase = 0.0;
float TempHome = 0.0;
float maxTempEnphase = 70.0;
float Corriente = 0.0;
float producSolar = 0.0;
float consumo = 0.0;
float consumoRed = 0.0;
float radiacionSolar = 0.0;
// tabla de potencia según potencia de regulación dimmer (resistencia de 45ohms a 233V)
uint16_t tablePower[11] = {110, 220, 330, 455, 560, 670, 780, 885, 990, 1100, 1150};
uint16_t adcOffset = 14145;
uint16_t ADC0 = 0;

uint8_t numberOfDevices = 0;

// Configuración red WiFi
const char *ssid = "xxxxxxx";
const char *password = "*********";

// configuración IP estática
IPAddress staticIP(192, 168, xxx, xxx);
IPAddress gateway(192, 168, xxx, xxx);
IPAddress dnServer(192, 168, xxx, xxx);
IPAddress subnet(255, 255, 255, 0);
// Nombre del punto de acceso si falla la conexión a la red WiFi
#define nameAp "TermoEnphase"
#define passAp "espEnphase"

// Temporizador
unsigned long ultimaConsulta = 0;
// 4.5 minutos (<MQTT_KEEPALIVE)
unsigned long tiempoConsulta = 270000;

bool APmode = false;
bool okMqtt = false;
bool locate = false;
bool manualRun = false;
// Arrancamos en modo control habilitado
bool StatusRunStop = true;
// Estado bomba recirculación
bool StatusRunBomba = false;
// Estado de flujo de agua entre termos
bool StatusFlow = true;
// Indicación stop por temperatura máxima en termo Enphase
bool stopEnphase = false;

// Periodo de reset 5 días
unsigned long periodoReset = 432000000 + tiempoConsulta;

// Uptime overflow
#define UPTIME_OVERFLOW 4294967295UL
const char *sofVersion = "0.1.5";
unsigned long speedData = 20000;
bool power230v = false;
bool updateData = false;
uint8_t dimmerPower = 0;
// contador regulación (histéresis)
uint8_t contHysteresis = 0;

// Instancia a objetos
OneWire oneWire(oneWireBusPin);
DallasTemperature sensorsDS18B20(&oneWire);
// dirección del sensor 1 Termo Enphase
DeviceAddress tempDeviceAddress_1 = {0x28, 0xFF, 0x64, 0x1E, 0xD, 0x9C, 0xA3, 0xB8};
// dirección del sensor 2 Termo casa
DeviceAddress tempDeviceAddress_2 = {0x28, 0xFF, 0x64, 0x1E, 0xD, 0x96, 0x10, 0x8F};
// para la versión de  16-bits
Adafruit_ADS1115 ads;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HTTPClient http;
// servidor HTTP run
WiFiClient clientHttp;
#include "mqtt.h"
#include "WCS1800.h"
#include "http.h"

ADC_MODE(ADC_VCC)
// periodo pulso del trigger del SSR, 0.2us x 75000 =15000us ->1-9% (mínimo)
volatile uint32_t timeoutPin = 75000;
volatile uint32_t oldtimeoutPin = timeoutPin;
volatile uint8_t dimmerRun = 0;
//************************************************************************************
// Isr timer 1
//************************************************************************************
IRAM_ATTR void onTimerISR()
{
  // Fin periodo activo?
  if (dimmerRun == 1)
  {
    digitalWrite(outputPin, LOW);
    dimmerRun = 2;
  }
  // Fin periodo reposo?
  if (dimmerRun == 3)
    dimmerRun = 0;
}
//************************************************************************************
// Isr pin paso por cero (50hz->10ms)
//************************************************************************************
IRAM_ATTR void onPinISR()
{
  if (dimmerRun == 0)
  {
    digitalWrite(outputPin, HIGH);
    // Inicio periodo activo
    timer1_write(timeoutPin);
    oldtimeoutPin = timeoutPin;
    dimmerRun = 1;
  }
  if (dimmerRun == 2)
  {
    // Inicio periodo en reposo
    timer1_write(1000000L - oldtimeoutPin);
    dimmerRun = 3;
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Configuración
//////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // configuramos GPIOs
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
  pinMode(relayPumpPin, OUTPUT);
  digitalWrite(relayPumpPin, HIGH);
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);
  pinMode(zerocrossPin, INPUT_PULLUP);
  pinMode(detecFlowPin, INPUT);

  EEPROM.begin(EEPROM_SIZE);
  delay(250);
#if DebugSerial
  Serial.begin(115200);
  delay(500);
  debugPrintln(String(F("\n\n<-------\n\n")));
  // 1=normal boot, 4=watchdog, 2=reset pin, 3=software reset
  debugPrintln(String(F("SYSTEM: Reinicio por: ")) + String(ESP.getResetInfo()));
#endif
#if DebugSerial == 1
  debugPrintln(String(F("SYSTEM: ID ESP: ")) + String(ESP.getChipId()));
  debugPrintln(String(F("SYSTEM: CPU frecuencia: ")) + String(ESP.getCpuFreqMHz()) + "MHz");
  debugPrintln(String(F("SYSTEM: Versión Core: ")) + String(ESP.getCoreVersion()));
  debugPrintln(String(F("SYSTEM: Versión SDK: ")) + String(ESP.getSdkVersion()));
  debugPrintln(String(F("SYSTEM: Versión: ")) + String(sofVersion));
#endif

  sensorsDS18B20.begin();
  // AddrresDS18B20();                                                                // Solo para encontrar las direcciones de los sensores conectados
  // DS1820 encontrados
  numberOfDevices = sensorsDS18B20.getDeviceCount();
  if (numberOfDevices == 2)
  {
    sensorsDS18B20.setResolution(tempDeviceAddress_1, TEMPERATURE_PRECISION);
    sensorsDS18B20.setResolution(tempDeviceAddress_2, TEMPERATURE_PRECISION);
#if DebugSerial
    debugPrintln(String(F("Sensores DS18B20 OK")));
#endif
  }
#if DebugSerial
  else
  {
    debugPrintln(String(F("Fallo en sensores DS18B20")));
  }
#endif
  // dirección I2C 0x48 defecto
  if (!ads.begin())
  {
    ADS1115 = false;
#if DebugSerial
    debugPrintln(String(F("Fallo de ADS1115")));
#endif
  }
  else
  {
    ads.setGain(GAIN_ONE);
    ADS1115 = true;
#if DebugSerial
    debugPrintln(String(F("ADS1115 OK")));
#endif
  }
  uint16_t eepromValue;
  // leemos la posición 1 de la EEPROM
  EEPROM.get(0x00, eepromValue);
  // valor última calibración->13600
  if (eepromValue == 0)
  {
    // Ejecutar una vez sin consumo, para obtener el valor de offset
    zeroRead = WCScalibrate();
    if (zeroRead > 1000)
    {
      // escribimos en la posición 0 de la EEPROM
      EEPROM.put(0x00, zeroRead);
      EEPROM.commit();
    }
  }
  else
    zeroRead = eepromValue;
  // leemos la posición 5 de la EEPROM
  EEPROM.get(0x05, eepromValue);
  // valor última calibración->14145
  if (eepromValue == 0)
  {
    // Ejecutar una vez sin radiación, para obtener el valor de offset
    readAds1115Sum(true);
    if (adcOffset > 1000)
    {
      // escribimos en la posición 5 de la EEPROM
      EEPROM.put(0x05, adcOffset);
      EEPROM.commit();
    }
  }
  else
    adcOffset = eepromValue;
  EEPROM.end();

#if DebugSerial
  debugPrintln(String(F("Calibracion WCS37A50 Vadc: ")) + String(adcOffset));
#endif
  if (zeroRead < 400)
  {
    WCS1800 = false;
    zeroRead = 0;
#if DebugSerial
    debugPrintln(String(F("ERROR en calibracion WCS1800 Vadc: ")) + String(zeroRead));
#endif
  }
  else
  {
    WCS1800 = true;
#if DebugSerial
    debugPrintln(String(F("Calibracion WCS1800 Vadc: ")) + String(zeroRead));
#endif
  }

  readAds1115Sum(false);
  DS1820 = readDS18B20();
  if (DS1820)
    stopEnphase = TempEnphase >= 70.0 ? true : false;

  // SSD1306_SWITCHCAPVCC = generar internamente la tensión de la pantalla a partir de 3,3V, dirección I2C 0x3C para 128x64
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    SSD1306 = true;
    diplaySsd1306(100);
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(200);
    delay(2000);
    diplaySsd1306(0);
  }
  else
  {
#if DebugSerial
    debugPrintln(String(F("Fallo con pantalla SSD1306")));
#endif
    SSD1306 = false;
  }

  power230v = detecPower();
#if DebugSerial
  if (power230v)
    debugPrintln(String(F("Detectada red electrica")));
  else
    debugPrintln(String(F("NO detectada red electrica")));
#endif
  // Tamaño máximo de paquete MQTT
  client.setBufferSize(1024);
  client.setKeepAlive(300);
  // Después de reiniciar aún se podría conserva la conexión antigua
  WiFi.disconnect();
  // No se guardar SSID y contraseña
  WiFi.persistent(false);
  // Reconectar si se pierde la conexión
  WiFi.setAutoReconnect(true);

  // Intentamos conectarnos a la red WIFI y luego conectarse al servidor MQTT
  reConnect();
  if (InitOTA())
  {
#if DebugSerial
    debugPrintln(F("[OTA] Inicializado"));
  }
  else
  {
    debugPrintln(F("[OTA] ERROR!!"));
#endif
  }
  detFlow();
  getUptime();
  InitServer();
  digitalWrite(LedPin, HIGH);
#if DebugSerial
  debugPrintln(String(F("Inicializado...\n\n")));
#endif
}

uint8_t contCicle = 0;
uint8_t contPage = 0;
uint8_t diplayPage = 1;
uint16_t cicleConnec = 0;
bool displayOnOff = true;
//************************************************************************************
// Principal
//************************************************************************************
void loop()
{
  // Reconectar si se perdió la conexión wifi o mqtt
  if (!client.connected() || WiFi.status() != 3)
  {
    reConnect();
  }
  delay(StatusRunStop == true ? 100 : 250);
  if ((millis() - ultimaConsulta) > speedData)
  {
    // Acceso a datos de Enboy Enphase
    DatosEnphase();
    ultimaConsulta = millis();
  }
  else
  {
    if (contCicle % 2 && SSD1306)
    {
      // 200ms *50 =10000
      if (++contPage == 50)
      {
        if ((radiacionSolar > 15.0 || producSolar > 49) && !displayOnOff)
        {
          // encendemos pantalla
          diplaySsd1306(20);
          displayOnOff = true;
        }
        while (!diplaySsd1306(diplayPage++) && diplayPage < 7)
        {
          diplaySsd1306(diplayPage++);
        }
        if (diplayPage == 8)
        {
          diplayPage = 1;
          if (radiacionSolar < 0.5 && producSolar < 10)
          {
            // apagamos pantalla
            diplaySsd1306(10);
            displayOnOff = false;
          }
        }
        contPage = 0;
      }
    }
    if (contCicle == 100)
    {
      if (DS1820)
        readDS18B20();
      getUptime();
    }
    if (contCicle == 150)
    {
      stopEnphase = TempEnphase >= 70.0 ? true : false;
      recirculacion(0);
    }
    if (++contCicle == 200)
    {
      if (WCS1800 && power230v)
        // leemos el consumo
        Corriente = WCSgetAC();
      contCicle = 0;
      publicMqtt();
    }
  }
  if (contHysteresis == 5 && updateData && StatusRunStop)
  {
    // detección paso por cero y NO modo AP
    if (power230v && APmode == false)
    {
      // máximo para unos 1200W
      if (dimmerPower < 100)
        ++dimmerPower;
      dimmerOnOff(dimmerPower);
      power230v = true;
#if DebugSerial
      debugPrintln(F("Incrementamos potencia.."));
#endif
    }
    // SIN tensión de red o modo AP
    else
    {
      dimmerPower = 0;
      dimmerOnOff(dimmerPower);
      contHysteresis = 0;
      power230v = false;
    }
    updateData = false;
  }
  else
  {
    if (contHysteresis == 0 && updateData && dimmerPower > 0)
    {
      // 10-100%
      if (dimmerPower > 9)
      {
        dimmerPower = uint8_t(dimmerPower / 10);
        if (dimmerPower != 1)
          dimmerPower = (dimmerPower - 1) * 10;
#if DebugSerial
        debugPrintln(F("Disminuimos potencia.."));
#endif
      }
      else
        dimmerPower = 0;
      dimmerOnOff(dimmerPower);
    }
  }
  detFlow();
  // Mantenemos activa la conexión MQTT
  client.loop();
  // escuchamos las conexiones http entrantes
  server.handleClient();
  ArduinoOTA.handle();
  // if (millis() > periodoReset)
  //   espReset(); // Reiniciamos?
}

//////////////////////////////////////////////////////////////////////////////////////
// Reseteamos ESP
//////////////////////////////////////////////////////////////////////////////////////
void espReset()
{
#if DebugSerial
  debugPrintln(F("RESET ESP8266"));
#endif
  if (client.connected())
    client.disconnect();
  WiFi.disconnect();
  ESP.restart();
  delay(5000);
}
//////////////////////////////////////////////////////////////////////////////////////
// Es necesario ejecutar SOLO una vez para obtener las direcciones de los sensores
// Address = 0x28, 0xFF, 0x64, 0x1E, 0xD, 0x9C, 0xA3, 0xB8
// Address = 0x28, 0xFF, 0x64, 0x1E, 0xD, 0x96, 0x10, 0x8F
//////////////////////////////////////////////////////////////////////////////////////
void AddrresDS18B20()
{
#ifndef DebugSerial
  Serial.begin(115200);
#endif
  byte addr[8];
  Serial.println("Obteniendo direcciones sensores DS18B20:");
  while (oneWire.search(addr))
  {
    Serial.print("Address = ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(" 0x");
      Serial.print(addr[i], HEX);
    }
    Serial.println();
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Gestión modo AP
//////////////////////////////////////////////////////////////////////////////////////
void wifiAP()
{
  WiFi.mode(WIFI_AP);
  // Canal RF 6, ISSD ON, 1 conexión
  while (!WiFi.softAP(nameAp, passAp, 6, 0, 1))
  {
    delay(100);
  }
  APmode = true;
  diplaySsd1306(8);
#if DebugSerial
  debugPrintln(String(F("Configurado modo AP, nombre: ")) + String(nameAp) + " Pass: " + String(passAp));
  debugPrintln(String(F("IP: ")) + WiFi.softAPIP().toString()); // Dirección para el AP
#endif
}
//////////////////////////////////////////////////////////////////////////////////////
// Gestión conexión/reconexión Wifi + MQTT
//////////////////////////////////////////////////////////////////////////////////////
void reConnect()
{
  uint8_t conectOK = 0;

  // Configuración en modo cliente
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, dnServer, gateway, subnet);
  WiFi.begin(ssid, password);
  // Re-intentamos conectarse a wifi si se pierde la conexión
  if (WiFi.status() != WL_CONNECTED)
  {
    locate = false;
#if DebugSerial
    debugPrintln(String(F("Conectando a: ")) + String(ssid));
#endif

    if (APmode)
      WiFi.softAPdisconnect();
    APmode = false;
    // permanecemos mientras esperamos la conexión
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
#if DebugSerial == 1
      debugPrintln(F("."));
#endif
      if (++conectOK == 100)
      {
        wifiAP();
        return;
      }
    }

    // Creamos el nombre del cliente basado en la dirección MAC y los últimos 3 bytes
    uint8_t mac[6];
    WiFi.macAddress(mac);
    clientId = "ENPHASE";
    clientId += "-" + String(mac[3], 16) + String(mac[4], 16) + String(mac[5], 16);
    clientId.toUpperCase();

#if DebugSerial == 1
    debugPrintln(String(F("WIFI: Conexión OK con IP: ")) + WiFi.localIP().toString());
    debugPrintln(String(F("WIFI: mascara de subred: ")) + WiFi.subnetMask().toString());
    debugPrintln(String(F("WIFI: gateway: ")) + WiFi.gatewayIP().toString());
    debugPrintln(String(F("WIFI: DNS: ")) + WiFi.dnsIP().toString());
    debugPrintln(String(F("WIFI: MAC ESP: ")) + WiFi.macAddress().c_str());
    debugPrintln(String(F("WIFI: HOST  http://")) + WiFi.hostname().c_str() + ".local");
    debugPrintln(String(F("WIFI: BSSID: ")) + WiFi.BSSIDstr().c_str());
    debugPrintln(String(F("WIFI: CH: ")) + WiFi.channel());
    debugPrintln(String(F("WIFI: RSSI: ")) + WiFi.RSSI());
#endif
  }
  // Conexión al broker MQTT
  // Cada mensaje MQTT puede ser enviado como un mensaje con retención (retained), en este caso cada
  // nuevo cliente que conecta a un topic recibirá el último mensaje retenido de ese tópico.
  // Cuando un cliente conecta con el Broker puede solicitar que la sesión sea persistente, en ese
  // caso el Broker almacena todas las suscripciones del cliente.
  // Un mensaje MQTT CONNECT contiene un valor keepAlive en segundos donde el cliente establece el
  // máximo tiempo de espera entre intercambio de mensajes
  // QOS
  // 0: El broker/cliente entregará el mensaje una vez, sin confirmación. (Baja/rápido)
  // 1: El broker/cliente entregará el mensaje al menos una vez, con la confirmación requerida. (Media)
  // 2: El broker/cliente entregará el mensaje exactamente una vez. (Alta/lento)
  okMqtt = false;
  conectOK = 25;
  if (WiFi.status() == WL_CONNECTED)
  {
#if DebugSerial == 1
    debugPrintln(String(F("Config MQTT: ")) + "clientID: " + clientId.c_str() + +" Broker: " + String(MQTT_SERVER) + " username: " + userMQTT + " password: " + passMQTT + " willTopic: " + String(willTopic) + " MQTT_QOS: " + String(MQTT_QOS) + " MMQTT_RETAIN: " + String(MQTT_RETAIN) + " willMessage: " + String(willMessage));
    debugPrintln("Intentando de conectar a MQTT...");
#endif
    client.disconnect();
    client.setClient(espClient);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    // Permanecemos mientras NO estemos conectados al servidor MQTT
    while (!client.connected())
    {
      //                              clientID          username  password  willTopic  willQoS,  willRetain,  willMessage,  cleanSession =1 (default)
      okMqtt = client.connect((char *)clientId.c_str(), userMQTT, passMQTT, willTopic, MQTT_QOS, MQTT_RETAIN, willMessage, 0);
      if (!okMqtt)
      {
        if (conectOK-- == 0)
          break;
        delay(2500);
        /* Respuesta a client.state()
          -4: MQTT_CONNECTION_TIMEOUT- el servidor no respondió dentro del periodo esperado
          -3: MQTT_CONNECTION_LOST- la conexión de red se interrumpió
          -2: MQTT_CONNECT_FAILED- la conexión de red falló
          -1: MQTT_DISCONNECTED- el cliente está desconectado limpiamente
          0: MQTT_CONNECTED el cliente está conectado
          1: MQTT_CONNECT_BAD_PROTOCOL el servidor no admite la versión solicitada de MQTT
          2: MQTT_CONNECT_BAD_CLIENT_ID el servidor rechazó el identificador del cliente
          3: MQTT_CONNECT_UNAVAILABLE- el servidor no pudo aceptar la conexión
          4: MQTT_CONNECT_BAD_CREDENTIALS- el nombre de usuario/contraseña NO validos
          5: MQTT_CONNECT_UNAUTHORIZED- el cliente no estaba autorizado para conectarse
        */

#if DebugSerial
        debugPrintln(String(F("Fallo al conectar al broker, rc=")) + String(client.state()) + " intentando conectar en 5 segundos, " + String(conectOK));
#endif
      }
    }
    if (okMqtt)
    {
      publicMqtt();
      // Suscripción a topics habilitar - deshabilitar control dimmer
      const String topicOnOff = clientId + "/on_off/#";
      client.subscribe((char *)topicOnOff.c_str(), 0);
      // Suscripción a topics marcha paro manual del control dimmer
      const String topicRunOnOff = clientId + "/run_on_off/#";
      client.subscribe((char *)topicRunOnOff.c_str(), 0);
      // Suscripción a topics habilitar - deshabilitar bomba recirculación
      const String topicBomba = clientId + "/bomba/#";
      client.subscribe((char *)topicBomba.c_str(), 0);
      // Suscripción a topics topicPowerDimmer, QoS 0
      const String topicPowerDimmer = clientId + "/power/#";
      client.subscribe((char *)topicPowerDimmer.c_str(), 0);
      // Suscripción a topics topicTemperatura1 termo 1, QoS 0
      const String topicTemperatura1 = clientId + "/tempEnphase/#";
      client.subscribe((char *)topicTemperatura1.c_str(), 0);
      // Suscripción a topics topicTemperatura2 termo 2, QoS 0
      const String topicTemperatura2 = clientId + "/tempHome/#";
      client.subscribe((char *)topicTemperatura2.c_str(), 0);
      // Suscripción a topics topicCmd comandos (cmd), QoS 0
      const String topicCmd = clientId + "/cmd/#";
      client.subscribe((char *)topicCmd.c_str(), 0);
      // Mensaje a enviar en caso de:
      // Un error I/O o fallo de red detectado por el servidor.
      // Un cliente falla al comunicarse dentro del intervalo Keep Alive configurado.
      // Un cliente cierra la conexión de red sin primero enviar el paquete DISCONNECT.
      // El servidor cierra la conexión de red debido a un error de protocolo.
      //  RETAIN ON
      const String topicLWT = clientId + "/LWT";
      client.publish((char *)topicLWT.c_str(), (char *)"Online", true);

#if DebugSerial
      debugPrintln("Conectado a MQTT");
      debugPrintln("Publicación de Topics enviada");
      debugPrintln(String(F("Subscrito a:")) + topicOnOff);
      debugPrintln(String(F("Subscrito a:")) + topicRunOnOff);
      debugPrintln(String(F("Subscrito a:")) + topicBomba);
      debugPrintln(String(F("Subscrito a:")) + topicPowerDimmer);
      debugPrintln(String(F("Subscrito a:")) + topicTemperatura1);
      debugPrintln(String(F("Subscrito a:")) + topicTemperatura2);
      debugPrintln(String(F("Subscrito a:")) + topicCmd);
      debugPrintln("Suscripción a topics enviada");
#endif
    }
    else
    {
      espReset();
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Barra de progreso para pantalla OLED, coordenada x , y, anchura, altura, valor de 0 a 100
//////////////////////////////////////////////////////////////////////////////////////
void drawProgressbar(int x, int y, int width, int height, int progress)
{
  progress = progress > 100 ? 100 : progress;
  progress = progress < 0 ? 0 : progress;

  float bar = ((float)(width - 1) / 100) * progress;

  display.drawRect(x, y, width, height, WHITE);
  display.fillRect(x + 2, y + 2, bar, height - 4, WHITE);

  // Mostrar el texto de progreso
  if (height >= 15)
  {
    display.setCursor((width / 2) - 3, y + 5);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    if (progress >= 50)
      // 'inverted' text
      display.setTextColor(BLACK, WHITE);
    display.print(progress);
    display.print("%");
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Gestión pantalla OLED
// display.setTextSize(1)->ocho lineas de altura de 20 caracteres de longitud y 8 de alto
// display.setCursor(anchura, altura);
//////////////////////////////////////////////////////////////////////////////////////
bool diplaySsd1306(uint8_t page)
{
  display.clearDisplay();
  char stringDisplay[9];
  switch (page)
  {
  case 0:
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(25, 0);
    display.print(String(F("TERMO")));
    display.setCursor(15, 16);
    display.print(String(F("Enphase")));
    display.setCursor(15, 40);
    display.print(String(F("V. ")) + String(sofVersion));
    break;
  case 1:
    if (consumo == 0.0)
      return (false);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(String(F("-CONSUMOS-")));
    if (consumoRed == 1 || consumoRed == 0)
      dtostrf(consumoRed, 1, 0, stringDisplay);
    else
      dtostrf(consumoRed, 5, 2, stringDisplay);
    display.setTextSize(1);
    display.setCursor(55, 16);
    display.println(String(F("RED")));
    display.setTextSize(2);
    display.print(String(stringDisplay) + String(F("w")));
    if (consumo == 1 || consumo == 0)
      dtostrf(consumo, 1, 0, stringDisplay);
    else
      dtostrf(consumo, 5, 2, stringDisplay);
    display.setCursor(50, 38);
    display.setTextSize(1);
    display.println(String(F("CASA")));
    display.setTextSize(2);
    display.print(String(stringDisplay) + String(F("w")));
    break;
  case 2:
    if (producSolar == 0.0)
      return false;
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 0);
    display.print(String(F("-SOLAR-")));
    display.setCursor(5, 20);
    display.println(String(producSolar) + String(F("w")));
    drawProgressbar(0, 46, 127, 15, map(producSolar, 0, 1600, 0, 100));
    break;
  case 3:
    if (!DS1820)
      return false;
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(String(F("TEMP. AGUA")));
    display.setTextSize(1);
    display.setCursor(44, 16);
    display.println(String(F("-Solar-")));
    if (TempEnphase == 1 || TempEnphase == 0)
      dtostrf(TempEnphase, 1, 0, stringDisplay);
    else
      dtostrf(TempEnphase, 5, 2, stringDisplay);
    display.setTextSize(2);
    display.print(String(stringDisplay));
    display.setTextSize(1);
    display.print((char)248);
    display.setTextSize(2);
    display.print(String(F("C")));
    display.setTextSize(1);
    display.setCursor(48, 38);
    display.println(String(F("-Casa-")));
    if (TempHome == 1 || TempHome == 0)
      dtostrf(TempHome, 1, 0, stringDisplay);
    else
      dtostrf(TempHome, 5, 2, stringDisplay);
    display.setTextSize(2);
    display.print(String(stringDisplay));
    display.setTextSize(1);
    display.print((char)248);
    display.setTextSize(2);
    display.print(String(F("C")));
    break;
  case 4:
    if (Corriente < 0.5)
      return false;
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 0);
    display.print(String(F("CORRIENTE")));
    if (Corriente == 1 || Corriente == 0)
      dtostrf(Corriente, 1, 0, stringDisplay);
    else
      dtostrf(Corriente, 5, 2, stringDisplay);
    display.setCursor(0, 26);
    display.print(String(stringDisplay) + String(F("A")));
    drawProgressbar(0, 46, 127, 15, map(Corriente, 0, 5, 0, 100));
    break;
  case 5:
    if (!StatusRunStop)
      return false;
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(String(F("REGULACION")));
    display.setCursor(0, 16);
    if (dimmerPower == 0)
    {
      display.println(String(F("--PARADA--")));
    }
    else
    {
      display.print(String(stringDisplay) + String(F("%")));
      dtostrf(dimmerPower, 1, 0, stringDisplay);
      drawProgressbar(0, 46, 127, 15, dimmerPower);
    }
    break;
  case 6:
    if (radiacionSolar < 5.0)
      return false;
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(String(F("RADIACION")));
    display.setCursor(0, 16);
    dtostrf(radiacionSolar, 1, 0, stringDisplay);
    display.print(String(stringDisplay) + String(F("Wm2")));
    drawProgressbar(0, 46, 127, 15, map(radiacionSolar, 0, 1000, 0, 100));
    break;
  case 7:
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 0);
    display.println(String(F("INFO. IP")));
    display.print(String(F("IP:")));
    display.setTextSize(1);
    display.println(WiFi.localIP().toString());
    display.setTextSize(2);
    display.setCursor(0, 32);
    display.print(String(F("RSSI:")));
    display.setTextSize(1);
    dtostrf(WiFi.RSSI(), 1, 0, stringDisplay);
    display.println(String(stringDisplay) + String(F("%")));
    display.setCursor(0, 48);
    display.setTextSize(2);
    if (locate)
      display.print(String(F("ON line")));
    else
      display.print(String(F("OFF line")));
    break;
  case 8:
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 0);
    display.println(String(F("MODO AP")));
    display.print(String(F("IP:")));
    display.setTextSize(1);
    display.println(WiFi.softAPIP().toString());
    display.setTextSize(2);
    display.setCursor(0, 32);
    display.print(String(F("PASS: ")));
    display.println(String(passAp));
    break;
  case 10:
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    break;
  case 20:
    display.ssd1306_command(SSD1306_DISPLAYON);
    break;
  default:
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.cp437(true);
    for (int16_t i = 0; i < 256; i++)
    {
      if (i == '\n')
        display.write(' ');
      else
        display.write(i);
    }
    break;
  }
  display.display();
  return true;
}
//************************************************************************************
// Lecturas entrada digital de detección de flujo de agua entre termos (home->Enphase)
//************************************************************************************
void detFlow()
{
  static bool oldStatusFlow = 0;
  oldStatusFlow = bool(digitalRead(detecFlowPin));
  if (oldStatusFlow != StatusFlow)
  {
    StatusFlow = oldStatusFlow;
    mqttSend("/flujo", StatusFlow ? (char *)"false" : (char *)"true");
  }
}
//************************************************************************************
// Lecturas señal analógica para obtener la radiación solar
//************************************************************************************
void readAds1115Sum(bool offSet)
{
  uint16_t adcSun;
  if (ADS1115)
  {
    // WCS37A50(módulo 0.5A) mirar https://solarduino.com/diy-irradiation-meter-with-arduino/
    float mVperAmpValue = 3500.0;
    // 3300mV
    float moduleSupplyVoltage = 3300.0;
    // la lectura actual final sin tomar el valor de compensación
    float finalCurrent;
    // 0.5V 270mA, corriente de cortocircuito (en condición STC) del panel solar
    float ShortCircuitCurrentSTC = 0.270;
    // 16 bits para ads1115 - 1bit signo->15bits->0-32767
    float resolutionAdc = 32767.0;

    if (offSet == 1)
    {
      // para contador de muestras
      float offsetSampleCount = 0;
      // acumulación de lecturas de muestra
      float SampleSum = 0;
      while (offsetSampleCount++ < 500)
      {
        adcSun = ads.readADC_SingleEnded(0);
        SampleSum = SampleSum + adcSun;
      }
      // obtenemos la media de todas las lecturas (ultimo 14145)
      adcOffset = SampleSum / offsetSampleCount;
#if DebugSerial
      debugPrintln(String(F("OffSet en reposo (SIN radiacion): ")) + String(adcOffset));
#endif
    }
    else
    {
      adcSun = ads.readADC_SingleEnded(0);
      ADC0 = adcSun;
      if (adcSun <= adcOffset)
      {
        radiacionSolar = 0.0;
      }
      else
      {
        // obtenemos la corriente final, sin offset-(compensación)
        finalCurrent = float((((adcSun - adcOffset) / resolutionAdc) * moduleSupplyVoltage) / mVperAmpValue);
        radiacionSolar = (finalCurrent / ShortCircuitCurrentSTC * 1000.0);
        if (radiacionSolar < 3.9)
          radiacionSolar = 0.0;
        if (radiacionSolar > 1000.0)
          radiacionSolar = 1000.0;
      }
#if DebugSerial
      debugPrintln(String(F("Radiacion solar: ")) + String(radiacionSolar) + "W/m2");
#endif
    }
  }
}
//************************************************************************************
// Leemos los dos sensores DS18B20 agua de los termos termo-enphase y termo-casa
//************************************************************************************
bool readDS18B20()
{
  if (numberOfDevices != 2)
  {
#if DebugSerial
    debugPrintln(String(F("ERROR DS18B20 encontrados: ")) + numberOfDevices);
#endif
    return false;
  }
  sensorsDS18B20.requestTemperatures();
  if (sensorsDS18B20.getAddress(tempDeviceAddress_1, 0))
  {
    // Obtenemos la temperatura del termo Enphase
    TempEnphase = sensorsDS18B20.getTempC(tempDeviceAddress_1);
#if DebugSerial
    debugPrintln(String(F("Temperatura termo Enphase: ")) + TempEnphase);
#endif
    if (manualRun == true)
    {
      if (sensorsDS18B20.getAddress(tempDeviceAddress_2, 1))
      {
        // Obtenemos la temperatura del segundo termo (casa)
        TempHome = sensorsDS18B20.getTempC(tempDeviceAddress_2);
#if DebugSerial
        debugPrintln(String(F("Temperatura termo casa: ")) + TempHome);
#endif
        return true;
      }
    }
    if (TempEnphase > maxTempEnphase || radiacionSolar == 0.0)
    {
      if (StatusRunStop == true)
      {
        StatusRunStop = false;
        mqttSend("/on_off", (char *)"0");
      }
#if DebugSerial
      if (radiacionSolar == 0.0)
        debugPrintln(String(F("PARAMOS por baja radiacion")));
      if (TempEnphase > maxTempEnphase)
        debugPrintln(String(F("PARAMOS por temperatura termo Enphase maxima")));
#endif
    }
    else if (TempEnphase < (maxTempEnphase - 5.0) && radiacionSolar > 15.0)
    {
      if (StatusRunStop == false)
      {
        StatusRunStop = true;
        mqttSend("/on_off", (char *)"1");
      }
#if DebugSerial
      debugPrintln(String(F("INICIAMOS regulación, radiacion de: ") + String(radiacionSolar) + "Wm2"));
#endif
    }
  }
  else
  {
    return false;
  }
  if (sensorsDS18B20.getAddress(tempDeviceAddress_2, 1))
  {
    // Obtenemos la temperatura del segundo termo (casa)
    TempHome = sensorsDS18B20.getTempC(tempDeviceAddress_2);
#if DebugSerial
    debugPrintln(String(F("Temperatura termo casa: ")) + TempHome);
#endif
  }
  else
  {
    return false;
  }
  return true;
}
//************************************************************************************
// Gestión bomba recirculación
//************************************************************************************
void recirculacion(bool force)
{
  static int8_t countRec = 0;
  if (force == true)
  {
    countRec = 0;
    StatusRunBomba = 0;
    // relé OFF->3v3
    digitalWrite(relayPumpPin, 1);
    return;
  }
  if (StatusRunBomba)
  {
    // 3*20=60seg.
    if (countRec++ == 3)
    {
      // 126*20=2520seg.->42minutos
      countRec = 126;
      // relé OFF->3v3
      digitalWrite(relayPumpPin, 1);
      StatusRunBomba = 0;
      mqttSend("/bomba", (char *)"0");
    }
  }
  else if (DS1820)
  {
    readDS18B20();
    if (countRec-- == 0)
    {
      if (TempEnphase > (TempHome + 10) && dimmerPower == 0)
      {
        // relé ON->gnd
        digitalWrite(relayPumpPin, 0);
        StatusRunBomba = 1;
        mqttSend("/bomba", (char *)"1");
      }
      else
        countRec = 1;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Acceso a través de http al envoy de Enphase para la lectura de datos
//////////////////////////////////////////////////////////////////////////////////////
void DatosEnphase()
{
  static uint8_t reponseEnphase = 0;
#if DebugSerial == 1
  if (dimmerPower)
    debugPrintln(String(F("Dimmer a: ")) + String(dimmerPower) + "%");
  debugPrintln(F("[HTTP] Enphase"));
#endif
  WiFiClient httpClient;
  HTTPClient clienteHttp;
  String url = "http://envoy.local/production.json";
  // Conexión con el servidor y configuración de la petición
  if (!clienteHttp.begin(httpClient, url))
  {
#if DebugSerial
    debugPrintln(F("[HTTP] No puede conectarse al Envoy"));
#endif
    return;
  }
#if DebugSerial == 1
  debugPrintln(F("[HTTP] GET..."));
#endif
  // OFF software watchdog (NO más de 6 segundos)
  // system_soft_wdt_stop();
  system_soft_wdt_feed();
  int codigoHttp = clienteHttp.GET();
  if (codigoHttp == HTTP_CODE_OK || codigoHttp == HTTP_CODE_MOVED_PERMANENTLY)
  {
#if DebugSerial == 1
    debugPrintln(F("Respuesta Enphase JSON correcta"));
#endif
    const size_t capacity = 2048;
    DynamicJsonDocument doc(capacity);
    // Recorremos todos los registros de una base de datos objeto JSON
    DeserializationError error = deserializeJson(doc, clienteHttp.getString());
    // Si hay error no se continua
    if (error)
    {
#if DebugSerial
      debugPrintln(String(F("Fallo al recorrer los registros de la base de datos JSON. Error: ")) + String(error.c_str()));
#endif
      clienteHttp.end();
      // ON software watchdog
      // system_soft_wdt_feed();
      // system_soft_wdt_restart();
      return;
    }
    // obtenemos datos
    producSolar = doc["production"][1]["wNow"];
    char produc_Solar[8];
    snprintf(produc_Solar, 8, "%.1f", producSolar);
    consumo = doc["consumption"][0]["wNow"];
    char consumo_total[8];
    snprintf(consumo_total, 8, "%.1f", consumo);
    consumoRed = doc["consumption"][1]["wNow"];
    char consumo_Red[8];
    snprintf(consumo_Red, 8, "%.1f", consumoRed);

    clienteHttp.end();
    // ON software watchdog
    // system_soft_wdt_feed();
    // system_soft_wdt_restart();

    readAds1115Sum(false);
    // producción mayor de 100 y menor de 1300W, consumo de red menor de 45W y permiso StatusRunStop a 1 y temperatura termo Enphase menor de 70 grados
    if ((producSolar > 100 && producSolar < 1300) && consumoRed < 46.0 && StatusRunStop == true && TempEnphase < maxTempEnphase)
    {
      digitalWrite(LedPin, LOW);
      reponseEnphase = 0;
      speedData = 3000;
      if (consumoRed < 0.0)                                                           // lectura negativa?
      {
        contHysteresis = 5;
        consumoRed = 0.0;
      }
      if (contHysteresis != 5)
      {
        contHysteresis++;
      }
      else
      {
        updateData = true;
#if DebugSerial
        debugPrintln(F("Tenemos excedentes.."));
#endif
      }
    }
    else
    {
      // Máximo consumo permitido de red 100W
      if (consumoRed > 100.0)
      {
        // permitimos 5 ciclos con lecturas superiores a 100W y menores a 250W (tiempo respuesta micro-inversores)
        if (consumoRed < 250.0 && reponseEnphase < 5)
        {
            reponseEnphase++;
        }
        else
        {
          // STOP
          reponseEnphase = 0;
          dimmerPower = 0;
          dimmerOnOff(dimmerPower);
          contHysteresis = 0;
          // Control parado?
          if (StatusRunStop)
          {
            speedData = 60000;
          }
          else
          {
            speedData = 20000;
          }
#if DebugSerial
          debugPrintln(F("Consumo RED MUY elevado, paramos potencia.."));
#endif
        }
      }
      else
      {
        reponseEnphase = 0;
        // disminución potencia dimmer
        if (consumoRed > 65.0)
        {
          digitalWrite(LedPin, LOW);
          if (contHysteresis != 0)
          {
            --contHysteresis;
            delay(250);
          }
          else
          {
            speedData = 5000;
          }
#if DebugSerial
          debugPrintln(F("Consumo RED elevado, disminuimos potencia.."));
#endif
        }
      }
      digitalWrite(LedPin, HIGH);
    }

#if DebugSerial == 1
    debugPrintln(String(F("Producion Solar: ")) + String(producSolar) + "W");
    debugPrintln(String(F("Consumo total casa: ")) + String(consumo_total) + "W");
    debugPrintln(String(F("Consumo de compañia: ")) + String(consumo_Red) + "W");
    debugPrintln(String(F("Hísterisis regulación: ")) + String(contHysteresis));
#endif
  }
#if DebugSerial
  else
  {
    debugPrintln(String(F("Error: ")) + String(codigoHttp) + " al recibir petición");
  }
#endif
}
//************************************************************************************
// Dimmer control
//************************************************************************************
void dimmerOnOff(uint8_t Power)
{
  static bool isrOn = false;
  static uint32_t oldtimeoutPin = 0;

  if (!StatusRunStop || stopEnphase)
    // Control deshabilitado
    Power = 0;
#if DebugSerial == 1
  debugPrintln(String(F("Orden Dimmer a: ")) + String(Power) + "%");
  debugPrintln(String(F("Tensión de red -paso por cero-: ")) + String(power230v));
#endif
  if (Power)
  {
    // Detección red eléctrica?
    if (!power230v)
      return;

    // al 100%
    if (Power > 99)
    {
      if (isrOn)
      {
        detachInterrupt(zerocrossPin);
        timer1_disable();
        timer1_detachInterrupt();
        dimmerRun = 0;
        isrOn = false;
        digitalWrite(LedPin, LOW);
      }
      digitalWrite(outputPin, HIGH);
    }
    else
    {
      if (Power < 10)
        // medio periodo ON (10ms cada 10 periodos->200ms)
        timeoutPin = 75000;
      else
        // Calculamos el periodo pulso a ON según el porcentaje
        timeoutPin = (100000 * (Power / 10)) + 75000;
      if (!isrOn)
      {
        timer1_attachInterrupt(onTimerISR);
        timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
        // espera disparo pulso 1-9% 15ms, 10%->35ms-99%-> 175ms
        timer1_write(timeoutPin);
        dimmerRun = 0;
        // ISR por paso de low a high
        attachInterrupt(digitalPinToInterrupt(zerocrossPin), onPinISR, RISING);
        isrOn = true;
      }
    }
  }
  // STOP
  else
  {
    detachInterrupt(zerocrossPin);
    timer1_disable();
    timer1_detachInterrupt();
    digitalWrite(outputPin, LOW);
    dimmerRun = 0;
    isrOn = false;
  }
  // hay cambio en la regulación, acualizamos por MQTT
  if (timeoutPin != oldtimeoutPin)
  {
    if (contCicle < 180)
      publicMqtt();
    oldtimeoutPin = timeoutPin;
  }
}
//////////////////////////////////////////////////////////////////////////////////////
// Detección tensión red en pin zerocrossPin (≈1ms duración pulso a 1 en paso por cero)
//////////////////////////////////////////////////////////////////////////////////////
bool detecPower()
{
  uint8_t Count = 0;
  if (digitalRead(zerocrossPin) == 0)
    return true;
  do
  {
    delayMicroseconds(50);
    if (digitalRead(zerocrossPin) == 0)
      return true;
    yield();
  } while (Count++ < 21);
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////
// Control manual On OFF (ON 5 minutos al 50% +-2A)
//////////////////////////////////////////////////////////////////////////////////////
void controlOnOFF(bool OnOFF)
{
  uint8_t x = 0;
  if (OnOFF)
  {
    StatusRunStop = true;
    dimmerPower = 50;
    dimmerOnOff(dimmerPower);
    manualRun = true;
#if DebugSerial
    debugPrintln(String(F("Potencia manual al: ")) + String(dimmerPower) + "%");
#endif
    do
    {
      delay(500);
      readDS18B20();
      delay(500);
      getUptime();
      delay(500);
      client.loop();
      server.handleClient();
      ArduinoOTA.handle();
      WCSgetAC();
      delay(500);
      publicMqtt();
      delay(500);
      if (StatusRunStop == false || TempEnphase > maxTempEnphase)
        x = 150;
    } while (x++ < 120);
  }
  StatusRunStop = false;
  dimmerOnOff(0);
  manualRun = false;
}
//////////////////////////////////////////////////////////////////////////////////////
// Obtenemos el periodo en marcha en segundos
//////////////////////////////////////////////////////////////////////////////////////
void getUptime()
{
  static unsigned long last_uptime = 0;
  static unsigned char uptime_overflows = 0;
  static uint8_t contMqtt = 2;
  String message;
  uint16_t messageLeng;
  if (millis() < last_uptime)
    ++uptime_overflows;
  last_uptime = millis();
  unsigned long uptime_seconds = uptime_overflows * (UPTIME_OVERFLOW / 1000) + (last_uptime / 1000);
  // Reset cada dos dias
  // if (uptime_seconds > 172799) espReset();
  String GetT = String(uptime_seconds);

  if (++contMqtt == 3)
  {
    message = String(F("/sensor/uptime/state"));
    messageLeng = message.length();
    char msg_t[messageLeng];
    message.toCharArray(msg_t, (messageLeng + 1));
    mqttSend(msg_t, (char *)GetT.c_str());
    contMqtt = 0;
  }
}
#if DebugSerial
//////////////////////////////////////////////////////////////////////////////////////
// Enviamos trama de debugger
//////////////////////////////////////////////////////////////////////////////////////
void debugPrintln(String debugText)
{
  // si se encuentra regulando  NO debugamos
  if (dimmerPower > 0 && dimmerPower < 100)
    return;
  yield();
  noInterrupts();
#if DebugSerial == 1
  String debugTimeText = "[+" + String(float(millis()) / 1000, 3) + "s] " + debugText;
  Serial.println(debugTimeText);
#else
  Serial.println(debugText);
#endif
  interrupts();
  yield();
  // Esperar hasta que el uart emita la interrupción TXComplete
  Serial.flush();
}

#endif
