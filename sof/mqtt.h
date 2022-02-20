#ifndef _MQTT_H_
#define _MQTT_H_

// MQTT
// Dirección IP del servidor MQTT
const char *MQTT_SERVER = "192.168.xxx.xxx";
// MQTT puerto broker
const uint16_t MQTT_PORT = 1883;
const char *userMQTT = "xxxxx";
const char *passMQTT = "*****";
String clientId = "ENPHASE";
// MQTT bandera retain
bool MQTT_RETAIN = false;
// MQTT QoS mensajes
uint8_t MQTT_QOS = 0;
// MQTT willTopic
const char *willTopic = 0;
// MQTT willMessage
const char *willMessage = 0;

void reConnect();
void debugPrintln(String debugText);
void espReset();
void callback(char *topic, byte *payload, unsigned int length);

// Instancia a objetos
WiFiClient espClient;

PubSubClient client(MQTT_SERVER, MQTT_PORT, callback, espClient);

void controlOnOFF(bool);

//////////////////////////////////////////////////////////////////////////////////////
// Publicamos en el broker MQTT
//////////////////////////////////////////////////////////////////////////////////////
void mqttSend(String topic, char *topublish)
{
    if (client.connected())
    {
        topic = clientId + topic;
        client.publish((char *)topic.c_str(), topublish, MQTT_RETAIN);
#if DebugSerial == 1
        debugPrintln(String(F("Publicado topic: ")) + topic + " valor: " + topublish);
#endif
    }
    else
    {
#if DebugSerial
        debugPrintln(String(F("Fallo en conexión con el broker, rc=")) + String(client.state()) + " intentando RE-conectar");
#endif
        reConnect();
    }
}

//////////////////////////////////////////////////////////////////////////////////////
// Publicación MQTT
//////////////////////////////////////////////////////////////////////////////////////
void publicMqtt()
{
    static uint8_t contLocate = 0;
    char payload[6];
    if (contLocate % 3 == 0 || locate == false)
    {
        mqttSend("/status", (char *)"1");
        mqttSend("/app", (char *)"Radioelf");
        mqttSend("/version", (char *)sofVersion);
        mqttSend("/board", (char *)"ENPHASE1_RADIOELF");
        mqttSend("/host", (char *)clientId.c_str());
        mqttSend("/desc", (char *)"ENPHASE_TERMO");
        mqttSend("/ssid", (char *)ssid);
        mqttSend("/ip", (char *)WiFi.localIP().toString().c_str());
        mqttSend("/mac", (char *)WiFi.macAddress().c_str());
        snprintf(payload, 5, "%d", WiFi.RSSI());
        mqttSend("/rssi", payload);
        snprintf(payload, 6, "%d", ESP.getVcc());
        mqttSend("/vcc", payload);
        // solo obtenemos 45 caracteres de la respuesta de ESP.getResetInfo()
        String infoReset = ESP.getResetInfo().substring(0, 45);
        mqttSend("/reset", (char *)infoReset.c_str());
    }
    String statusTx = "offline";
    if (locate)
    {
        statusTx = "online";
        char string_mqtt[9];
        if (TempEnphase == 1 || TempEnphase == 0)
            dtostrf(TempEnphase, 1, 0, string_mqtt);
        else
            dtostrf(TempEnphase, 5, 2, string_mqtt);
        mqttSend("/sensor/temperatura_1/state", string_mqtt);

        if (TempHome == 1 || TempHome == 0)
            dtostrf(TempHome, 1, 0, string_mqtt);
        else
            dtostrf(TempHome, 5, 2, string_mqtt);
        mqttSend("/sensor/temperatura_2/state", string_mqtt);

        if (Corriente == 1 || Corriente == 0)
            dtostrf(Corriente, 1, 0, string_mqtt);
        else
            dtostrf(Corriente, 5, 2, string_mqtt);
        mqttSend("/sensor/consumo/state", string_mqtt);

        snprintf(payload, 5, "%d", dimmerPower);
        mqttSend("/sensor/potencia/state", payload);

        uint16_t powerW;
        dimmerPower != 0 ? powerW = tablePower[dimmerPower / 10] : powerW = 0;
        if (powerW == 1 || powerW == 0)
            dtostrf(powerW, 1, 0, string_mqtt);
        else
            dtostrf(powerW, 5, 2, string_mqtt);
        mqttSend("/sensor/potenciaw/state", string_mqtt);

        if (radiacionSolar == 1 || radiacionSolar == 0)
            dtostrf(radiacionSolar, 1, 0, string_mqtt);
        else
            dtostrf(radiacionSolar, 5, 2, string_mqtt);
        mqttSend("/sensor/radiacion/state", string_mqtt);

        mqttSend("/on_off", StatusRunStop ? (char *)"1" : (char *)"0");

        mqttSend("/bomba", StatusRunBomba ? (char *)"1" : (char *)"0");

        mqttSend("/flujo", StatusFlow ? (char *)"false" : (char *)"true");

        if (producSolar == 1 || producSolar == 0)
            dtostrf(producSolar, 1, 0, string_mqtt);
        else
            dtostrf(producSolar, 5, 2, string_mqtt);
        mqttSend("/sensor/solar/state", string_mqtt);

        if (consumo == 1 || consumo == 0)
            dtostrf(consumo, 1, 0, string_mqtt);
        else
            dtostrf(consumo, 5, 2, string_mqtt);
        mqttSend("/sensor/casa/state", string_mqtt);

        if (consumoRed == 1 || consumoRed == 0)
            dtostrf(consumoRed, 1, 0, string_mqtt);
        else
            dtostrf(consumoRed, 5, 2, string_mqtt);
        mqttSend("/sensor/red/state", string_mqtt);

        String GetAdc = String(ADC0);
        mqttSend("/adc0", (char *)GetAdc.c_str());
    }
    mqttSend("/status", (char *)statusTx.c_str());

    if (++contLocate == 120)
    {
        contLocate = 0;
        locate = false;
    }
    if (!locate)
    {
        // Auto localización para Home Assistant <discovery_prefix>/<component>/[<node_id>/]<object_id>/config
        String message = "{";
        message += String(F("\"name\":\"ENPHASE_TERMO")) + String(F("\",\""));
        message += String(F("icon\":\"mdi:power-socket-eu\",\""));
        message += String(F("state_topic\":\"")) + String(clientId) + String(F("/on_off")) + String(F("\",\""));
        message += String(F("command_topic\":\"")) + String(clientId) + String(F("/on_off")) + String(F("/set\",\""));
        message += String(F("payload_on\":\"1\",\"payload_off\":\"0\",\""));
        message += String(F("availability_topic\":\"")) + String(clientId) + String(F("/status\",\""));
        message += String(F("payload_available\":\"1\",\"payload_not_available\":\"0\",\""));
        message += String(F("uniq_id\":\"")) + String(clientId) + String(F("TermoSwitch")) + String(F("\",\""));
        message += String(F("device\":{\"identifiers\":[\"")) + String(clientId) + String(F("\"],\""));
        message += String(F("name\":\"TermoEnphase\",\"sw_version\":\"ENPHASE ")) + String(sofVersion) + String(F("\",\"manufacturer\":\"Radioelf\",\"model\":\"ENPHASE1\"}"));
        message += "}";

        String topic = ("homeassistant/switch/" + clientId + "/config");
        uint16_t messageLeng = message.length();
        char msg[messageLeng];
        message.toCharArray(msg, (messageLeng + 1));
        client.publish((char *)topic.c_str(), msg, messageLeng);

        message = "{";
        message += String(F("\"name\":\"BOMBA_RECIRCULACION")) + String(F("\",\""));
        message += String(F("icon\":\"mdi:pump\",\""));
        message += String(F("state_topic\":\"")) + String(clientId) + String(F("/bomba")) + String(F("\",\""));
        message += String(F("command_topic\":\"")) + String(clientId) + String(F("/bomba")) + String(F("/set\",\""));
        message += String(F("payload_on\":\"1\",\"payload_off\":\"0\",\""));
        message += String(F("availability_topic\":\"")) + String(clientId) + String(F("/status\",\""));
        message += String(F("payload_available\":\"1\",\"payload_not_available\":\"0\",\""));
        message += String(F("uniq_id\":\"")) + String(clientId) + String(F("TermoSwitch2")) + String(F("\",\""));
        message += String(F("device\":{\"identifiers\":[\"")) + String(clientId) + String(F("\"],\""));
        message += String(F("name\":\"TermoEnphase\",\"sw_version\":\"ENPHASE ")) + String(sofVersion) + String(F("\",\"manufacturer\":\"Radioelf\",\"model\":\"ENPHASE1\"}"));
        message += "}";

        String topic_Bomba = ("homeassistant/switch/" + clientId + "_bomba/config");
        uint16_t messageLeng_Bomba = message.length();
        char msg_Bomba[messageLeng_Bomba];
        message.toCharArray(msg_Bomba, (messageLeng_Bomba + 1));
        client.publish((char *)topic_Bomba.c_str(), msg_Bomba, messageLeng_Bomba);

        message = "{";
        message += String(F("\"stat_t\":\"")) + String(clientId) + String(F("/status\",\""));
        message += String(F("name\":\"SYS: Connectivity_")) + String(clientId) + String(F("\",\""));
        message += String(F("unique_id\":\"")) + String(clientId) + String(F("_connectivity\",\""));
        message += String(F("dev_cla\":\"connectivity\",\"pl_on\":\"online\",\"pl_off\":\"offline\",\""));
        message += String(F("pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\""));
        message += String(F("device\":{\"identifiers\":\"")) + String(clientId) + String(F("\",\""));
        message += String(F("name\":\"TermoEnphase\",\"sw_version\":\"ENPHASE ")) + String(sofVersion) + String(F("\",\"manufacturer\":\"Radioelf\",\"model\":\"ENPHASE1\"}"));
        message += "}";

        String topic_Conect = ("homeassistant/binary_sensor/" + clientId + "_connectivity/config");
        uint16_t messageLeng_Status = message.length();
        char msg_Status[messageLeng_Status];
        message.toCharArray(msg_Status, (messageLeng_Status + 1));
        client.publish((char *)topic_Conect.c_str(), msg_Status, messageLeng_Status);

        message = "{";
        message += String(F("\"stat_t\":\"")) + String(clientId) + String(F("/binary_sensor_flujo\",\""));
        message += String(F("name\":\"Flujo agua\",\""));
        message += String(F("unique_id\":\"")) + String(clientId) + String(F("_flujo\",\""));
        // message += String(F("dev_cla\":\"flujo\",\"pl_on\":\"true\",\"pl_off\":\"false\",\""));
        message += String(F("pl_avail\":\"true\",\"pl_not_avail\":\"false\",\""));
        message += String(F("device\":{\"identifiers\":\"")) + String(clientId) + String(F("\",\""));
        message += String(F("name\":\"TermoEnphase\",\"sw_version\":\"ENPHASE ")) + String(sofVersion) + String(F("\",\"manufacturer\":\"Radioelf\",\"model\":\"ENPHASE1\"}"));
        message += "}";

        String topic_Rec = ("homeassistant/binary_sensor/" + clientId + "/flujo/config");
        uint16_t messageLeng_Rec = message.length();
        char msg_Rec[messageLeng_Rec];
        message.toCharArray(msg_Rec, (messageLeng_Rec + 1));
        client.publish((char *)topic_Rec.c_str(), msg_Rec, messageLeng_Rec);

        uint8_t sensor = 10;
        while (sensor--) // 0-9
        {
            String type, name, uniqId, unit, icon;
            switch (sensor)
            {
            case 1:
                type = "temperatura_1";
                name = "Temperatura 1";
                uniqId = "TermoTemperatura1";
                unit = "°C";
                icon = "thermometer";
                break;
            case 2:
                type = "temperatura_2";
                name = "Temperatura 2";
                uniqId = "TermoTemperatura2";
                unit = "°C";
                icon = "thermometer";
                break;
            case 3:
                type = "potencia";
                name = "Control Potencia";
                uniqId = "TermoPotencia";
                unit = "%";
                icon = "car-cruise-control";
                break;
            case 4:
                type = "consumo";
                name = "Consumo Termo";
                uniqId = "TermoConsumo";
                unit = "A";
                icon = "sine-wave";
                break;
            case 5:
                type = "red";
                name = "Consumo Red";
                uniqId = "TermoConsumoRed";
                unit = "W";
                icon = "sine-wave";
                break;
            case 6:
                type = "casa";
                name = "Consumo Casa";
                uniqId = "TermoCasa";
                unit = "W";
                icon = "sine-wave";
                break;
            case 7:
                type = "solar";
                name = "Produccion Solar";
                uniqId = "TermoSolar";
                unit = "W";
                icon = "sine-wave";
                break;
            case 8:
                type = "radiacion";
                name = "Radiacion Solar";
                uniqId = "RadiacionSolar";
                unit = "w/m2";
                icon = "solar-power";
                break;
            case 9:
                type = "potenciaw";
                name = "Consumo W";
                uniqId = "PotenciaTermo";
                unit = "w";
                icon = "gauge";
                break;
            case 0:
                type = "uptime";
                name = "Enphase Uptime";
                uniqId = "EnphaseUptime";
                unit = "seg";
                icon = "timeline-clock";
                break;
            }
            String message = "{";
            message += String(F("\"unit_of_measurement\":\"")) + unit + String(F("\",\"icon\":\"mdi:")) + icon + String(F("\",\""));
            message += String(F("name\":\"")) + name + String(F("\",\""));
            message += String(F("state_topic\":\"")) + String(clientId) + String(F("/sensor/")) + type + String(F("/state\",\""));
            message += String(F("availability_topic\":\"")) + String(clientId) + String(F("/status\",\""));
            message += String(F("uniq_id\":\"")) + String(clientId) + uniqId + String(F("\",\""));
            message += String(F("device\":{\"identifiers\":\"")) + String(clientId) + String(F("\",\""));
            message += String(F("name\":\"TermoEnphase\",\"sw_version\":\"ENPHASE ")) + String(sofVersion) + String(F("\",\"manufacturer\":\"Radioelf\",\"model\":\"ENPHASE1\"}"));
            message += "}";

            String topic_sensors = ("homeassistant/sensor/" + clientId + "/" + type + "/config");
            uint16_t messageLeng_sensors = message.length();
            char msg_sensors[messageLeng_sensors];
            message.toCharArray(msg_sensors, (messageLeng_sensors + 1));
            locate = client.publish((char *)topic_sensors.c_str(), msg_sensors, messageLeng_sensors);
        }
#if DebugSerial == 1
        debugPrintln("Auto localización Home Assistant");
#endif
    }
    // client.endPublish();
}
//////////////////////////////////////////////////////////////////////////////////////
// Gestionamos la recepción de los topic recibidos
// topic:
// ENPHASE-xxxxxx/cmd restartCmd
// ENPHASE-xxxxxx/cmd updateCmd
// ENPHASE-4EF85/on_off/set
// ENPHASE-4EF85/run_on_off/set
// ENPHASE-4EF85/bomba/set
//////////////////////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int length)
{
    payload[length] = '\0'; // Añadimos null al final
    String recv_payload = String((char *)payload);
#if DebugSerial == 1
    debugPrintln(String(F("Topic recibido [")) + String(topic) + "] payload " + recv_payload + " Longitud: " + String(length));
#endif
    // Comando reset?
    if (!strncmp((char *)payload, "restartCmd", length))
        espReset();
    // Comando actualizar
    if (!strncmp((char *)payload, "updateCmd", length))
    {
        locate = false;
        publicMqtt();
        return;
    }
    String topicStatus = clientId + "/on_off/set";
    if (strcmp(topic, (char *)topicStatus.c_str()) == 0)
    {
        if (length == 1)
        {
            if (payload[0] == 49)
                StatusRunStop = true;
            else if (payload[0] == 48)
                StatusRunStop = false;
            mqttSend("/on_off", StatusRunStop ? (char *)"1" : (char *)"0");
        }
        return;
    }
    topicStatus = clientId + "/run_on_off/set";
    if (strcmp(topic, (char *)topicStatus.c_str()) == 0)
    {
        if (!strncmp((char *)payload, "run", length))
        {
            if (manualRun == false)
                controlOnOFF(true);
            return;
        }
        if (!strncmp((char *)payload, "stop", length))
        {
            controlOnOFF(false);
            return;
        }
    }
    topicStatus = clientId + "/bomba/set";
    if (strcmp(topic, (char *)topicStatus.c_str()) == 0)
    {
        if (length == 1)
        {
            if (payload[0] == 49)
                StatusRunBomba = true;
            else if (payload[0] == 48)
                StatusRunBomba = false;
            // relé ON->gnd, OFF->Vcc
            digitalWrite(relayPumpPin, StatusRunBomba ? 0 : 1);
            mqttSend("/bomba", StatusRunBomba ? (char *)"1" : (char *)"0");
        }
    }
    return;
}
//////////////////////////////////////////////////////////////////////////////////////
#endif
