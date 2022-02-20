#ifndef _HTTP_H_
#define _HTTP_H_

ESP8266WebServer server(80);
//////////////////////////////////////////////////////////////////////////////////////
// Se ejecutara en la URI '/'
//////////////////////////////////////////////////////////////////////////////////////
void handleRoot()
{
    if (APmode)
    {
        int8_t numSsid;
        server.send(200, "text/html", "Servidor Http Termo Enphase en modo punto de acceso<br>)");
        numSsid = WiFi.scanNetworks();
        if (numSsid <= 0)
        {
            server.send(200, "text/html", "NO SE ENCONTRARÓN PUNTOS DE ACCESOS)");
            return;
        }
        for (uint8_t i = 0; i < numSsid; i++)
        {
            server.send(200, "text/html", "AP: " + WiFi.SSID(i) + "RSSI: " + WiFi.RSSI(i) + "<br>");
        }
        return;
    }
    server.send(200, "text/html", "Servidor Http Termo Enphase (<a href=\"http://" + WiFi.localIP().toString() + "/data/json\">Obtener datos Json</a>)");
}
//////////////////////////////////////////////////////////////////////////////////////
// Se ejecutara en la URI '/data/json'
//////////////////////////////////////////////////////////////////////////////////////
void handleJson ()
{
    if (APmode)
    {
        server.send(200, "text/html", "Acceso NO valido, en modo punto de acceso)");
        return;
    }
    String response = "{";
    response += "\"TermoEnphase\":[{";
    response += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    response += ",\"gw\":\"" + WiFi.gatewayIP().toString() + "\"";
    response += ",\"mac\":\"" + WiFi.macAddress() + "\"";
    response += ",\"ssid\":\"" + String(WiFi.RSSI()) + "\"";
    response += ",\"ID\":\"" + String(clientId) + "\"";
    response += ",\"conectado\":\"" + String(locate) + "\"";
    response += ",\"vpp\":\"" + String(ESP.getVcc()) + "\"";
    response += "},{\"temp_termo_casa\":\"" + String(TempHome) + "\"";
    response += ",\"temp_termo_enpahse\":\"" + String(TempEnphase) + "\"";
    response += ",\"intesidad_A\":\"" + String(Corriente) + "\"";
    response += ",\"radiacion_solar\":\"" + String(radiacionSolar) + "\"";
    response += ",\"run_stop\":\"" + String(StatusRunStop) + "\"";
    response += ",\"dimmer_power\":\"" + String(dimmerPower) + "\"";
    response += ",\"recirculacion\":\"" + String(StatusRunBomba) + "\"";
    response += ",\"flujo_agua\":\"" + String(StatusFlow) + "\"";
    response += ",\"producion_solar\":\"" + String(producSolar) + "\"";
    response += ",\"consumo_red\":\"" + String(consumoRed) + "\"";
    response += ",\"consumo_casa\":\"" + String(consumo) + "\"";
    response += ",\"dimmer_power\":\"" + String(dimmerPower) + "\"";
    response += "}]}";

    server.send(200, "application/json", response);

#if DebugSerial == 1
    debugPrintln(String(F("TX Json por servidor HTTP")));
#endif
}

// Se ejecutara si la URI desconocida
void handleNotFound()
{
    server.send(404, "application/json", "{\"message\":\"NO encontrada\"}");
#if DebugSerial == 1
    debugPrintln(String(F("URL desconocida")));
#endif
}

void InitServer()
{
    // Ruteo para '/'
    server.on("/", handleRoot);
    // Ruteo para data/json
    server.on("/data/json", handleJson);
    // Ruteo para URI desconocida
    server.onNotFound(handleNotFound);
    // Iniciar servidor
    server.begin();
#if DebugSerial == 1
    debugPrintln(String(F("Servidot HTTP inicializado")));
#endif
}
#endif