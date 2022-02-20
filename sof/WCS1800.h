#ifndef _WCS1800_H_
#define _WCS1800_H_
//////////////////////////////////////////////////////////////////////////////////////
// Sensibilidad, voltaje cuando I=0, frecuencia en HZ
//////////////////////////////////////////////////////////////////////////////////////
#define WCS3v3 1
#if WCS3v3
float sensitivity = 0.066;                                                            // 66mV/A para 25A AC RMS y 3.3V.
float zero = 1.65;
#else
float sensitivity =0.066;                                                             // 66mV/A para 25A AC (30DC) RMS y 5V.
float zero = 2.5;
#endif

int16_t zeroRead = 0;
float VRMS = 0.0;
bool WCS1800 = false;
//////////////////////////////////////////////////////////////////////////////////////
// calibración inicial (SIN consumo)
//////////////////////////////////////////////////////////////////////////////////////
int16_t WCScalibrate()
{
    if (ADS1115 == false)
        return 0;
    int32_t Aout = 0;
    for (uint8_t i = 0; i < 200; i++)
    {
        Aout += ads.readADC_SingleEnded(1);                                           // 200 lecturas
    }
    zeroRead = Aout / 200;
    return zeroRead;
}
//////////////////////////////////////////////////////////////////////////////////////
// Leer la tensión en la entrada analógica
//////////////////////////////////////////////////////////////////////////////////////
float WCSgetVout()
{
    if (ADS1115 == false)
        return 0.0;
    float vout = 0.0;
    uint16_t adc0 = ads.readADC_SingleEnded(1);
    vout = ads.computeVolts(adc0);
    float V = vout * zero / zeroRead;
    return V;
}
//////////////////////////////////////////////////////////////////////////////////////
// lectura corriente  en continua
//////////////////////////////////////////////////////////////////////////////////////
float WCSgetDC(float Vo)
{
    float I = (Vo - zero);
    I = I / sensitivity;
    return I;
}
//////////////////////////////////////////////////////////////////////////////////////
// Lectura de corriente en alterna
//////////////////////////////////////////////////////////////////////////////////////
float WCSgetAC()
{
    if (ADS1115 == false)
        return 0.0;
    uint16_t maxCurrent = 0;
    // 15 bits para ads1115 - 1bit signo->32767, 12 bits ads1015 - 1bit signo->2047
    uint16_t minCurrent = 32767; 
    uint16_t Current = 0;
    uint8_t startAdc = 0;
    // delay read +-2.8ms (60ms)
    while (startAdc++ != 20)
    {
        Current = ads.readADC_SingleEnded(1);                                         
        if (Current > maxCurrent)
        {
            maxCurrent = Current;
        }
        if (Current < minCurrent)
        {
            minCurrent = Current;
        }
    }
    if (maxCurrent - minCurrent < 5)
        return 0.0;
    float Iac = (((maxCurrent - minCurrent) * zero / zeroRead) * 0.707 / 2) / sensitivity;
    return Iac;

}
#endif
