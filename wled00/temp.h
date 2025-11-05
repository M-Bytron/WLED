#include <Arduino.h>
#include "config.h"

#define NTC_PIN SENSOR_PIN
#define VCC 3.3
#define RPULL 9860.0
#define R0 10000.0
#define BETA 3950.0
#define T0 298.15
#define err_adc ADC_OFFSET

int readTemp(){
    double adc_sum = 0;
    int adcValue = 0;

    for (int i = 0; i < 100; i++) {
        adc_sum += analogRead(NTC_PIN);
    }

    adcValue = adc_sum / 100.0;
    double Vntc = (adcValue / 4095.0) * VCC + err_adc;

    double Rntc = (Vntc * RPULL) / (VCC - Vntc);

    double tempK = 1.0 / ((log(Rntc / R0) / BETA) + (1.0 / T0));

    double tempC = tempK - 273.15;

//    Serial.print("\tsum: "); Serial.print(adc_sum, 2);
//    Serial.print("\tADC: "); Serial.print(adcValue);
//    Serial.print("\tVntc: "); Serial.print(Vntc, 3); Serial.print(" V");
//    Serial.print("\tRntc: "); Serial.print(Rntc, 1); Serial.print(" Ω");
//    Serial.print("\tTempK: "); Serial.print(tempK, 2);
//    Serial.print("\tTempC: "); Serial.print(tempC, 2); Serial.println(" °C");
    return tempC;
}
