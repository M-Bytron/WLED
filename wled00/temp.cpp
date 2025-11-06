//
// Created by ata on 10/29/25.
//
#include <temp.h>

int readTemp() {
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

    return tempC;
}