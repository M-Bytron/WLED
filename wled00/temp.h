#ifndef WLEDX_TEMP_H
#define WLEDX_TEMP_H

#include <Arduino.h>
#include "config.h"

#define NTC_PIN SENSOR_PIN
#define VCC 3.3
#define RPULL 9860.0
#define R0 10000.0
#define BETA 3950.0
#define T0 298.15
#define err_adc ADC_OFFSET

int readTemp();

#endif //WLEDX_TEMP_H