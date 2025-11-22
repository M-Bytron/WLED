#pragma once

#ifndef WLED_CONFIG_H
#define WLED_CONFIG_H

#define MBytron_Config
#define Print_DMX

// ---------------------------------
// ------ GPIOs --------------------
#define DMX_TX              -1           // don't change it
#define DMX_RX              4
#define SENSOR_PIN          32
// #define MY_LED_PINS         26  // for WS2812
#define MY_LED_PINS         22,21,17,16  // RGBW PWM LED: Red, Green, Blue, White (Based on Harald Circuit)

// ---------------------------------
// ------ Network Credentials ------
#define STATIC_SSID     "Buero"         // Buero: Based on Harald Circuit
#define STATIC_PASS     "@Buero#123!"   // @Buero#123!: Based on Harald Circuit

// ---------------------------------
// ------ DMX Receiver Config ------
#define DMX_UART_NUM    2  //UART_NUM_2
#define START_CHANNEL   1  // DMXAddress

// ---------------------------------
// ---------- LED Config -----------
#define MY_LED_TYPE             TYPE_ANALOG_4CH // PWM RGBW 
// #define MY_LED_TYPE             TYPE_WS2812_RGB // PWM RGBW 
#define LED_REVERSED            false
#define MY_GAMMA_CORRECT        1.5f            // previous: 2.8f
#define MAX_LEDs_Number         1               // Number of the LEDs, for PWM RGBW all the LEDs are paralleled
#define Turn_ON_Brightness      0xFF            // The value of brightness (0-255) when esp32 is rebooted

//----------------------------------
//--------- Sensor config ----------
#define CRITIC_TEMP         25          // Minimum temperature for brightness decrease.
#define ADC_OFFSET          0.23        // sensor ADC voltage offset 
#define BDP                 0.05        // BRIGHTNESS_DECREASE_PERCENT (Default 5%)
#define CHECK_DELAY         1           // Check temperature every second.

//----------------------------------
//------------ Effects -------------
// sample of effects:
#define Wipe            3
#define ColorLoop       8
#define TwoDots         50
#define TwinkleFox      80

//-------- Default Effect ------
#define My_Effect_ID               ColorLoop
#define My_Effect_Speed            128    // 0 - 255
#define My_Effect_Intensity        128    // 0 - 255


//------------------------------
#endif //WLED_CONFIG_H
