//
// Created by ata on 10/28/25.
//
#pragma once

#ifndef WLED_CONFIG_H
#define WLED_CONFIG_H

#define MBytron_Config

// ---------------------------------
// ------ Network Credentials ------
#define STATIC_SSID     "Bambootec"  // Buero: Based on Harald Circuit
#define STATIC_PASS     "12345678"   // @Buero#123!: Based on Harald Circuit

// ---------------------------------
// ------ DMX Receiver Config ------
#define DMX_UART_NUM    2  //UART_NUM_2
#define DMX_TX          -1
#define DMX_RX          4
#define START_CHANNEL   1

// ---------------------------------
// ---------- LED Config -----------
// -> PWM RGBW
#define MY_LED_TYPE     TYPE_ANALOG_4CH
#define MY_LED_PINS     22,21,17,16  // Red, Green, Blue, White (Based on Harald Circuit)

// -> WS28*1x
// #define MY_LED_TYPE     TYPE_WS2812_RGB
// #define MY_LED_PINS     16

#define LED_REVERSED    false

//----------------------------------
//--------- Sensor config ----------
#define SENSOR_PIN 32
#define ADC_OFFSET 0.23
#define MIN_TEMP 40     // Minimum temperature for brightness decrease.
#define BDP 0.05        // BRIGHTNESS_DECREASE_PERCENT (Default 5%)
#define CHECK_DELAY 1   // Check temperature every second.

//----------------------------------
//------------ Effects -------------
// sample of effects:
#define Wipe 3
#define ColorLoop 8
#define TwoDots 50
#define TwinkleFox 80

//-------- Default Effect ------
#define EFFECT_ID ColorLoop
#define EFFECT_SPEED 128    // 0 - 255
#define EFFECT_INTENSITY 128    // 0 - 255



//------------------------------
#endif //WLED_CONFIG_H
