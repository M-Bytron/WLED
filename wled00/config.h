//
// Created by ata on 10/28/25.
//
#pragma once

#ifndef WLED_CONFIG_H
#define WLED_CONFIG_H


// ---------------------------------
// ------ Network Credentials ------
#define STATIC_SSID     "Buero"  // Buero: Based on Harald Circuit
#define STATIC_PASS     "@Buero#123!"   // @Buero#123!: Based on Harald Circuit

// ---------------------------------
// ------ DMX Receiver Config ------
#define DMX_INPUT_RX_PIN    4    // 4: Based on Harald Circuit
#define DMX_INPUT_TX_PIN    25
#define DMX_INPUT_EN_PIN    -1   // we dont use RS485 module
#define DMX_INPUT_PORT      1    // DMX Universe

// ---------------------------------
// ---------- LED Config -----------
// -> PWM RGBW
#define MY_LED_TYPE     TYPE_ANALOG_4CH
#define MY_LED_PINS     22,21,17,16  // Red, Green, Blue, White (Based on Harald Circuit)

// -> WS28*1x
// #define MY_LED_TYPE     TYPE_WS2812_RGB
// #define MY_LED_PINS     16  

#define LED_REVERSED    false


#endif //WLED_CONFIG_H
