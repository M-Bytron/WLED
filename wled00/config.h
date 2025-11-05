//
// Created by ata on 10/28/25.
//
#pragma once

#ifndef WLED_CONFIG_H
#define WLED_CONFIG_H

#define STATIC_SSID "Bambootec"
#define STATIC_PASS "12345678"
#define DMX_INPUT_RX_PIN    33
#define DMX_INPUT_TX_PIN    25
#define DMX_INPUT_EN_PIN    -1  // we dont use RS485
#define DMX_INPUT_PORT  1   //

#define MY_LED_PINS 5,21,17,16   // Red, Green, Blue, White
#define LED_REVERSED false

#define SENSOR_PIN 4
#define ADC_OFFSET 0.23

#endif //WLED_CONFIG_H
