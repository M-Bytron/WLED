#pragma once

#include <Arduino.h>

// ---------------------
// ------- Macro -------
#define DMX_FRAME_SIZE 514 // 1 start code + 512 channels

// -------------------------
// ------- Variables -------
extern bool dmxIsConnected;
extern unsigned long dmxLastPacketTime;
extern uint8_t dmx_frame[DMX_FRAME_SIZE];

// -------------------------
// ------- Functions -------
void DMX_Setup(int dmx_num, int TX_PIN, int RX_PIN);
void DMX_RX_Task_Init();

void process_dmx_frame(uint8_t *frame, int len) ;
void logBufferHex(const uint8_t *data, size_t len);
void Extract_LED_Values(const uint8_t *buffer, size_t len, int Red_DMX, int Green_DMX, int Blue_DMX, int White_DMX, uint8_t *Values);