#pragma once

#include <Arduino.h>

// ---------------------
// ------- Macro -------
#define DMX_FRAME_SIZE 513 // 1 start code + 512 channels

// -------------------------
// ------- Variables -------
extern bool dmxIsConnected;
extern uint8_t dmx_frame[DMX_FRAME_SIZE];

// -------------------------
// ------- Functions -------
void DMX_Setup(int dmx_num, int TX_PIN, int RX_PIN);
void DMX_RX_Task_Init();

// ----- Private Function -----
void process_dmx_frame(uint8_t *frame, int len) ;
