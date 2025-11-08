#include "MBytron_DMX.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "wled.h"
#include <Arduino.h>


// ----------------------
// ------- Globals -------
int enablePin = 21;

static const char *TAG = "DMX_RX";

// ---- UART Configs
#define DMX_UART_NUM UART_NUM_2
#define DMX_RX_PIN DMX_TX
#define DMX_TX_PIN DMX_RX
#define DMX_FRAME_SIZE 513 // 1 start code + 512 channels
#define DMX_BAUD_RATE 250000
#define DMX_RX_BUF_SIZE 1024
#define DMX_TIMEOUT_MS 2000
QueueHandle_t uart_queue;

// ---- DMX variables
uint8_t dmx_frame[DMX_FRAME_SIZE];
bool dmxIsConnected = false;
unsigned long dmxlastUpdate = millis();
unsigned long dmxLastPacketTime;

/*
// ------- Extract LED Values -------
void Extract_LED_Values(const uint8_t *buffer, size_t len, int Red_DMX, int
Green_DMX, int Blue_DMX, int White_DMX, uint8_t *Values) { int indices[4] =
{Red_DMX, Green_DMX, Blue_DMX, White_DMX}; for (int j = 0; j < 4; j++) { if
(indices[j] < len) { Values[j] = buffer[indices[j]]; DEBUG_PRINTF("Values[%d]:
%d\n", j, Values[j]); } else { Values[j] = 0; // or handle error if index is out
of range
        }
    }
}*/

// ---------------------------------------------------
// ------- UART x DMX Functions ----------------------
// ---------------------------------------------------

// ------- DMX Setup -------
void DMX_Setup(int dmx_num, int TX_PIN, int RX_PIN) {

  DEBUG_PRINTF("--------------\n");
  DEBUG_PRINTF(">  DMX Setup <\n");
  DEBUG_PRINTF("--------------\n");

  uart_config_t uart_config = {.baud_rate = DMX_BAUD_RATE,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_2,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .source_clk = UART_SCLK_APB};

  uart_param_config(DMX_UART_NUM, &uart_config);
  uart_set_pin(DMX_UART_NUM, DMX_TX, DMX_RX, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  // Install UART driver with event queue
  // QueueHandle_t uart_queue;
  uart_driver_install(DMX_UART_NUM, DMX_RX_BUF_SIZE, 0, 20, &uart_queue, 0);
  DEBUG_PRINTF("DMX UART RX task started!\n");
}

// ------- DMX UART RX Task -------
void dmx_uart_rx_task(void *pvParameters) {

  uart_event_t event;
  int len;
  uint8_t start_code;
  size_t buffer_uart_rx = 0;

  while (true) {
    // Wait for UART events
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {

      switch (event.type) {

        case UART_BREAK: {
          // Read the start code (first byte after BREAK)
          int start_len = uart_read_bytes(DMX_UART_NUM, &start_code, 1, 10 / portTICK_PERIOD_MS);

          if (start_len == 1 && start_code == 0x00) {  // Valid DMX start code
            if (!dmxIsConnected) {
              dmxIsConnected = true;
              DEBUG_PRINTF("DMX Connected!\n");
            }

            uint32_t break_start_time = millis();
            DEBUG_PRINTF("BREAK detected at %lu\n", break_start_time);

            // Read the remaining 512 data bytes
            int data_len = uart_read_bytes(DMX_UART_NUM, &dmx_frame[1], 512, 50 / portTICK_PERIOD_MS);

            if (data_len == 512) {
              dmx_frame[0] = 0x00;  // Set start code
              DEBUG_PRINTF("DMX Frame received: start=0x%02X, len=%d\n", dmx_frame[0], 513);
              logBufferHex("UART", dmx_frame, 513);
              dmxLastPacketTime = millis();
            }
          }
          break;
        }

        default: {
          // Drain any unexpected data
          uint8_t trash;
          uart_read_bytes(DMX_UART_NUM, &trash, 1, 0);
          break;
        }
      }
    }
  }
}

// ------- DMX UART RX Task -------
void dmx_connection_check_task(void *pvParameters) {

  while (true) {
     
  if (dmxIsConnected && (millis() - dmxLastPacketTime > DMX_TIMEOUT_MS)) {
      DEBUG_PRINTF("DMX Disconnected! (timeout)\n");
      esp_err_t err = uart_flush_input(DMX_UART_NUM);
      if (err == ESP_OK) {
        DEBUG_PRINTF("UART RX buffer flushed successfully.\n");
      } else {
        DEBUG_PRINTF("UART RX buffer flush failed! Error code: %d\n", err);
      }
      dmxIsConnected = false;
    }
    
    // Small delay to prevent busy-waiting
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ------- DMX UART RX Task -------
// Must be called from setup()
void DMX_RX_Task_Init() {

  DEBUG_PRINTF("---------------------\n");
  DEBUG_PRINTF(">  DMX RX Task Init <\n");
  DEBUG_PRINTF("---------------------\n");
  xTaskCreatePinnedToCore(dmx_uart_rx_task, "dmx_uart_rx_task", 4096, NULL, 4,
                          NULL, 0);
  xTaskCreatePinnedToCore(dmx_connection_check_task, "dmx_connection_check_task", 1024, NULL, 1,
                          NULL, 0);
}

// ------- Process DMX Frame -------
void process_dmx_frame(uint8_t *frame, int len) {
  ESP_LOGI(TAG,
           "DMX frame received len=%d start=0x%02X ch1=%u ch2=%u ch3=%u ch4=%u",
           len, frame[0], frame[1], frame[2], frame[3], frame[4]);
}

// ------- log Buffer Hex -------
void logBufferHex(const char *tag, const uint8_t *data, size_t len) {

  DEBUG_PRINTF("Start Byte: %02X\n", data[0]);
  DEBUG_PRINTF("Data: ");
  for (size_t i = 1; i < len; i++) {
    // if (data[i] < 0x10)
    //   DEBUG_PRINTF("0");
    DEBUG_PRINTF("%02X ", data[i]);
    DEBUG_PRINTF(" ");
  }
  DEBUG_PRINTF("\n");
}