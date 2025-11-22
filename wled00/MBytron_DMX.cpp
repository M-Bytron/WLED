#include "MBytron_DMX.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "wled.h"
#include <Arduino.h>
#include <cstddef>

// -----------------------
// ------- Globals -------


static const char *TAG = "DMX_RX";

// ---- UART Configs
#define DMX_RX_PIN        ((gpio_num_t)DMX_RX)
#define DMX_TX_PIN        DMX_TX
#define DMX_BAUD_RATE     250000
#define DMX_RX_BUF_SIZE   DMX_FRAME_SIZE
#define DMX_TIMEOUT_MS    200

// ---- DMX variables
#define dmx_data_queue_size 10
#define required_dmx_data_size 4

uint8_t dmx_frame[DMX_FRAME_SIZE];
bool dmxIsConnected = false;
unsigned long dmxlastUpdate = millis();

typedef struct {
  uint8_t data[required_dmx_data_size];
} dmx_short_frame_t;
dmx_short_frame_t frame_to_send; 
QueueHandle_t dmx_data_queue;
SemaphoreHandle_t dmx_break_semaphore; // used to signal from ISR to task

// ---- Counters
volatile int16_t extra_break = 0;
uint32_t data_couter = 0;

// ----- UART FIFO Data
size_t fifo_B_bef_strtr_svng;
size_t fifo_B_aft_break;
size_t fifo_B_bef_clr;

// ---- Break Variable
volatile uint64_t break_start_time = 0;
volatile uint64_t break_end_time = 0;
volatile uint32_t break_width_us = 0;
volatile uint64_t mab_duration = 0;

// ---- Flags
volatile bool break_detected = false;
volatile bool ready_to_receive = false;
volatile bool wait_to_start_dmx_data = false;

// ----------------------------------------------------------------------------
// ------- General Functions --------------------------------------------------
// ----------------------------------------------------------------------------

// ---------- set RGBW values directly ----------
void setRGBWValues(byte r, byte g, byte b, byte w) {

  if (LEDs_Temp > CRITIC_TEMP){
    BusManager::setBrightness(bri);
    applyFinalBri();
  }
  uint32_t color = RGBW32(r, g, b, w);
  for (int i = 0; i < MAX_LEDs_Number; i++) {
    BusManager::setPixelColor(i, color);
  }
  BusManager::show();
}

// ---------------------------------------------------
// ------- UART x DMX Functions ----------------------
// ---------------------------------------------------

// ------- DMX Setup -------
void DMX_Setup(int dmx_num, int TX_PIN, int RX_PIN) {

  DEBUG_PRINTF("--------------\n");
  DEBUG_PRINTF(">  DMX Setup <\n");
  DEBUG_PRINTF("--------------\n");

  uart_config_t uart_config;
  uart_config = {.baud_rate = DMX_BAUD_RATE,
                 .data_bits = UART_DATA_8_BITS,
                 .parity = UART_PARITY_DISABLE,
                 .stop_bits = UART_STOP_BITS_2,
                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                 .source_clk = UART_SCLK_APB};

  uart_param_config(dmx_num, &uart_config);
  uart_set_pin(dmx_num, DMX_TX, DMX_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // ---- Install UART driver with event queue
  QueueHandle_t uart_queue;
  uart_driver_install(dmx_num, DMX_RX_BUF_SIZE, 0, 20, &uart_queue, 0);
  DEBUG_PRINTF("DMX UART RX task started!\n");
}

// ------- Break Detection ISR -------
static void IRAM_ATTR dmx_rx_isr_handler(void *arg) {

  int level = gpio_get_level(DMX_RX_PIN);
  uint64_t now = esp_timer_get_time(); // in microseconds

  // ---- Falling edge → start of break
  if (level == 0) {    
    break_detected = true;
    break_start_time = now;
  } 
  // ---- Rising edge → end of break
  else {    
    if (break_detected) {
      break_end_time = now;
      break_width_us = (uint32_t)(break_end_time - break_start_time);
      break_detected = false;

      // ---- Valid DMX break is typically >88 µs
      if (break_width_us > 88) {
        if (!ready_to_receive) {          
          uart_get_buffered_data_len(DMX_UART_NUM, &fifo_B_aft_break);
          uart_flush_input(DMX_UART_NUM);
          ready_to_receive = true;
          /* wait_to_start_dmx_data = true; */
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          xSemaphoreGiveFromISR(dmx_break_semaphore, &xHigherPriorityTaskWoken);
          if (xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
        } else
          extra_break++;
      }
    }
  }
  // ---- MBA duration
  /*if (ready_to_receive && wait_to_start_dmx_data) {
    int level = gpio_get_level(DMX_RX_PIN);
    if (level == 0) {      
      wait_to_start_dmx_data = false;
      uint64_t now = esp_timer_get_time(); // in microseconds
      mab_duration = now - break_end_time;
    }
  }*/
}

// ------- GPIO Init -------
void dmx_gpio_init(void) {
  gpio_config_t io_conf = {.pin_bit_mask = 1ULL << DMX_RX_PIN,
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_ANYEDGE};
  gpio_config(&io_conf);

  dmx_break_semaphore = xSemaphoreCreateBinary();

  gpio_install_isr_service(0);
  gpio_isr_handler_add(DMX_RX_PIN, dmx_rx_isr_handler, NULL);

  DEBUG_PRINTF("DMX RX GPIO ISR initialized\n");
}

// ------- DMX UART RX Task -------
void dmx_uart_rx_task(void *pvParameters) {

  uart_event_t event;
  int len;
  bool break_detected = false;

  while (1) {

    if (xSemaphoreTake(dmx_break_semaphore, portMAX_DELAY) == pdTRUE) {
      dmxlastUpdate = millis();
      if (!dmxIsConnected)
        dmxIsConnected = true;      
      uart_get_buffered_data_len(DMX_UART_NUM, &fifo_B_bef_strtr_svng);
      int len = uart_read_bytes(DMX_UART_NUM, dmx_frame, DMX_FRAME_SIZE,
                                50 / portTICK_PERIOD_MS);
      // -- valid DMX data
      if (len == DMX_FRAME_SIZE) {
        // -- Queue DMX to be processed it in another task
        if (dmx_data_queue) {
          data_couter++;
          memcpy(frame_to_send.data, dmx_frame + DMXAddress, sizeof(frame_to_send.data));
          xQueueSend(dmx_data_queue, &frame_to_send, 0);
        }
      }
      // -- Invalid DMX data
      else {
        DEBUG_PRINTF("Incomplete DMX frame (%d bytes)\n", len);
      }      
      uart_get_buffered_data_len(DMX_UART_NUM, &fifo_B_bef_clr);
      ready_to_receive = false;
    }
  }
}

// ------- Print DMX Data Task -------
void process_dmx_data_task(void *pvParameters) {

  dmx_short_frame_t received_frame;
  dmx_short_frame_t previous_frame;

  while (true) {

    if (xQueueReceive(dmx_data_queue, &received_frame, portMAX_DELAY)) {

      // ----- Block strip servicing
      strip.suspend(); 
      
      // ---- Process DMX data -----      
      setRGBWValues(received_frame.data[0], received_frame.data[1],
                    received_frame.data[2], received_frame.data[3]);

      // ---- Print DMX data -----
      #ifdef Print_DMX
      if (memcmp(&previous_frame, &received_frame, sizeof(dmx_short_frame_t))
      != 0){
        previous_frame = received_frame;      
      
      // DEBUG_PRINTF("Bytes after detection break: %u\n", fifo_B_aft_break);
      // DEBUG_PRINTF("Bytes before start saving: %u\n", fifo_B_bef_strtr_svng);
      // DEBUG_PRINTF("Bytes before clearing: %u\n", fifo_B_bef_clr);
      DEBUG_PRINTF("Unprocessed Breaks: %u\n\n", extra_break);

      DEBUG_PRINTF("BREAK detected at %lu\n", break_start_time);
      DEBUG_PRINTF("Break Width: %lu us\n", (unsigned long)break_width_us); 
      DEBUG_PRINTF("MAB duration: %lu us\n\n", mab_duration);
      DEBUG_PRINTF("Data Couter: %ld\n", data_couter);
      DEBUG_PRINTF(" - Red: %02X \n - Green: %02X \n - Blue: %02X \n - White: %02X\n", 
        received_frame.data[0], received_frame.data[1], 
        received_frame.data[2], received_frame.data[3]);
      DEBUG_PRINTF("First 5 DMX bytes: ");
      for (int i = 0; i < 5; i++) {
        DEBUG_PRINTF("%02X ", dmx_frame[i]);
      }
      DEBUG_PRINTF("\n");
      DEBUG_PRINTF("--------------------------\n");
      }
      #endif
    }
  }
}

// ------- DMX Connection Check -------
void dmx_connection_check_task(void *pvParameters) {

  while (true) {
    if (dmxIsConnected && (millis() - dmxlastUpdate > DMX_TIMEOUT_MS)) {
      dmxIsConnected = false;
      strip.resume();
      DEBUG_PRINTF("*** DMX DISCONNECTED!\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
// ------- DMX UART RX Task -------
// Must be called from setup()
void DMX_RX_Task_Init() {

  DEBUG_PRINTF("---------------------\n");
  DEBUG_PRINTF(">  DMX RX Task Init <\n");
  DEBUG_PRINTF("---------------------\n");

  dmx_gpio_init();

  dmx_data_queue = xQueueCreate(
      dmx_data_queue_size,
      sizeof(uint8_t) * required_dmx_data_size); // up to 10 queued DMX messages
  if (dmx_data_queue == NULL) {
    DEBUG_PRINTF("Failed to create DMX data queue!\n");
  }

  xTaskCreatePinnedToCore(dmx_uart_rx_task, "dmx_uart_rx_task", 4096, NULL, 4,
                          NULL, 0);
  xTaskCreatePinnedToCore(dmx_connection_check_task,
                          "dmx_connection_check_task", 4096, NULL, 1, NULL, 0);
#ifdef Print_DMX
  xTaskCreatePinnedToCore(process_dmx_data_task, "process_dmx_data_task", 2048, NULL, 1, NULL,
                          0);
#endif
}
