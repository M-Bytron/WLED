#include "MBytron_DMX.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "wled.h"
#include <Arduino.h>
// #include "set.cpp"

// ----------------------
// ------- Globals -------
int enablePin = 21;

static const char *TAG = "DMX_RX";

// ---- UART Configs
#define DMX_RX_PIN ((gpio_num_t)DMX_RX)
#define DMX_TX_PIN DMX_TX

// #define DMX_FRAME_SIZE 513 // 1 start code + 512 channels
#define DMX_BAUD_RATE 250000
#define DMX_RX_BUF_SIZE 514
#define DMX_TIMEOUT_MS 200
#define MAX_LEDs_Number 10
QueueHandle_t uart_queue;

// ---- DMX variables
#define dmx_data_queue_size 10
#define required_dmx_data_size 4
uint8_t dmx_frame[DMX_FRAME_SIZE];
bool dmxIsConnected = false;
unsigned long dmxlastUpdate = millis();
unsigned long dmxLastPacketTime;
QueueHandle_t dmx_data_queue;
uint32_t data_couter = 0;

typedef struct {
    uint8_t data[required_dmx_data_size];
  } dmx_short_frame_t;


// ---- Break Variable
volatile bool break_detected = false;
volatile uint64_t break_start_time = 0;
volatile uint64_t break_end_time = 0;
volatile uint32_t break_width_us = 0;
volatile bool ready_to_receive = false;
SemaphoreHandle_t dmx_break_semaphore;  // used to signal from ISR to task


// ---------------------------------------------------
// ------- General Functions -------------------------
// ---------------------------------------------------

// ---------- set RGBW values directly ----------
void setRGBWValues(byte r, byte g, byte b, byte w) {
  // strip.suspend(); // Block strip servicing
  uint32_t color = RGBW32(r, g, b, w);
  for (int i=0 ; i < MAX_LEDs_Number; i++){
  BusManager::setPixelColor(i, color);
  }
  BusManager::show();
  // strip.resume(); // Allow strip servicing again
}

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

  uart_param_config(dmx_num, &uart_config);
  uart_set_pin(dmx_num, DMX_TX, DMX_RX, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  // Install UART driver with event queue
  // QueueHandle_t uart_queue;
  uart_driver_install(dmx_num, DMX_RX_BUF_SIZE, 0, 20, &uart_queue, 0);
  DEBUG_PRINTF("DMX UART RX task started!\n");
}

// ------- Break Detection ISR -------
static void IRAM_ATTR dmx_rx_isr_handler(void *arg) { 

  if (!ready_to_receive) {
    int level = gpio_get_level(DMX_RX_PIN);
    uint64_t now = esp_timer_get_time(); // in microseconds
    if (level == 0) {
      // Falling edge → start of break
      break_detected = true;
      break_start_time = now;
    } else {
      // Rising edge → end of break
      if (break_detected) {
        break_end_time = now;
        break_width_us = (uint32_t)(break_end_time - break_start_time);
        break_detected = false;

        // Valid DMX break is typically >88 µs
        if (break_width_us > 80) {
          ready_to_receive = true;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          xSemaphoreGiveFromISR(dmx_break_semaphore, &xHigherPriorityTaskWoken);
          if (xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
        }
      }
    }
  }
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

  printf("DMX RX GPIO ISR initialized\n");
}

// ------- DMX UART RX Task -------
void dmx_uart_rx_task(void *pvParameters) {

  uart_event_t event;
  int len;
  bool break_detected = false;
  uint32_t break_start_time = 0;
  uint8_t trash = 0;
  size_t buffer_uart_rx = 0;

  while (1) {

    if (xSemaphoreTake(dmx_break_semaphore, portMAX_DELAY) == pdTRUE) {

      dmxlastUpdate = millis();
      dmxIsConnected = true;
      strip.suspend(); // Block strip servicing
      int len = uart_read_bytes(DMX_UART_NUM, dmx_frame, DMX_FRAME_SIZE, 50 / portTICK_PERIOD_MS);
      if (len == DMX_FRAME_SIZE) {
        if (dmx_data_queue) {
          data_couter++;
          dmx_short_frame_t frame_to_send;
          memcpy(frame_to_send.data, dmx_frame + START_CHANNEL + 1, sizeof(frame_to_send.data));
          if (xQueueSend(dmx_data_queue, &frame_to_send, 0) != pdTRUE ) // sends first 6 bytes (array decays to pointer)          
            printf("Failed to queue\n");
        }
      } else {
        printf("Incomplete DMX frame (%d bytes)\n", len);
      }
      esp_err_t err = uart_flush_input(DMX_UART_NUM);
        if (err != ESP_OK) {
            DEBUG_PRINTF("UART flush failed: %d\n", err);
        }
      ready_to_receive = false;
      }
  }

}

// ------- Print DMX Data Task -------
void print_dmx_data(void *pvParameters) {
  
  dmx_short_frame_t received_frame;
  dmx_short_frame_t previous_frame;

  while (true) {

    if (xQueueReceive(dmx_data_queue, &received_frame, portMAX_DELAY)){

      // if (previous_frame != received_frame){
      if (memcmp(&previous_frame, &received_frame, sizeof(dmx_short_frame_t)) != 0){
        previous_frame = received_frame;
        printf("Valid break detected: %lu us\n", (unsigned long)break_width_us);
        printf("data_couter: %ld\n", data_couter);
        printf("First 6 DMX bytes: ");
        for (int i = 0; i < 4; i++) {
          printf("%02X ", received_frame.data[i]);
        }
        printf("\n--------------------------\n");
      }
      setRGBWValues(received_frame.data[0], received_frame.data[1], received_frame.data[2], received_frame.data[3]);
    }

  }
}

// ------- DMX Connection Check -------
void dmx_connection_check_task(void *pvParameters) {

  while (true) {
    if (dmxIsConnected && (millis() - dmxlastUpdate > DMX_TIMEOUT_MS)) {
      dmxIsConnected = false;
      strip.resume();
      printf("*** DMX DISCONNECTED!\n");
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

  dmx_data_queue = xQueueCreate(dmx_data_queue_size, sizeof(uint8_t) * required_dmx_data_size); // up to 10 queued DMX messages
  if (dmx_data_queue == NULL) {
    DEBUG_PRINTF("Failed to create DMX data queue!\n");
  }

  xTaskCreatePinnedToCore(dmx_uart_rx_task, "dmx_uart_rx_task", 4096, NULL, 4,
                          NULL, 0);
  xTaskCreatePinnedToCore(dmx_connection_check_task,
                          "dmx_connection_check_task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(print_dmx_data,
                          "print_dmx_data", 2048, NULL, 1, NULL, 0);
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
    DEBUG_PRINTF("%02X ", data[i]);
    DEBUG_PRINTF(" ");
  }
  DEBUG_PRINTF("\n");
}