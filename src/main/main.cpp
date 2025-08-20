#include "main.h"

/*
#include "driver/uart.h"
#include "freertos/ringbuf.h"

#define UART_PORT UART_NUM_1
#define RX_PIN GPIO_NUM_1  // IO1
#define BUF_SIZE 256

#define IBUS_UART UART_NUM_1
#define IBUS_FRAME_LEN 32
#define START_BYTE1 0x20
#define START_BYTE2 0x40
#define RINGBUF_SIZE 256

RingbufHandle_t buf_handle;

// Dummy handler for a complete IBUS frame
void handle_frame(const uint8_t* frame) {
  printf("Got IBUS frame: ");
  for (int i = 0; i < IBUS_FRAME_LEN; i++) {
    printf("%02X ", frame[i]);
  }
  printf("\n");
}

// Call this periodically or in a UART task
void process_ibus_frames() {
  size_t item_size;
  uint8_t* data;
  uint8_t last_frame_start_pos;
  int16_t frame_start = -1;

  int channel_1 = 0;
  int channel_2 = 0;
  int channel_3 = 0;
  int channel_4 = 0;
  int channel_5 = 0;
  int channel_6 = 0;

  // Receive all available bytes from the ring buffer
  data = static_cast<uint8_t*>(xRingbufferReceive(buf_handle, &item_size, 0));

  if (data != NULL) {
    last_frame_start_pos = item_size - 33;
    for (int i = last_frame_start_pos; i >= 0; i--) {
      // Check if frame start is detected within a full frame
      if (data[i] == 0x20 && data[i + 1] == 0x40) {
        frame_start = i;
        break;
      }
    }
    // If there is a complete frame - process it
    if (frame_start != -1) {
      // for (int i = frame_start; i < frame_start + 32; i++) {
      //   printf("%02X ", data[i]);
      // }
      // printf("\n");
      channel_1 = data[frame_start + 2] | data[frame_start + 3] << 8;
      printf("Channel1: %d ", channel_1);
      channel_2 = data[frame_start + 4] | data[frame_start + 5] << 8;
      printf("Channel2: %d ", channel_2);
      channel_3 = data[frame_start + 6] | data[frame_start + 7] << 8;
      printf("Channel3: %d ", channel_3);
      channel_4 = data[frame_start + 8] | data[frame_start + 9] << 8;
      printf("Channel4: %d ", channel_4);
      channel_5 = data[frame_start + 10] | data[frame_start + 11] << 8;
      printf("Channel5: %d ", channel_5);
      channel_6 = data[frame_start + 12] | data[frame_start + 13] << 8;
      printf("Channel6: %d ", channel_6);
      printf("\n");
    }
    vRingbufferReturnItem(buf_handle, static_cast<void*>(data));  // Free the received memory
  }
}
*/

extern "C" void app_main(void) {
  /*
  // Configure UART params
  uart_config_t uart_config = {};
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, -1, RX_PIN, -1, -1));

  uint8_t data[BUF_SIZE];

  // Setup ring buffer to decode the iBus data
  buf_handle = xRingbufferCreate(256, RINGBUF_TYPE_BYTEBUF);

  while (true) {
    // Read and add to ring buffer
    int len = uart_read_bytes(UART_PORT, data, sizeof(data), 0);
    xRingbufferSend(buf_handle, data, len, 0);

    // Print
    process_ibus_frames();
    vTaskDelay(pdMS_TO_TICKS(10));
  }*/

  const sensor_data example_data{};
  QueueHandle_t controlOutputQueue = xQueueCreate(1, sizeof(example_data));

  xTaskCreatePinnedToCore(core_0_task, "Core_0", 10240, (void*)controlOutputQueue, MOST_PRIORITY,
                          NULL, 0);

  xTaskCreatePinnedToCore(core_1_task, "Core_1", 10240, (void*)controlOutputQueue, LEAST_PRIORITY,
                          NULL, 0);
}
