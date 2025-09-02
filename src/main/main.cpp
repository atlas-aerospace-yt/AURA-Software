#include "main.h"

extern "C" void app_main(void) {
  const sensor_data example_data{};
  QueueHandle_t controlOutputQueue = xQueueCreate(1, sizeof(example_data));

  xTaskCreatePinnedToCore(core_0_task, "Core_0", 10240,
                          (void*)controlOutputQueue, MOST_PRIORITY, NULL, 0);

  xTaskCreatePinnedToCore(core_1_task, "Core_1", 10240,
                          (void*)controlOutputQueue, LEAST_PRIORITY, NULL, 0);
}
