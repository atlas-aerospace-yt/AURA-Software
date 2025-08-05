#include "main_core_1.h"

void core_1_task(void *args)
{
    QueueHandle_t controlOutputQueue = (QueueHandle_t)args;
    sensor_data my_data;

    spi::create_spi_bus();

    sdmmc_card_t sd_card;

    flash::flash my_flash(GPIO_NUM_21);

    sd_card::mount_sd_card(&sd_card);
    my_flash.mount_fat_partition();

    sd_card::unmount_sd_card(&sd_card);
    my_flash.unmount_fat_partition();

    while (true)
    {
        vTaskDelay(10);
        if (xQueueReceive(controlOutputQueue, &my_data, MAX_DELAY)) {
            ESP_LOGI("LOGGER", "Delta Time: %.8f",my_data.dt);
            GRAPH("GyroX", my_data.gyro_x, TOP);
            GRAPH("GyroY", my_data.gyro_y, TOP);
            GRAPH("GyroZ", my_data.gyro_z, TOP);
        }
    }
}
