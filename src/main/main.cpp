#include "main.h"

extern "C" void app_main(void)
{
    spi_bus_config_t spi_config = {};

    spi_config.mosi_io_num = GPIO_NUM_35;
    spi_config.miso_io_num = GPIO_NUM_37;
    spi_config.sclk_io_num = GPIO_NUM_36;
    spi_config.quadwp_io_num = -1;
    spi_config.quadhd_io_num = -1;
    spi_config.max_transfer_sz = 4000;

    ESP_ERROR_CHECK(spi_bus_initialize(
        HSPI_HOST,
        &spi_config,
        SPI_DMA_CH_AUTO
    ));

    sdmmc_card_t *sd_card;

    esp_vfs_fat_mount_config_t file_config = {};
    file_config.format_if_mount_failed = false;
    file_config.max_files = 5;

    sdmmc_host_t sd_host = SDSPI_HOST_DEFAULT();
    sd_host.slot = HSPI_HOST;

    sdspi_device_config_t sd_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sd_config.gpio_cs = GPIO_NUM_13;
    sd_config.host_id = HSPI_HOST;

    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(
        "/sdcard",
        &sd_host,
        &sd_config,
        &file_config,
        &sd_card
    ));

    std::ofstream MyFile("/sdcard/big_cock.txt");
    MyFile << "I have a massive weiner!";
    MyFile.close();

    esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);

    /*i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);
    bmp280::bmp280 my_bmp(&my_bus);
    icm20948::icm20948 my_icm(&my_bus);
    //ina219::ina219 my_ina(&my_bus);

    Quat rate;
    Quat gyro = {0.0f, 0.0f, 0.0f, 0.0f};
    Quat ori_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    Vect ori_euler = {0.0f, 0.0f, 0.0f};

    float dt;
    float x_offs = 0.0f, y_offs = 0.0f, z_offs = 0.0f;

    int64_t delta_us, now;
    int64_t last_time = esp_timer_get_time();

    for (int i=0; i<1000; i++)
    {
        my_icm.update();
        x_offs += my_icm.gyro_x;
        y_offs += my_icm.gyro_y;
        z_offs += my_icm.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    x_offs /= 1000.0f;
    y_offs /= 1000.0f;
    z_offs /= 1000.0f;*/

    while (true)
    {
        /*now = esp_timer_get_time();
        delta_us = now - last_time;
        last_time = now;

        if (delta_us < 0) delta_us = 0;
        dt = (float)delta_us / 1e6f;

        my_icm.update();
        gyro.i = my_icm.gyro_x - x_offs;
        gyro.j = my_icm.gyro_y - y_offs;
        gyro.k = my_icm.gyro_z - z_offs;


        rate = ori_quat * 0.5f * gyro;
        ori_quat = ori_quat + rate * dt;

        ori_euler = ori_quat.To_Euler();
        ori_euler = ori_euler.To_Degrees();

        GRAPH("x", ori_euler.x, TOP);
        GRAPH("y", ori_euler.y, TOP);
        GRAPH("z", ori_euler.z, TOP);
        printf("%.6f\n", dt);*/

        printf("Running...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
