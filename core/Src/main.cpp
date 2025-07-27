// LED Blink Tutorial

/*
#include "main.h"

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

#include "esp_log.h"

#define LOG_TAG "MAIN"

static Main my_main;

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(my_main.setup());

    while (true)
    {
        my_main.loop();
    }
}

esp_err_t Main::setup(void)
{
    esp_err_t status{ESP_OK};

    ESP_LOGI(LOG_TAG, "Setup!");

    status |= led.init();

    return status;
}

void Main::loop(void)
{
    ESP_LOGI(LOG_TAG, "LED ON");
    led.set(true);
    vTaskDelay(milis_ticks(1000));
    ESP_LOGI(LOG_TAG, "LED OFF");
    led.set(false);
    vTaskDelay(milis_ticks(1000));
}
*/

/*
// I2C Scanner Tutorial

//Device addresses:
//PCA - 0x40!
//ICM - 0x68!
//??? - 0x70!
//BMP - 0x76!

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define LOG_TAG "MAIN"

#include "esp_log.h"
#include "main.h"

TaskHandle_t CheckAddressHandle = NULL;

// Initialise I2C bus

extern "C" void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7
    };
 
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    esp_err_t err;

    while (true)
    {
        printf(LOG_TAG, "Scanning...");

        for (uint8_t i=0x03; i<0x78; i++)
        {
            err = i2c_master_probe(bus_handle, i, 1000);
            if (err == ESP_OK)
            {
                printf("Device Found at 0x%X!\r\n", i);
            }
        }

        printf("Done Scanning!\n\r");

        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
*/

// Reading I2C devices example
/*
#include "esp_log.h"
#include "main.h"

TaskHandle_t CheckAddressHandle = NULL;

// Initialise I2C bus
esp_err_t read_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

uint16_t read_uint16_t_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr)
{
    uint8_t data[2];

    read_byte_i2c(dev_handle, reg_addr, data, 2);

    return (uint16_t) ((uint16_t)data[0] << 8) | data[1];
}

int16_t read_int16_t_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr)
{
    uint8_t data[2];

    read_byte_i2c(dev_handle, reg_addr, data, 2);

    return (int16_t) ((uint16_t)data[0] << 8) | data[1];
}

int32_t read_int20_t_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr)
{
    uint8_t data[3];

    read_byte_i2c(dev_handle, reg_addr, data, 3);

    return (int32_t) ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
}

esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

extern "C" void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7
    };
 
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP_ADDR,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    while (true)
    {
        printf("Starting new reading...\n\n");

        uint8_t ctrl_meas_reg[2] = {0xF4, 0x27};
        i2c_master_transmit(dev_handle, ctrl_meas_reg, 2, pdMS_TO_TICKS(100));

        uint16_t dig_t1, dig_p1;
        int16_t dig_t2, dig_t3, dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;
        int32_t temp, pressure;

        dig_t1 = read_uint16_t_i2c(dev_handle, BMP_DIG_T1);
        dig_t2 = read_int16_t_i2c(dev_handle, BMP_DIG_T2);
        dig_t3 = read_int16_t_i2c(dev_handle, BMP_DIG_T3);
        dig_p1 = read_uint16_t_i2c(dev_handle, BMP_DIG_P1);
        dig_p2 = read_int16_t_i2c(dev_handle, BMP_DIG_P2);
        dig_p3 = read_int16_t_i2c(dev_handle, BMP_DIG_P3);
        dig_p4 = read_uint16_t_i2c(dev_handle, BMP_DIG_P4);
        dig_p5 = read_int16_t_i2c(dev_handle, BMP_DIG_P5);
        dig_p6 = read_int16_t_i2c(dev_handle, BMP_DIG_P6);
        dig_p7 = read_uint16_t_i2c(dev_handle, BMP_DIG_P7);
        dig_p8 = read_int16_t_i2c(dev_handle, BMP_DIG_P8);
        dig_p9 = read_int16_t_i2c(dev_handle, BMP_DIG_P9);

        temp = read_int20_t_i2c(dev_handle, BMP_TEMP_DATA);
        pressure = read_int20_t_i2c(dev_handle, BMP_PRESS_DATA);

        printf("Data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %ld, %ld\n", dig_t1, dig_t2, dig_t3, dig_p1, dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9, temp, pressure);
        printf("Done!\n\n");

        int32_t var1, var2, temp_c;
        var1 = ((((temp >> 3) - ((int32_t)dig_t1 << 1))) * ((int32_t)dig_t2)) >> 11;

        var2 = (((((temp >> 4) - ((int32_t)dig_t1)) * ((temp >> 4) - ((int32_t)dig_t1))) >> 12) * ((int32_t)dig_t3)) >> 14;

        temp_c = ((var1 + var2) * 5 + 128) >> 8;

        printf("Final temp is %f\n\n", static_cast<float>(temp_c)/100.0f);

        int64_t var1_p, var2_p, pressure_pa;

        var1_p = ((int64_t)temp_c*100) - 128000;
        var2_p = var1_p * var1_p * (int64_t)dig_p6;
        var2_p = var2_p + ((var1_p * (int64_t)dig_p5) << 17);
        var2_p = var2_p + (((int64_t)dig_p4) << 35);
        var1_p = ((var1_p * var1_p * (int64_t)dig_p3) >> 8) + ((var1_p * (int64_t)dig_p2) << 12);
        var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)dig_p1) >> 33;
        pressure_pa = 1048576 - pressure;
        pressure_pa = (((pressure_pa << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)dig_p9) * (pressure_pa >> 13) * (pressure_pa >> 13)) >> 25;
        var2_p = (((int64_t)dig_p8) * pressure_pa) >> 19;

        pressure_pa = ((pressure_pa + var1_p + var2_p) >> 8) + (((int64_t)dig_p7) << 4);

        printf("Final pressure is %f\n\n", static_cast<float>(pressure_pa)/256000.0f);

        vTaskDelay(milis_ticks(100));
    }

    vTaskDelete(NULL);
}

*/
#include "main.h"

extern "C" void app_main(void)
{

    i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);
    bmp280::bmp280 my_bmp(&my_bus);
    float data=0;

    while (true)
    {
        my_bmp.update();
        data = my_bmp.get_pressure();

        printf("Recorded temperature: %fkPa\n", data);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}