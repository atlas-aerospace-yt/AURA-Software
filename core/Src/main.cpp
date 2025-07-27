#include "main.h"

extern "C" void app_main(void)
{

    i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);
    bmp280::bmp280 my_bmp(&my_bus);
    icm20948::icm20948 my_icm(&my_bus);

    float data;

    while (true)
    {
        /*
        my_bmp.update();

        data = my_bmp.get_temperature();
        printf("Recorded temperature: %fC\n", data);
        data = my_bmp.get_pressure();
        printf("Recorded pressure: %fkPa\n", data);
        data = my_bmp.get_altitude();
        printf("Recorded altiitude: %fm\n", data);
        */
        ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, PWR_MGMT_1, ON_MODE));
        ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, PWR_MGMT_2, ON_MODE));
  
        int16_t icm_test;
        icm_test = my_bus.read_bytes_i2c<int16_t,2>(my_icm._icm_handle, ICM_GYRO_XOUT);
        printf("%d\n", icm_test);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}