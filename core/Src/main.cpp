#include "main.h"

extern "C" void app_main(void)
{

    i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);
    bmp280::bmp280 my_bmp(&my_bus);
    icm20948::icm20948 my_icm(&my_bus);
    //ina219::ina219 my_ina(&my_bus);

    gpio_reset_pin(GPIO_NUM_5);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

    while (true)
    {

        my_icm.update();
        my_icm.update_mag();

        printf("Gyro: %f %f %f\n", my_icm.gyro_x, my_icm.gyro_y, my_icm.gyro_z);
        printf("Accel: %f %f %f\n", my_icm.acc_x, my_icm.acc_y, my_icm.acc_z);
        printf("Mag: %f %f %f\n\n", my_icm.mag_x, my_icm.mag_y, my_icm.mag_z);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
