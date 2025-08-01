#include "main.h"

extern "C" void app_main(void)
{

    i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);
    bmp280::bmp280 my_bmp(&my_bus);
    icm20948::icm20948 my_icm(&my_bus);
    ina219::ina219 my_ina(&my_bus);

    gpio_reset_pin(GPIO_NUM_5);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

    while (true)
    {
        printf("Voltage: %f\n", my_ina.get_voltage());
        printf("Current: %f\n", my_ina.get_current());
        printf("Power: %f\n\n", my_ina.get_power());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}