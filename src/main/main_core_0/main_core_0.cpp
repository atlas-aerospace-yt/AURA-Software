#include "main_core_0.h"

void control_loop(void)
{

}

void core_0_task(void *args)
{
    QueueHandle_t controlOutputQueue = (QueueHandle_t)args;

    sensor_data my_data;

    i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);

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
    z_offs /= 1000.0f;

    while (true)
    {
        now = esp_timer_get_time();
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

        my_data.gyro_x = ori_euler.x;
        my_data.gyro_y = ori_euler.y;
        my_data.gyro_z = ori_euler.z;
        my_data.dt = dt;

        //GRAPH("x", ori_euler.x, TOP);
        //GRAPH("y", ori_euler.y, TOP);
        //GRAPH("z", ori_euler.z, TOP);
        //printf("%.6f\n", dt);

        xQueueOverwrite(controlOutputQueue, &my_data);
    }
}
