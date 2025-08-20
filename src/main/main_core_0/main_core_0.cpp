#include "main_core_0.h"

void control_loop() {}

void core_0_task(void* args) {
  uint64_t loop_counter = 0;

  auto controlOutputQueue = static_cast<QueueHandle_t>(args);

  sensor_data my_data{};

  i2c::i2c_bus my_bus(GPIO_NUM_8, GPIO_NUM_9);

  bmp280::bmp280 my_bmp(&my_bus);
  icm20948::icm20948 my_icm(&my_bus);
  ina219::ina219 my_ina(&my_bus);

  fsa8s::receiver my_receiver(GPIO_NUM_1);

  ori::Quat ori_quat(1.0F, 0.0F, 0.0F, 0.0F);

  ori::Vect gyro;
  ori::Vect ori_euler;

  float dt{};
  float x_offs{};
  float y_offs{};
  float z_offs{};

  int64_t delta_us{};
  int64_t now{};

  int64_t last_time = esp_timer_get_time();

  for (int i = 0; i < 1000; i++) {
    my_icm.update();
    x_offs += my_icm.gyro_x();
    y_offs += my_icm.gyro_y();
    z_offs += my_icm.gyro_z();
    vTaskDelay(pdMS_TO_TICKS(3));
  }

  x_offs /= 1000.0F;
  y_offs /= 1000.0F;
  z_offs /= 1000.0F;

  while (true) {
    loop_counter++;

    now = esp_timer_get_time();
    delta_us = now - last_time;
    last_time = now;

    if (delta_us < 0) {
      delta_us = 0;
    }

    dt = static_cast<float>(delta_us) / 1e6F;

    my_icm.update();
    my_receiver.update();

    gyro = ori::Vect(my_icm.gyro_x() - x_offs, my_icm.gyro_y() - y_offs, my_icm.gyro_z() - z_offs);

    ori_quat = ori_quat.update(gyro, dt);

    ori_euler = ori_quat.to_euler();
    ori_euler = ori_euler.to_degrees();

    my_data.gyro_x = ori_euler.x();
    my_data.gyro_y = ori_euler.y();
    my_data.gyro_z = ori_euler.z();

    my_data.dt = dt;

    my_data.ch1 = my_receiver.ch1();
    my_data.ch2 = my_receiver.ch2();
    my_data.ch3 = my_receiver.ch3();
    my_data.ch4 = my_receiver.ch4();
    my_data.ch5 = my_receiver.ch5();
    my_data.ch6 = my_receiver.ch6();

    if (loop_counter % 50 == 0) {
      my_bmp.update();
      my_data.temp = my_bmp.get_temperature();
      my_data.height = my_bmp.get_altitude();

      my_data.voltage = my_ina.get_voltage();
      my_data.current = my_ina.get_current();
      my_data.power = my_ina.get_power();
      xQueueOverwrite(controlOutputQueue, &my_data);
    }
  }
}
