#include "main_core_0.h"

auto update_yaw(float curr_yaw, float yaw_rate, float dt) -> float {
  return curr_yaw + yaw_rate * dt;
}

void control_loop() {}

void core_0_task(void* args) {
  uint64_t loop_counter = 0;

  auto controlOutputQueue = static_cast<QueueHandle_t>(args);

  sensor_data my_data{};

  // I2C Bus definition
  i2c::i2c_bus my_bus(SDA, SCL);

  // I2C Device definitions
  bmp280::bmp280 my_bmp(&my_bus);
  icm20948::icm20948 my_icm(&my_bus);
  ina219::ina219 my_ina(&my_bus);
  pca9685::pca9685 my_pca(&my_bus);

  // PID Controller defintions
  pid::pid pid_x(0.65F, 0.45F, 0.00013F, 0.0F, 25.0F);
  pid::pid pid_y(0.65F, 0.45F, 0.00013F, 0.0F, 25.0F);
  pid::pid pid_z(1.2F, 1.0F, 0.0001F, 0.0F, 25.0F);

  // ESC object definitions
  pca9685::pca_esc esc0(&my_pca, 0);
  pca9685::pca_esc esc1(&my_pca, 1);
  pca9685::pca_esc esc2(&my_pca, 2);
  pca9685::pca_esc esc3(&my_pca, 3);

  // FS-A8S Receiver definition
  fsa8s::receiver my_receiver(RC_RX);

  // Orientation definitions
  ori::Quat ori_quat(1.0F, 0.0F, 0.0F, 0.0F);
  ori::Quat ori_setpoint_quat{};

  ori::Vect gyro{};
  ori::Vect ori_setpoint{};
  ori::Vect ori_euler{};
  ori::Vect ori_error{};

  // Time handling definitons
  float dt{};
  int64_t last_time = esp_timer_get_time();
  int64_t delta_us{};
  int64_t now{};

  // Control input definitions
  float yaw{};
  uint8_t throttle{};

  // Setup
  vTaskDelay(pdMS_TO_TICKS(3000));

  my_icm.calibrate_gyro(1000);

  // Main loop
  while (true) {
    loop_counter++;

    // Calculate delta time
    now = esp_timer_get_time();
    delta_us = now - last_time;
    last_time = now;
    dt = static_cast<float>(delta_us) / 1e6F;

    // Update necessary sensors
    my_icm.update();
    my_receiver.update();

    // Compute control inputs and desired state
    my_data.ch1 = static_cast<float>(my_receiver.ch1() - 1500) / (5 * 57.3);
    my_data.ch2 = static_cast<float>(my_receiver.ch2() - 1500) / (5 * 57.3);
    my_data.ch3 = static_cast<float>(my_receiver.ch3());
    my_data.ch4 = static_cast<float>(1500 - my_receiver.ch4()) / (5 * 57.3);
    my_data.ch5 = static_cast<float>(my_receiver.ch5());
    my_data.ch6 = static_cast<float>(my_receiver.ch6());

    yaw = update_yaw(yaw, my_data.ch4, dt);
    throttle = (my_data.ch3 - 1000) / 10;

    // Calculate current orientation
    gyro = ori::Vect(my_icm.gyro_x(), my_icm.gyro_y(), my_icm.gyro_z());
    ori_quat = ori_quat.update(gyro, dt);
    ori_quat = ori_quat.normalise();

    // Calculate error direction
    ori_setpoint = ori::Vect(my_data.ch1, my_data.ch2, yaw);
    ori_setpoint_quat = ori_setpoint.to_quat();
    ori_error = ori_quat.calc_error_axis(ori_setpoint_quat);

    // Check if armed, if not armed: continue
    if (my_data.ch5 <= 1500) {
      pid_x.reset();
      pid_y.reset();
      pid_z.reset();

      my_data.esc0 = 0;
      my_data.esc1 = 0;
      my_data.esc2 = 0;
      my_data.esc3 = 0;

      continue;
    }

    // Avoid integral windup by only enabling when flying
    if (my_data.ch3 < 1050) {
      pid_x.enable_disable_interal(false);
      pid_y.enable_disable_interal(false);
      pid_z.enable_disable_interal(false);
    } else {
      pid_x.enable_disable_interal(true);
      pid_y.enable_disable_interal(true);
      pid_z.enable_disable_interal(true);
    }

    // Check if stabalised or acro mode
    if (my_data.ch6 < 1250) {
      pid_x.set_setpoint((my_data.ch1));
      pid_y.set_setpoint((my_data.ch2));
      pid_z.set_setpoint((my_data.ch4));
    } else {
      pid_x.set_setpoint(ori_error.x() * 10);
      pid_y.set_setpoint(ori_error.y() * 10);
      pid_z.set_setpoint(ori_error.z() * 10);
    }

    //  Calculate control outputs
    my_data.pid_x = pid_x.update(gyro.x(), dt);
    my_data.pid_y = pid_y.update(gyro.y(), dt);
    my_data.pid_z = pid_z.update(gyro.z(), dt);

    // Update ESC values (throttle %)
    my_data.esc0 =
        throttle + 10 - my_data.pid_x - my_data.pid_y + my_data.pid_z;
    my_data.esc1 =
        throttle + 10 + my_data.pid_x - my_data.pid_y - my_data.pid_z;
    my_data.esc2 =
        throttle + 10 + my_data.pid_x + my_data.pid_y + my_data.pid_z;
    my_data.esc3 =
        throttle + 10 - my_data.pid_x + my_data.pid_y - my_data.pid_z;

    ESP_ERROR_CHECK(esc0.set_throttle(my_data.esc0));
    ESP_ERROR_CHECK(esc1.set_throttle(my_data.esc1));
    ESP_ERROR_CHECK(esc2.set_throttle(my_data.esc2));
    ESP_ERROR_CHECK(esc3.set_throttle(my_data.esc3));

    // Data log every 50 loops to keep control loop fast
    if (loop_counter % 50 != 0) {
      continue;
    }

    my_bmp.update();

    ori_euler = ori_quat.to_euler();

    my_data.ori_x = ori_euler.x();
    my_data.ori_y = ori_euler.y();
    my_data.ori_z = ori_euler.z();

    my_data.set_x = ori_setpoint.x();
    my_data.set_y = ori_setpoint.y();
    my_data.set_z = ori_setpoint.z();

    my_data.gyro_x = gyro.x();
    my_data.gyro_y = gyro.y();
    my_data.gyro_z = gyro.z();

    my_data.dt = dt;

    my_data.temp = my_bmp.get_temperature();
    my_data.height = my_bmp.get_altitude();

    my_data.voltage = my_ina.get_voltage();
    my_data.current = my_ina.get_current();
    my_data.power = my_ina.get_power();

    xQueueOverwrite(controlOutputQueue, &my_data);
  }
}
