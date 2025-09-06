#include "main_core_0.h"

#include "Madgwick.h"

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
  pid::pid pid_x(0.65F, 0.75F, 0.00013F, 0.0F, 25.0F);
  pid::pid pid_y(0.65F, 0.75F, 0.00013F, 0.0F, 25.0F);
  pid::pid pid_z(1.2F, 1.0F, 0.0001F, 0.0F, 25.0F);

  // ESC object definitions
  pca9685::pca_esc esc0(&my_pca, 0);
  pca9685::pca_esc esc1(&my_pca, 1);
  pca9685::pca_esc esc2(&my_pca, 2);
  pca9685::pca_esc esc3(&my_pca, 3);

  // FS-A8S Receiver definition
  fsa8s::receiver my_receiver(RC_RX);

  // Orientation definitions
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

  float min_x{}, max_x{}, min_y{}, max_y{}, min_z = 60, max_z{};

  // Setup
  vTaskDelay(pdMS_TO_TICKS(3000));

  my_icm.calibrate_gyro(1000);

  ori::Vect meas_mag;
  ori::Vect meas_acc;

  my_icm.update();
  my_icm.update_mag();
  meas_acc = ori::Vect(my_icm.acc_x(), my_icm.acc_y(), my_icm.acc_z());
  meas_mag = ori::Vect(my_icm.mag_x(), my_icm.mag_y(), my_icm.mag_z());
  meas_acc = meas_acc / meas_acc.mag();
  meas_mag = meas_mag / meas_mag.mag();

  ori::Quat ori_quat =
      ori::Vect::quat_from_two_vect(meas_acc, ori::Vect(0.0F, 0.0F, 1.0F));
  ori::Quat meas_mag_q =
      ori::Quat(0.0F, meas_mag.x(), meas_mag.y(), meas_mag.z());
  ori::Quat mag_exp = ori_quat.conjugate() * meas_mag_q * ori_quat;

  meas_mag = ori::Vect(mag_exp.i(), mag_exp.j(), mag_exp.k());

  ori::madgwick filter(0.05, meas_mag, ori::Vect(0.0F, 0.0F, 1.0F));

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
    my_icm.update_mag();
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

    // TESTING
    ori::Vect meas_acc(my_icm.acc_x(), my_icm.acc_y(), my_icm.acc_z());
    ori::Vect meas_mag(my_icm.mag_x(), my_icm.mag_y(), my_icm.mag_z());

    float acc_mag = meas_acc.mag();
    meas_acc = meas_acc / acc_mag;
    float mag_mag = meas_mag.mag();
    meas_mag = meas_mag / mag_mag;

    if (acc_mag > 9.0F && acc_mag < 10.6F) {
      ori_quat = filter.update(ori_quat, gyro, meas_mag, meas_acc, dt);
    } else {
      ori_quat = ori_quat.update(gyro, dt);
    }
    // END

    // Calculate error direction
    ori::Quat q_roll =
        ori::Quat(cos(my_data.ch1 / 2), sin(my_data.ch1 / 2), 0, 0);
    ori::Quat q_pitch =
        ori::Quat(cos(my_data.ch2 / 2), 0, sin(my_data.ch2 / 2), 0);
    ori::Quat q_yaw = ori::Quat(cos(yaw / 2), 0, 0, sin(yaw / 2));
    ori_setpoint_quat = q_yaw * q_pitch * q_roll;

    ori_error = ori_quat.calc_error(ori::Quat(1.0F, 0.0F, 0.0F, 0.0F));

    ori_euler = ori_quat.to_euler();
    ori_euler = ori_euler.to_degrees();

    // Check if armed, if not armed: continue
    if (my_data.ch5 <= 1500) {
      pid_x.reset();
      pid_y.reset();
      pid_z.reset();

      ESP_ERROR_CHECK(esc0.set_throttle(0));
      ESP_ERROR_CHECK(esc1.set_throttle(0));
      ESP_ERROR_CHECK(esc2.set_throttle(0));
      ESP_ERROR_CHECK(esc3.set_throttle(0));

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
      pid_x.set_setpoint((ori_error.x() - my_data.ch1) * 5);
      pid_y.set_setpoint((ori_error.y() - my_data.ch2) * 5);
      pid_z.set_setpoint((ori_error.z() - yaw) * 5);
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

    my_data.set_x = ori_error.x() * 5;
    my_data.set_y = ori_error.y() * 5;
    my_data.set_z = ori_error.z() * 5;

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
