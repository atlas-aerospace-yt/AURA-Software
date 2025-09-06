/**
 *
 * ICM20948 datasheet -
 * https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
 *
 * AK09916 datasheet -
 * https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
 *
 */
#pragma once

#include <cmath>

#include "I2C.h"

namespace icm20948 {

constexpr uint32_t I2C_SPEED = 1100000;
constexpr uint32_t I2C_SPEED_MAG = 400000;

/* ICM20948 default I2C address on AURA PCB */
constexpr uint8_t ICM_ADDR = 0x68;

/* Expected chip id (whoami) */
constexpr uint8_t CHIPID = 0x00;

/* Register bank selection*/
constexpr uint8_t ICM_REG_BANK_SEL = 0x7F;
constexpr uint8_t BANK_SEL_0 = 0x00;
constexpr uint8_t BANK_SEL_1 = 0x10;
constexpr uint8_t BANK_SEL_2 = 0x20;
constexpr uint8_t BANK_SEL_3 = 0x30;

/* Setup register bank 0 addresses */
constexpr uint8_t ICM_USER_CTRL = 0x03;
constexpr uint8_t ICM_LP_CONFIG = 0x05;
constexpr uint8_t ICM_PWR_MGMT_1 = 0x06;
constexpr uint8_t ICM_PWR_MGMT_2 = 0x07;
constexpr uint8_t ICM_INT_PIN_CFG = 0x0F;

/* Setup register bank 2 addresses */
constexpr uint8_t ICM_GYRO_CONFIG_1 = 0x01;
constexpr uint8_t ICM_GYRO_CONFIG_2 = 0x02;
constexpr uint8_t ICM_GYRO_SMPLRT_DIV = 0x00;

constexpr uint8_t ICM_ACCEL_CONFIG_1 = 0x14;
constexpr uint8_t ICM_ACCEL_CONFIG_2 = 0x15;
constexpr uint8_t ICM_ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t ICM_ACCEL_SMPLRT_DIV_2 = 0x11;

/* Accel register addresses (BANK 0) */
constexpr uint8_t ICM_ACC_XOUT = 0x2D;
constexpr uint8_t ICM_ACC_YOUT = 0x2F;
constexpr uint8_t ICM_ACC_ZOUT = 0x31;

/* GYRO register addresses (BANK 0) */
constexpr uint8_t ICM_GYRO_XOUT = 0x33;
constexpr uint8_t ICM_GYRO_YOUT = 0x35;
constexpr uint8_t ICM_GYRO_ZOUT = 0x37;

constexpr uint8_t ICM_TEMP_OUT = 0x39;

/* AK09916 Magnetometer definitions*/
constexpr uint8_t MAG_ADDR = 0x0C;

constexpr uint8_t MAG_ST1 = 0x10;
constexpr uint8_t MAG_ST2 = 0x18;

constexpr uint8_t MAG_CNTRL_2 = 0x31;
constexpr uint8_t MAG_CNTRL_3 = 0x32;

constexpr uint8_t MAG_HXL = 0x11;
constexpr uint8_t MAG_HXH = 0x12;
constexpr uint8_t MAG_HYL = 0x13;
constexpr uint8_t MAG_HTH = 0x14;
constexpr uint8_t MAG_HZL = 0x15;
constexpr uint8_t MAG_HZH = 0x16;

/* Initial settings values */
constexpr uint8_t PWR_MGMT_1 = 0x01;
constexpr uint8_t PWR_MGMT_2 = 0x00;
constexpr uint8_t LP_CONFIG = 0x00;
constexpr uint8_t INT_PIN_CFG = 0x52;

constexpr uint8_t GYRO_CONFIG_1 = 0x37;
constexpr uint8_t GYRO_CONFIG_2 = 0x02;
constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;

constexpr uint8_t ACCEL_CONFIG_1 = 0x37;
constexpr uint8_t ACCEL_CONFIG_2 = 0x02;
constexpr uint8_t ACCEL_SMPLRT_DIV = 0x00;

constexpr uint8_t MAG_SLEEP = 0x00;
constexpr uint8_t MAG_CONT_MODE_4 = 0x08;

constexpr uint16_t AXIS_MASK = 0xffff;

/* Conversion definitions */
constexpr float CONVERT_ACCEL = 208.767F;
constexpr float CONVERT_GYRO = 938.74F;
constexpr float CONVERT_MAG = 0.15F;

constexpr float MAG_X_OFFS = -33.0F;
constexpr float MAG_Y_OFFS = 17.13F;
constexpr float MAG_Z_OFFS = 64.1F;

constexpr float MAG_X_SCALE = 0.022989F;
constexpr float MAG_Y_SCALE = 0.021966F;
constexpr float MAG_Z_SCALE = 0.020964F;

//
// A wrapper class to handle communication with an ICM20948 device connected via
// the I2c protocol
//
class icm20948 {
 private:
  i2c::i2c_bus* _master_bus;

  // ICM20948 I2C handle
  i2c_master_dev_handle_t _icm_handle;
  i2c_device_config_t _icm_config = {};

  // Internal AK09916 I2C handle
  i2c_master_dev_handle_t _mag_handle;
  i2c_device_config_t _mag_config = {};

  // IMU data floats
  float _gyro_x{};
  float _gyro_y{};
  float _gyro_z{};
  float _acc_x{};
  float _acc_y{};
  float _acc_z{};
  float _mag_x{};
  float _mag_y{};
  float _mag_z{};

  // Offset definitions
  float _x_offs{};
  float _y_offs{};
  float _z_offs{};

  //
  // Write to PWR_MGMT_1 and PWR_MGMT_2 to turn on accelerometer and gyroscope
  //
  void _wake_device();

  //
  // Set the gyroscope configuration to +-2000deg/s and configure the lowpass
  // filter to a cuttoff frequency of 5Hz
  //
  void _configure_gyro();

  //
  // Set the accelerometer configuration to +-16g and configure the lowpass
  // filter to a cuttoff frequency of 5Hz
  //
  void _configure_accel();

  //
  // The AK0991 is on a seperate I2C bus controlled by the ICM20948 so this
  // makes the AK09916 show up on the same bus the ICM20948 is on
  //
  void _set_i2c_bypass();

  //
  // Configure the magnetometer to wake the device up and set it to read data
  // continuously
  //
  void _configure_mag();

  //
  // A convinience function which selects which register bank you are
  // reading/writing to
  //
  // NOTE: you must select bank 0 with BANK_SEL_0 after changing as the ICM20948
  // does not
  //       automatically deselect the chosen bank
  //
  void _select_register(uint8_t reg);

 public:
  explicit icm20948(i2c::i2c_bus* master_bus);

  [[nodiscard]] auto gyro_x() const -> float { return _gyro_x; }
  [[nodiscard]] auto gyro_y() const -> float { return _gyro_y; }
  [[nodiscard]] auto gyro_z() const -> float { return _gyro_z; }

  [[nodiscard]] auto acc_x() const -> float { return _acc_x; }
  [[nodiscard]] auto acc_y() const -> float { return _acc_y; }
  [[nodiscard]] auto acc_z() const -> float { return _acc_z; }

  [[nodiscard]] auto mag_x() const -> float { return _mag_x; }
  [[nodiscard]] auto mag_y() const -> float { return _mag_y; }
  [[nodiscard]] auto mag_z() const -> float { return _mag_z; }

  //
  // Read the accelerometer readings in one go and then the gyro readings in one
  // go and convert to a float
  //
  auto update() -> void;

  //
  // Read all magnetometer readings in one go following the specific read order:
  // 1 - Read ST1 to check if new data is ready
  // 2 - Read all of the data in one go (HXH to HZL)
  // 3 - Read ST2 to check if overflow occured
  //
  auto update_mag() -> void;

  //
  // Average N samples to calculate the offsets for the gyro
  //
  // The device must be completely still during this process
  //
  // @param n_samples the number of samples to take
  //
  auto calibrate_gyro(uint16_t n_samples) -> void;
};

}  // namespace icm20948
