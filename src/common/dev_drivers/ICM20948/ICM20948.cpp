#include "ICM20948.h"

namespace icm20948 {

icm20948::icm20948(i2c::i2c_bus* master_bus) : _master_bus(master_bus) {
  _icm_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  _icm_config.scl_speed_hz = I2C_SPEED;
  _icm_config.device_address = ICM_ADDR;

  ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(&_icm_config, &_icm_handle));

  _set_i2c_bypass();

  _mag_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  _mag_config.scl_speed_hz = I2C_SPEED;
  _mag_config.device_address = MAG_ADDR;

  ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(&_mag_config, &_mag_handle));

  _wake_device();
  _configure_gyro();
  _configure_accel();
  _configure_mag();
}

void icm20948::_wake_device() {
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_PWR_MGMT_1, PWR_MGMT_1));
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_PWR_MGMT_2, PWR_MGMT_2));
}

void icm20948::_select_register(uint8_t reg) {
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_REG_BANK_SEL, reg));
}

void icm20948::_configure_gyro() {
  _select_register(BANK_SEL_2);

  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_GYRO_SMPLRT_DIV, GYRO_SMPLRT_DIV));
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_GYRO_CONFIG_1, GYRO_CONFIG_1));
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_GYRO_CONFIG_2, GYRO_CONFIG_2));

  _select_register(BANK_SEL_0);
}

void icm20948::_configure_accel() {
  _select_register(BANK_SEL_2);

  ESP_ERROR_CHECK(
      _master_bus->write_byte_i2c(_icm_handle, ICM_ACCEL_SMPLRT_DIV_1, ACCEL_SMPLRT_DIV));
  ESP_ERROR_CHECK(
      _master_bus->write_byte_i2c(_icm_handle, ICM_ACCEL_SMPLRT_DIV_2, ACCEL_SMPLRT_DIV));
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_ACCEL_CONFIG_1, ACCEL_CONFIG_1));
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_ACCEL_CONFIG_2, ACCEL_CONFIG_2));

  _select_register(BANK_SEL_0);
}

void icm20948::_set_i2c_bypass() {
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_icm_handle, ICM_INT_PIN_CFG, INT_PIN_CFG));
}

void icm20948::_configure_mag() {
  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_mag_handle, MAG_CNTRL_2, MAG_SLEEP));

  vTaskDelay(pdMS_TO_TICKS(10));

  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_mag_handle, MAG_CNTRL_2, MAG_CONT_MODE_4));

  vTaskDelay(pdMS_TO_TICKS(10));
}

// NOLINTBEGIN(misc-magic-numbers, readability-magic-numbers)
void icm20948::update() {
  const uint64_t acc = _master_bus->read_bytes_i2c<uint64_t, 6>(_icm_handle, ICM_ACC_XOUT);
  const uint64_t gyro = _master_bus->read_bytes_i2c<uint64_t, 6>(_icm_handle, ICM_GYRO_XOUT);

  _acc_x = static_cast<float>(static_cast<int16_t>((acc >> 32) & AXIS_MASK)) / CONVERT_ACCEL;
  _acc_y = static_cast<float>(static_cast<int16_t>((acc >> 16) & AXIS_MASK)) / CONVERT_ACCEL;
  _acc_z = static_cast<float>(static_cast<int16_t>((acc >> 0) & AXIS_MASK)) / CONVERT_ACCEL;

  _gyro_x = static_cast<float>(static_cast<int16_t>((gyro >> 32) & AXIS_MASK)) / CONVERT_GYRO;
  _gyro_y = static_cast<float>(static_cast<int16_t>((gyro >> 16) & AXIS_MASK)) / CONVERT_GYRO;
  _gyro_z = static_cast<float>(static_cast<int16_t>((gyro >> 0) & AXIS_MASK)) / CONVERT_GYRO;
}

//
// NOTE: mag must be read in the order shown - ST1, data, ST2
//
void icm20948::update_mag() {
  _master_bus->read_bytes_i2c<uint8_t, 1>(_mag_handle, MAG_ST1);

  const uint64_t mag = _master_bus->read_bytes_i2c<uint64_t, 6>(_mag_handle, MAG_HXL, true);

  _master_bus->read_bytes_i2c<uint8_t, 1>(_mag_handle, MAG_ST2);

  _mag_x = static_cast<float>(static_cast<int16_t>((mag >> 32) & AXIS_MASK)) * CONVERT_MAG;
  _mag_y = static_cast<float>(static_cast<int16_t>((mag >> 16) & AXIS_MASK)) * CONVERT_MAG;
  _mag_z = static_cast<float>(static_cast<int16_t>(mag & AXIS_MASK)) * CONVERT_MAG - MAG_Z_OFFS;
}
// NOLINTEND(misc-magic-numbers, readability-magic-numbers)
}  // namespace icm20948
