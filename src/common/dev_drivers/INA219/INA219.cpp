#include "INA219.h"

namespace ina219 {

ina219::ina219(i2c::i2c_bus* master_bus) : _master_bus(master_bus) {
  _ina_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  _ina_config.scl_speed_hz = I2C_SPEED;
  _ina_config.device_address = INA_ADDR;

  ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(&_ina_config, &_ina_handle));

  _calibrate();
}

void ina219::_calibrate() {
  uint8_t calib_buffer[2] = {CALIB_VALUE_1, CALIB_VALUE_2};
  _master_bus->write_bytes_i2c<2>(_ina_handle, INA_CALIB, calib_buffer);
}

auto ina219::get_voltage() const -> float {
  const uint16_t voltage = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_BUS_V);
  return static_cast<float>(voltage) / LSB_BUS_V;
}

auto ina219::get_current() const -> float {
  const uint16_t current = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_CURRENT);
  const float current_f = static_cast<float>(current) / LSB_CURRENT;
  if (current_f > MAX_CURRENT) {
    return 0.0F;
  }
  return current_f;
}

auto ina219::get_power() const -> float {
  const uint16_t power = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_POWER);
  return static_cast<float>(power) / LSB_POWER;
}

}  // namespace ina219
