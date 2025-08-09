//
// BMP280 I2C library.
//
// Datasheet for reference:
// https://www.mouser.co.uk/datasheet/2/783/BST_BMP280_DS001-1509562.pdf
//
#include "BMP280.h"

namespace bmp280 {

bmp280::bmp280(i2c::i2c_bus* master_bus, uint8_t addr) : _master_bus(master_bus), _bmp_addr(addr) {
  _bmp_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  _bmp_config.scl_speed_hz = I2C_SPEED;
  _bmp_config.device_address = _bmp_addr;

  ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(&_bmp_config, &_bmp_handle));

  _set_mode();
  _calibrate();
  update();
  _initial_pressure = get_pressure();
}

// NOLINTBEGIN(misc-magic-numbers, readability-magic-numbers)
void bmp280::_set_mode() {
  uint8_t ctrl_meas_init = 0;
  ctrl_meas_init |= static_cast<uint8_t>(TEMP_OVERSAMPLE << 5);
  ctrl_meas_init |= static_cast<uint8_t>(PRESS_OVERSAMPLE << 2);
  ctrl_meas_init |= static_cast<uint8_t>(NORMAL_MODE);

  ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_bmp_handle, BMP_CTRL_MEAS, ctrl_meas_init));
}
// NOLINTEND(misc-magic-numbers, readability-magic-numbers)

void bmp280::_calibrate() {
  _dig_t1 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_T1);
  _dig_t2 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_T2);
  _dig_t3 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_T3);
  _dig_p1 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P1);
  _dig_p2 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P2);
  _dig_p3 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P3);
  _dig_p4 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P4);
  _dig_p5 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P5);
  _dig_p6 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P6);
  _dig_p7 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P7);
  _dig_p8 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P8);
  _dig_p9 = _master_bus->read_bytes_i2c<uint16_t, 2>(_bmp_handle, BMP_DIG_P9);
}

// NOLINTBEGIN(misc-magic-numbers, readability-magic-numbers,
// readability-convert-member-functions-to-static)
// clang-format off
void bmp280::_get_compensated_temp() {
  int32_t var1;
  int32_t var2;

  var1 = ((((_raw_temp >> 3) - (static_cast<int32_t>(_dig_t1) << 1)) *
          static_cast<int32_t>(_dig_t2)) >> 11);
  var2 = (((((_raw_temp >> 4) - static_cast<int32_t>(_dig_t1)) *
            ((_raw_temp >> 4) - static_cast<int32_t>(_dig_t1))) >> 12) *
          static_cast<int32_t>(_dig_t3)) >> 14;

  _comp_temp = static_cast<int32_t>(var1 + var2);
}
// clang-format on

void bmp280::set_height(float acc_height) {
  // TODO(Alexander)
}

void bmp280::update() {
  uint64_t temp_and_pressure;

  temp_and_pressure = _master_bus->read_bytes_i2c<uint64_t, 6>(_bmp_handle, BMP_PRESS_DATA);
  _raw_temp = static_cast<int32_t>((temp_and_pressure >> 4) & TEMP_AND_PRESS_MASK);
  _raw_pressure = static_cast<int32_t>((temp_and_pressure >> 28) & TEMP_AND_PRESS_MASK);
  _get_compensated_temp();
}

[[nodiscard]] auto bmp280::get_pressure() const -> float {
  int64_t var1;
  int64_t var2;
  int64_t pressure;

  var1 = (static_cast<int64_t>(_comp_temp)) - 128000;
  var2 = var1 * var1 * static_cast<int64_t>(_dig_p6);
  var2 = var2 + ((var1 * static_cast<int64_t>(_dig_p5)) << 17);
  var2 = var2 + ((static_cast<int64_t>(_dig_p4)) << 35);
  var1 = ((var1 * var1 * static_cast<int64_t>(_dig_p3)) >> 8) +
         ((var1 * static_cast<int64_t>(_dig_p2)) << 12);
  var1 = ((((static_cast<int64_t>(1)) << 47) + var1)) * (static_cast<int64_t>(_dig_p1)) >> 33;

  if (var1 == 0) {
    return 0.0F;
  }

  pressure = 1048576 - _raw_pressure;
  pressure = (((pressure << 31) - var2) * 3125) / var1;
  var1 = ((static_cast<int64_t>(_dig_p9)) * (pressure >> 13) * (pressure >> 13)) >> 25;
  var2 = ((static_cast<int64_t>(_dig_p8)) * pressure) >> 19;
  pressure = ((pressure + var1 + var2) >> 8) + ((static_cast<int64_t>(_dig_p7)) << 4);

  return static_cast<float>(pressure) / 256000.0F;
}

[[nodiscard]] auto bmp280::get_temperature() const -> float {
  return static_cast<float>((_comp_temp * 5 + 128) >> 8) / 100.0F;
}

[[nodiscard]] auto bmp280::get_altitude() const -> float {
  const float pressure = get_pressure();
  const float curr_press_hpa = pressure / 10.0F;
  const float init_press_hpa = _initial_pressure / 10.0F;

  return static_cast<float>(44330 * (1.0 - pow(curr_press_hpa / init_press_hpa, 0.1903F)));
}
// NOLINTEND(misc-magic-numbers, readability-magic-numbers,
// readability-convert-member-functions-to-static)
}  // namespace bmp280
