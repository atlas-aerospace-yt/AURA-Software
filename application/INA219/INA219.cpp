#include "INA219.h"

void ina219::ina219::_calibrate(void)
{
    _master_bus->write_bytes_i2c<uint16_t, 2>(_ina_handle, INA_CALIB, CALIB_VALUE);
}

float ina219::ina219::get_voltage(void)
{
    uint16_t voltage;
    voltage = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_BUS_V);
    return static_cast<float>(voltage) / LSB_BUS_V;
}

float ina219::ina219::get_current(void)
{
    uint16_t current;
    current = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_CURRENT);
    return static_cast<float>(current) / LSB_CURRENT;
}

float ina219::ina219::get_power(void)
{
    uint16_t power;
    power = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_POWER);
    return static_cast<float>(power) / LSB_POWER;
}
