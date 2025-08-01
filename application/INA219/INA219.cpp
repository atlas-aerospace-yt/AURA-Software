#include "INA219.h"

float ina219::ina219::_calibrate(void)
{
    // TODO figure out how to write 16 bits.
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

    printf("Data from register: %f\n", static_cast<float>(current));

    return 0.0f;
}

float ina219::ina219::get_power(void)
{
    uint16_t power;
    power = _master_bus->read_bytes_i2c<uint16_t, 2>(_ina_handle, INA_POWER);

    printf("Data from register: %d\n", power);

    return 0.0f;
}
