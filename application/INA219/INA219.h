#pragma once

#include "I2C.h"

#define INA_ADDR 0x41

#define INA_CONFIG 0x00
#define INA_SHUNT_V 0x01
#define INA_BUS_V 0x02
#define INA_POWER 0x03
#define INA_CURRENT 0x04
#define INA_CALIB 0x05

#define I2C_SPEED 100000

#define CALIB_VALUE 0xD1B8
#define LSB_BUS_V 2048.0f
#define LSB_CURRENT
#define LSB_POWER

namespace ina219
{    
    class ina219
    {
    private:
        i2c::i2c_bus* _master_bus;

        i2c_master_dev_handle_t _ina_handle;
        i2c_device_config_t _ina_config = {};

        void _calibrate(void);

    public:
        ina219(i2c::i2c_bus* master_bus)
            : _master_bus(master_bus)
        {
            _ina_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            _ina_config.scl_speed_hz = I2C_SPEED;
            _ina_config.device_address = INA_ADDR;

            ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(
                &_ina_config,
                &_ina_handle
            ));
        }

        float get_power(void);
        float get_voltage(void);
        float get_current(void);
    };
};