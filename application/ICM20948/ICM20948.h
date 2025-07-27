/**
 * 
 * 
 */
 #pragma once

#include "I2C.h"

#define ICM_ADDR 0x68

#define CHIPID 0x00
#define USER_CTRL 0x03
#define LP_CONFIG 0x05
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07

#define ICM_ACC_XOUT 0x2D
#define ICM_ACC_YOUT 0x2F
#define ICM_ACC_ZOUT 0x31

#define ICM_GYRO_XOUT 0x33
#define ICM_GYRO_YOUT 0x35
#define ICM_GYRO_ZOUT 0x37

#define ICM_TEMP_OUT 0x39

#define I2C_SPEED 100000

#define ON_MODE 0x00

namespace icm20948
{
    class icm20948
    {
    public:
    //private:
        i2c::i2c_bus* _master_bus;

        i2c_master_dev_handle_t _icm_handle;
        i2c_device_config_t _icm_config = {};

        const uint8_t _icm_addr;

    //public:
        icm20948(i2c::i2c_bus* master_bus, uint8_t addr=ICM_ADDR)
            : _master_bus(master_bus), _icm_addr(addr)
        {
            _icm_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            _icm_config.scl_speed_hz = I2C_SPEED;
            _icm_config.device_address = _icm_addr;

            ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(
                &_icm_config,
                &_icm_handle
            ));
        }
    };
};
