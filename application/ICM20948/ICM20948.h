/**
 * 
 * 
 */
 #pragma once

#include "I2C.h"

#define ICM_ADDR 0x68
#define I2C_SPEED 100000

#define CHIPID 0x00

/* Register bank selection*/
#define ICM_REG_BANK_SEL 0x7F
#define BANK_SEL_0 0x00
#define BANK_SEL_1 0x10
#define BANK_SEL_2 0x20
#define BANK_SEL_3 0x30

/* Setup register bank 0 addresses */
#define ICM_USER_CTRL 0x03
#define ICM_LP_CONFIG 0x05
#define ICM_PWR_MGMT_1 0x06
#define ICM_PWR_MGMT_2 0x07
#define ICM_INT_PIN_CFG 0x0F

/* Setup register bank 2 addresses */
#define ICM_GYRO_CONFIG_1 0x01
#define ICM_GYRO_CONFIG_2 0x02
#define ICM_GYRO_SMPLRT_DIV 0x00

#define ICM_ACCEL_CONFIG_1 0x14
#define ICM_ACCEL_CONFIG_2 0x15
#define ICM_ACCEL_SMPLRT_DIV_1 0x10
#define ICM_ACCEL_SMPLRT_DIV_2 0x11

/* Accel register addresses (BANK 0) */
#define ICM_ACC_XOUT 0x2D
#define ICM_ACC_YOUT 0x2F
#define ICM_ACC_ZOUT 0x31

/* GYRO register addresses (BANK 0) */
#define ICM_GYRO_XOUT 0x33
#define ICM_GYRO_YOUT 0x35
#define ICM_GYRO_ZOUT 0x37

#define ICM_TEMP_OUT 0x39

/* AK09916 Magnetometer definitions*/
#define MAG_ADDR 0x0C
#define MAG 1

/* Initial settings values */
#define ON_MODE 0x00
#define LP_CONFIG 0x00
#define INT_PIN_CFG 0x52

#define GYRO_CONFIG_1 0x37
#define GYRO_CONFIG_2 0x02
#define GYRO_SMPLRT_DIV 0x00

#define ACCEL_CONFIG_1 0x37
#define ACCEL_CONFIG_2 0x02
#define ACCEL_SMPLRT_DIV 0x00

#define AXIS_MASK 0xffff

namespace icm20948
{
    class icm20948
    {
    private:
        i2c::i2c_bus* _master_bus;

        i2c_master_dev_handle_t _icm_handle;
        i2c_device_config_t _icm_config = {};

        const uint8_t _icm_addr;

        void _wake_device(void);
        void _select_register(uint8_t reg);
        void _configure_gyro(void);
        void _configure_accel(void);
        void _configure_mag(void);

    public:
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

            _wake_device();
            _configure_gyro();
            _configure_accel();
            _configure_mag();
        }

        void update(void);

    };
};
