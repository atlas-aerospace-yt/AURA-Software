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

#include "math.h"
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

#define MAG_ST1 0x10
#define MAG_ST2 0x18

#define MAG_CNTRL_2 0x31
#define MAG_CNTRL_3 0x32

#define MAG_HXL 0x11
#define MAG_HXH 0x12
#define MAG_HYL 0x13
#define MAG_HTH 0x14
#define MAG_HZL 0x15
#define MAG_HZH 0x16

/* Initial settings values */
#define PWR_MGMT_1 0x01
#define PWR_MGMT_2 0x00
#define LP_CONFIG 0x00
#define INT_PIN_CFG 0x52

#define GYRO_CONFIG_1 0x37
#define GYRO_CONFIG_2 0x02
#define GYRO_SMPLRT_DIV 0x00

#define ACCEL_CONFIG_1 0x37
#define ACCEL_CONFIG_2 0x02
#define ACCEL_SMPLRT_DIV 0x00

#define MAG_SLEEP 0x0
#define MAG_CONT_MODE_4 0x08

#define AXIS_MASK 0xffff

/* Conversion definitions */
#define CONVERT_ACCEL 208.767f
#define CONVERT_GYRO 938.74f
#define CONVERT_MAG 0.15f

#define MAG_Z_OFFS 45.0f

namespace icm20948
{
    class icm20948
    {
    private:
        i2c::i2c_bus* _master_bus;

        // ICM20948 I2C handle
        i2c_master_dev_handle_t _icm_handle;
        i2c_device_config_t _icm_config = {};

        // Internal AK09916 I2C handle
        i2c_master_dev_handle_t _mag_handle;
        i2c_device_config_t _mag_config = {};

        void _wake_device(void);
        void _select_register(uint8_t reg);
        void _configure_gyro(void);
        void _configure_accel(void);
        void _set_i2c_bypass(void);
        void _configure_mag(void);

    public:
        float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
        float mag_x, mag_y, mag_z;

        icm20948(i2c::i2c_bus* master_bus)
            : _master_bus(master_bus)
        {
            _icm_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            _icm_config.scl_speed_hz = I2C_SPEED;
            _icm_config.device_address = ICM_ADDR;

            ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(
                &_icm_config,
                &_icm_handle
            ));

            _set_i2c_bypass();

            _mag_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            _mag_config.scl_speed_hz = I2C_SPEED;
            _mag_config.device_address = MAG_ADDR;

            ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(
                &_mag_config,
                &_mag_handle
            ));

            _wake_device();
            _configure_gyro();
            _configure_accel();
            _configure_mag();
        }

        void update(void);
        void update_mag(void);

    };
};
