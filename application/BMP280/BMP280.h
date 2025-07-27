/**
 * 
 * BMP280 I2C library.
 * 
 * Datasheet for reference:
 * https://www.mouser.co.uk/datasheet/2/783/BST_BMP280_DS001-1509562.pdf
 * 
 */
#pragma once

#include "I2C.h"

#define BMP_ADDR 0x76

#define BMP_DIG_T1 0x89
#define BMP_DIG_T2 0x8B
#define BMP_DIG_T3 0x8D

#define BMP_DIG_P1 0x8F
#define BMP_DIG_P2 0x91
#define BMP_DIG_P3 0x93
#define BMP_DIG_P4 0x95
#define BMP_DIG_P5 0x97
#define BMP_DIG_P6 0x99
#define BMP_DIG_P7 0x9B
#define BMP_DIG_P8 0x9D
#define BMP_DIG_P9 0x9F

#define BMP_ID 0xD0 // Chip ID is 0x58 and can be read after pwr on
#define BMP_RESET 0xE0 // Write 0xB6 to reset device fully
#define BMP_STATUS 0xF3 // Bit 3 means measuring, Bit 0 means updated 
#define BMP_CTRL_MEAS 0xF4 // [7:5] temp [4:2] pressure [1:0] mode
#define BMP_CONFIG 0xF5 // [7:5] standby [4:2] filter [0] enable SPI

#define BMP_PRESS_DATA 0xF7 // Only 20 bits long i.e. 0xF9 [7:4]
#define BMP_TEMP_DATA 0xFA // Only 20 bits long i.e. 0xFC [7:4]

#define TEMP_OVERSAMPLE 0x01
#define PRESS_OVERSAMPLE 0x01
#define NORMAL_MODE 0x03

#define I2C_SPEED 100000

#define TEMP_AND_PRESS_MASK 0xFFFFF

namespace bmp280
{
    class bmp280
    {
    private:
        i2c::i2c_bus* _master_bus;

        i2c_master_dev_handle_t _bmp_handle;
        i2c_device_config_t _bmp_config = {};

        const uint8_t _bmp_addr;

        int32_t _comp_temp, _raw_temp, _raw_pressure;
        uint16_t _dig_t1, _dig_p1;
        int16_t _dig_t2, _dig_t3, _dig_p2, _dig_p3, _dig_p4, _dig_p5, _dig_p6, _dig_p7, _dig_p8, _dig_p9;

        float _initial_pressure=0;

        void _calibrate(void)
        {
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

        void _get_compensated_temp(void)
        {
            uint32_t var1, var2;
            var1 = ((((_raw_temp >> 3) - ((int32_t)_dig_t1 << 1))) * ((int32_t)_dig_t2)) >> 11;
            var2 = (((((_raw_temp >> 4) - ((int32_t)_dig_t1)) * ((_raw_temp >> 4) - ((int32_t)_dig_t1))) >> 12) * ((int32_t)_dig_t3)) >> 14;
            _comp_temp = var1 + var2;
        }

    public:
        bmp280(i2c::i2c_bus* master_bus, uint8_t addr=BMP_ADDR)
            : _master_bus(master_bus), _bmp_addr(addr)
        {
            _bmp_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            _bmp_config.scl_speed_hz = I2C_SPEED;
            _bmp_config.device_address = _bmp_addr;

            ESP_ERROR_CHECK(_master_bus->add_slave_to_bus(
                &_bmp_config,
                &_bmp_handle
            ));

            uint8_t ctrl_meas_init = ((uint8_t) TEMP_OVERSAMPLE << 5);
            ctrl_meas_init |= ((uint8_t) PRESS_OVERSAMPLE << 2);
            ctrl_meas_init |= ((uint8_t) NORMAL_MODE);

            ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_bmp_handle,
                BMP_CTRL_MEAS,
                ctrl_meas_init
            ));

            _calibrate();
        }

        void set_height(float acc_height);
        void update(void);
        float get_pressure(void);
        float get_temperature(void);
        float get_alt(void);
    };
};