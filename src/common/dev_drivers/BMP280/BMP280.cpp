#include "BMP280.h"
#include "math.h"

void bmp280::bmp280::_set_mode(void)
{
    uint8_t ctrl_meas_init = 0;
    ctrl_meas_init |= ((uint8_t) TEMP_OVERSAMPLE << 5);
    ctrl_meas_init |= ((uint8_t) PRESS_OVERSAMPLE << 2);
    ctrl_meas_init |= ((uint8_t) NORMAL_MODE);

    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(_bmp_handle,
        BMP_CTRL_MEAS,
        ctrl_meas_init
    ));
}

void bmp280::bmp280::_calibrate(void)
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

void bmp280::bmp280::_get_compensated_temp(void)
{
    uint32_t var1, var2;
    var1 = ((((_raw_temp >> 3) - ((int32_t)_dig_t1 << 1))) * ((int32_t)_dig_t2)) >> 11;
    var2 = (((((_raw_temp >> 4) - ((int32_t)_dig_t1)) * ((_raw_temp >> 4) - (
        (int32_t)_dig_t1))) >> 12) * ((int32_t)_dig_t3)) >> 14;
    _comp_temp = var1 + var2;
}

void bmp280::bmp280::set_height(float acc_height)
{
    //TODO
}

void bmp280::bmp280::update(void)
{
    uint64_t temp_and_pressure;

    // Read temperature and pressure in one go.
    temp_and_pressure = _master_bus->read_bytes_i2c<uint64_t, 6>(
        _bmp_handle,
        BMP_PRESS_DATA
    );

    // Split binary into correct values.
    _raw_temp = (temp_and_pressure >> 4) & TEMP_AND_PRESS_MASK;
    _raw_pressure = (temp_and_pressure >> 28) & TEMP_AND_PRESS_MASK;

    // Calculate "t_fine" for compensation calculations.
    // Needed for both temperature and pressure readings.
    _get_compensated_temp();
}

float bmp280::bmp280::get_pressure(void)
{
    int64_t var1, var2, pressure;

    var1 = ((int64_t) _comp_temp) - 128000;
    var2 = var1 * var1 * (int64_t) _dig_p6;
    var2 = var2 + ((var1 * (int64_t) _dig_p5) << 17);
    var2 = var2 + (((int64_t) _dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t) _dig_p3) >> 8) + ((var1 * (int64_t) _dig_p2) << 12);
    var1 = (((((int64_t) 1 ) << 47) + var1)) * ((int64_t) _dig_p1) >> 33;

    if (var1 == 0)
    {
        return 0.0f;
    }

    pressure = 1048576 - _raw_pressure;
    pressure = (((pressure << 31) - var2)* 3125) / var1;
    var1 = (((int64_t) _dig_p9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (((int64_t) _dig_p8) * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + (((int64_t) _dig_p7) << 4);

    return static_cast<float>(pressure) / 256000.0f;
}

float bmp280::bmp280::get_temperature(void)
{
    return static_cast<float>((_comp_temp * 5 + 128) >> 8)/100.0f;
}

float bmp280::bmp280::get_altitude(void)
{
    float pressure = get_pressure();

    float curr_press_hpa = pressure / 10.0f;
    float init_press_hpa = _initial_pressure / 10.0f;

    return 44330 * (1.0 - pow(
        curr_press_hpa / init_press_hpa,
        0.1903
    ));
}
