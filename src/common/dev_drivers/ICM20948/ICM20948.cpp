#include "ICM20948.h"

void icm20948::icm20948::_wake_device(void)
{
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_PWR_MGMT_1,
        PWR_MGMT_1
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_PWR_MGMT_2,
        PWR_MGMT_2
    ));
}

void icm20948::icm20948::_select_register(uint8_t reg)
{
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_REG_BANK_SEL,
        reg
    ));
}

void icm20948::icm20948::_configure_gyro(void)
{
    _select_register(BANK_SEL_2);

    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_GYRO_SMPLRT_DIV,
        GYRO_SMPLRT_DIV
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_GYRO_CONFIG_1,
        GYRO_CONFIG_1
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_GYRO_CONFIG_2,
        GYRO_CONFIG_2
    ));

    _select_register(BANK_SEL_0);
}

void icm20948::icm20948::_configure_accel(void)
{
    _select_register(BANK_SEL_2);

    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_ACCEL_SMPLRT_DIV_1,
        ACCEL_SMPLRT_DIV
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_ACCEL_SMPLRT_DIV_2,
        ACCEL_SMPLRT_DIV
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_ACCEL_CONFIG_1,
        ACCEL_CONFIG_1
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_ACCEL_CONFIG_2,
        ACCEL_CONFIG_2
    ));

    _select_register(BANK_SEL_0);
}

void icm20948::icm20948::_set_i2c_bypass(void){
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_INT_PIN_CFG,
        INT_PIN_CFG
    ));
}

void icm20948::icm20948::_configure_mag(void)
{
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _mag_handle,
        MAG_CNTRL_2,
        MAG_SLEEP
    ));

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _mag_handle,
        MAG_CNTRL_2,
        MAG_CONT_MODE_4
    ));

    vTaskDelay(pdMS_TO_TICKS(10));
}

void icm20948::icm20948::update(void)
{
    uint64_t gyro, acc;

    acc = _master_bus->read_bytes_i2c<uint64_t,6>(_icm_handle, ICM_ACC_XOUT);
    gyro = _master_bus->read_bytes_i2c<uint64_t,6>(_icm_handle, ICM_GYRO_XOUT);

    acc_x = static_cast<float>(static_cast<int16_t>(
        (acc >> 32) & AXIS_MASK)) / CONVERT_ACCEL;
    acc_y = static_cast<float>(static_cast<int16_t>(
        (acc >> 16) & AXIS_MASK)) / CONVERT_ACCEL;
    acc_z = static_cast<float>(static_cast<int16_t>(
        (acc) & AXIS_MASK)) / CONVERT_ACCEL;

    gyro_x = static_cast<float>(static_cast<int16_t>(
        (gyro >> 32) & AXIS_MASK)) / CONVERT_GYRO;
    gyro_y = static_cast<float>(static_cast<int16_t>(
        (gyro >> 16) & AXIS_MASK)) / CONVERT_GYRO;
    gyro_z = static_cast<float>(static_cast<int16_t>(
        (gyro) & AXIS_MASK)) / CONVERT_GYRO;
}

/**
 * NOTE: mag must be read in the order shown - ST1, data, ST2
 */
void icm20948::icm20948::update_mag(void)
{
    uint64_t mag;

    _master_bus->read_bytes_i2c<uint8_t, 1>(_mag_handle, MAG_ST1);

    mag = _master_bus->read_bytes_i2c<uint64_t, 6>(_mag_handle, MAG_HXL, true);

    _master_bus->read_bytes_i2c<uint8_t, 1>(_mag_handle, MAG_ST2);

    mag_x = static_cast<float>(static_cast<int16_t>(
        (mag >> 32) & AXIS_MASK)) * CONVERT_MAG;
    mag_y = static_cast<float>(static_cast<int16_t>(
        (mag >> 16) & AXIS_MASK)) * CONVERT_MAG;
    mag_z = static_cast<float>(static_cast<int16_t>(
        (mag) & AXIS_MASK)) * CONVERT_MAG - MAG_Z_OFFS;
}
