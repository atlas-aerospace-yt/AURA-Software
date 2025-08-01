#include "ICM20948.h"

/*
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_PWR_MGMT_1, ON_MODE));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_PWR_MGMT_2, ON_MODE));

ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_REG_BANK_SEL, BANK_SEL_2));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_GYRO_SMPLRT_DIV, GYRO_SMPLRT_DIV));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_ACCEL_SMPLRT_DIV_1, ACCEL_SMPLRT_DIV));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_ACCEL_SMPLRT_DIV_2, ACCEL_SMPLRT_DIV));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_GYRO_CONFIG_1, GYRO_CONFIG_1));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_GYRO_CONFIG_1, GYRO_CONFIG_2));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_ACCEL_CONFIG_1, ACCEL_CONFIG_1));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_ACCEL_CONFIG_2, ACCEL_CONFIG_2));
ESP_ERROR_CHECK(my_bus.write_byte_i2c(my_icm._icm_handle, ICM_REG_BANK_SEL, BANK_SEL_0));

int16_t icm_test;
icm_test = my_bus.read_bytes_i2c<int16_t,2>(my_icm._icm_handle, ICM_ACC_XOUT);
printf("%f\n", (static_cast<float>(icm_test)) / 2048.0f * 9.81f);
vTaskDelay(pdMS_TO_TICKS(250));

*/

void icm20948::icm20948::_wake_device(void)
{
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_PWR_MGMT_1,
        ON_MODE
    ));
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_PWR_MGMT_2,
        ON_MODE
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

void icm20948::icm20948::_configure_mag(void)
{
    ESP_ERROR_CHECK(_master_bus->write_byte_i2c(
        _icm_handle,
        ICM_INT_PIN_CFG,
        INT_PIN_CFG
    ));
}

void icm20948::icm20948::update(void)
{
    uint64_t gyro, acc;
    float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;

    acc = _master_bus->read_bytes_i2c<uint64_t,6>(_icm_handle, ICM_ACC_XOUT);
    gyro = _master_bus->read_bytes_i2c<uint64_t,6>(_icm_handle, ICM_GYRO_XOUT);

    acc_x = static_cast<float>(static_cast<int16_t>((acc >> 32) & AXIS_MASK)) / 2048.0f * 9.81f;
    acc_y = static_cast<float>(static_cast<int16_t>((acc >> 16) & AXIS_MASK)) / 2048.0f * 9.81f;
    acc_z = static_cast<float>(static_cast<int16_t>((acc) & AXIS_MASK)) / 2048.0f * 9.81f;

    gyro_x = static_cast<float>(static_cast<int16_t>((gyro >> 32) & AXIS_MASK)) / 2048.0f * 125.0f;
    gyro_y = static_cast<float>(static_cast<int16_t>((gyro >> 16) & AXIS_MASK)) / 2048.0f * 125.0f;
    gyro_z = static_cast<float>(static_cast<int16_t>((gyro) & AXIS_MASK)) / 2048.0f * 125.0f;

    printf("%f\n %f\n %f\n %f\n %f\n %f\n\n\n", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
}
