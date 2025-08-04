/** 
 *  
 */
#include "I2C.h"

esp_err_t i2c::i2c_bus::read_byte_arr_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            uint8_t *data,
            size_t len)
{
    return i2c_master_transmit_receive(dev_handle,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t i2c::i2c_bus::write_byte_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(
        dev_handle,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t i2c::i2c_bus::master_probe(uint8_t dev_addr)
{
    return i2c_master_probe(_bus_handle, dev_addr, -1);
}
