/** 
 *  
 */
#include "I2C.h"

using namespace i2c;

esp_err_t i2c_bus::read_byte_i2c(
            i2c_master_dev_handle_t dev_handle,
            uint8_t reg_addr,
            uint8_t *data,
            size_t len)
{
    esp_err_t status = ESP_OK;
    return status;
}
