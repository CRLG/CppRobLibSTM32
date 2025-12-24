#include "vl53l0x.h"
#include "main.h"
// _____________________________________________
VL53L0x::VL53L0x()
	: m_i2c_hdl(nullptr)
{
}

// _____________________________________________
void VL53L0x::set_i2c_interface(I2C_HandleTypeDef *i2c_hdl)
{
	m_i2c_hdl = i2c_hdl;
}

// _____________________________________________
bool VL53L0x::i2c_write_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout)
{
	if (!m_i2c_hdl) return false;

	bool status;
    if (HAL_I2C_Mem_Write(m_i2c_hdl, i2c_address, reg_index, 1, pdata, size, timeout) != HAL_OK) status = false;
    else status = true;
    return status;
}

// _____________________________________________
bool VL53L0x::i2c_read_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout)
{
	if (!m_i2c_hdl) return false;

    bool status;
    if (HAL_I2C_Mem_Read(m_i2c_hdl, i2c_address, reg_index, 1, pdata, size, timeout) != HAL_OK) status = false;
    else status = true;
    return status;
}

// _____________________________________________
void VL53L0x::delay_ms(int msec)
{
    HAL_Delay(msec);
}



