#ifndef _VL53L0X_STM32_H_
#define _VL53L0X_STM32_H_

#include "stm32h7xx_hal.h"

#include "vl53l0x_base.h"


class VL53L0x : public VL53L0xBase
{
public :
    VL53L0x();

    void set_i2c_interface(I2C_HandleTypeDef *i2c_hdl);

    /*virtual*/bool i2c_write_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout=1000/*msec*/);
    /*virtual*/bool i2c_read_register(uint16_t i2c_address, uint8_t reg_index, uint8_t *pdata, uint16_t size, uint32_t timeout=1000/*msec*/);
    /*virtual*/void delay_ms(int msec);

protected :
    I2C_HandleTypeDef *m_i2c_hdl;
};

#endif // _VL53L0X_STM32_H_
