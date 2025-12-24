#ifndef _VL53L0X_MULTI_BASE_STM32_H_
#define _VL53L0X_MULTI_BASE_STM32_H_
#include "vl53l0x.h"

class VL53L0xMultiBase
{
public :
	VL53L0xMultiBase(I2C_HandleTypeDef *i2c_hdl);

    virtual uint8_t get_vl53_count() = 0;
    virtual VL53L0x *get_vl53(uint8_t index) = 0;
    virtual void set_reset_pin(uint8_t vl53_index, bool pin_state) = 0;
    virtual uint8_t get_i2c_address(uint8_t vl53_index) = 0;

    virtual VL53L0X_Error init();
    virtual VL53L0X_Error read();
    virtual uint16_t get_last_distance(uint8_t index);

protected :
    virtual VL53L0X_Error _init();

    I2C_HandleTypeDef *m_i2c_hdl;
};

#endif // _VL53L0X_MULTI_BASE_STM32_H_
