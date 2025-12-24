#include <stdio.h>
#include "vl53l0xMultiBase.h"

// _____________________________________________
VL53L0xMultiBase::VL53L0xMultiBase(I2C_HandleTypeDef *i2c_hdl)
	: m_i2c_hdl(i2c_hdl)
{
}

// _____________________________________________
VL53L0X_Error VL53L0xMultiBase::init()
{
	VL53L0X_Error status;
	uint8_t attempt=0;

	do {
		status = _init();
	} while ((status != VL53L0X_ERROR_NONE) && (attempt < 3));

	return status;
}

// _____________________________________________
VL53L0X_Error VL53L0xMultiBase::_init()
{
	VL53L0X_Error status;
	int error_count = 0;

	for (int i=0; i<get_vl53_count(); i++) {
		get_vl53(i)->set_i2c_interface(m_i2c_hdl);
		set_reset_pin(i, false); // force tous les VL53 en reset
	}
	HAL_Delay(10);

	// initialise chaque VL53
	for (int i=0; i<get_vl53_count(); i++) {
		// Sort le VL53 de l'état de reset
		set_reset_pin(i, true);
		HAL_Delay(10);
		uint8_t i2c_addr = get_i2c_address(i);
		status = get_vl53(i)->init(i2c_addr);
	    if (status != VL53L0X_ERROR_NONE) {
	    	error_count++;
	    	printf("Error : Init VL53 #1\n\r");
	    }
	}

	return (error_count==0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_UNDEFINED;
}

// _____________________________________________
VL53L0X_Error VL53L0xMultiBase::read()
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int error_count = 0;
	uint16_t distance;

	for(int i=0;i<get_vl53_count(); i++) {
		if (get_vl53(i)->m_init_ok) {  // ne traite que les VL53 dont l'initialisation s'est bien passée
			Status = get_vl53(i)->read_distance(&distance);
			if (Status != VL53L0X_ERROR_NONE) {
				error_count++;
			}
		}
	}
	return (error_count==0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_UNDEFINED;
}

// _____________________________________________
/*! Récupère la dernière distance mesurée sur un VL53 de la liste
 * @param index : l'index du VL53 (0, 1, ...)
 * @return la dernière distance mesurée par le capteur
 * Valeur spécifique 0xFFFF en cas de mesure invalide
 * La fonction n'accède pas au capteur sur le bus mais récupère juste la dernière mesure mémorisée
 */
uint16_t VL53L0xMultiBase::get_last_distance(uint8_t index)
{
	if (index >= get_vl53_count()) return 0xFFFF;
	return get_vl53(index)->get_last_distance();
}
