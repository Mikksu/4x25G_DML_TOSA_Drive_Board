#ifndef __INA226_H
#define __INA226_H

#include "stdint.h"
#include "i2c.h"

#define INA226_VCC1		0x40 << 1
#define INA226_VCC2		0x41 << 1
#define INA226_VCC3		0x44 << 1

#define INA266_REG_CONFIG         0x00
#define INA266_REG_MANUF_ID       0xFE
#define INA266_REG_DIE_ID         0xFF


typedef struct 
{
	I2C_HandleTypeDef*		I2CPort;
	uint8_t					      I2CAddress;
	union
	{
	  uint32_t            ChipID;
	  struct
	  {
	    uint8_t           ManufID[2];
	    uint8_t           DieID[2];
	  };
	};
	

} INA226_TypeDef;


/*
 * Initialize the INA226
 *
 * @param ina226 			The point to the instance of the INA226.
 * @param hi2c				The point to i2c handler.
 * @param address			The slave address of the i2c of the INA226.
 * 
 */
void INA226_Init(INA226_TypeDef* ina226, I2C_HandleTypeDef* hi2c, uint8_t address);


/*
 * Reset the INA226 by the software.
 *
 * @param ina226      The point to the instance of the INA226.
 */
void INA226_SoftwareReset(INA226_TypeDef* ina226);



#endif
