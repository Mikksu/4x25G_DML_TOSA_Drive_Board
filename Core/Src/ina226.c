#include "ina226.h"

static void i2c_write(INA226_TypeDef* ina226, uint8_t reg, uint16_t data);


void INA226_Init(INA226_TypeDef* ina226, I2C_HandleTypeDef* hi2c, uint8_t address)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;

  // try to read the manufacturer ID and the Die ID to make sure the chip works is connected.
  // manufacture id
  i2cStatus = HAL_I2C_Mem_Read(&hi2c1, address, INA266_REG_MANUF_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t*)ina226->ManufID, 2, 100);
  if(i2cStatus != HAL_OK) return;

  // Die id
  i2cStatus = HAL_I2C_Mem_Read(&hi2c1, address, INA266_REG_DIE_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t*)ina226->DieID, 2, 100);
  if(i2cStatus != HAL_OK) return;

  /*
   * uint8_t reg = INA266_REG_MANUF_ID;
  i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*)&reg, 1, HAL_MAX_DELAY);

  i2cStatus = HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t*)&ina226->ChipID, 4, HAL_MAX_DELAY);
  if(i2cStatus != HAL_OK) return;
  */

  // check the id of the chip
  if(ina226->ChipID != 0x60224954) return;

  ina226->I2CPort = hi2c;
  ina226->I2CAddress = address;
}

void INA226_SoftwareReset(INA226_TypeDef* ina226)
{
  i2c_write(ina226, INA266_REG_CONFIG, 0x8000);
}


static void i2c_write(INA226_TypeDef* ina226, uint8_t reg, uint16_t data)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint8_t tmpBuf[2];
  tmpBuf[0] = (data & 0xFF00) >> 8;
  tmpBuf[1] = data & 0x00FF;

  if(ina226->I2CPort == NULL) return; // the INA226 is not initialized.

  i2cStatus = HAL_I2C_Mem_Write(ina226->I2CPort, ina226->I2CAddress, INA266_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, tmpBuf, 2, 100);
  i2cStatus = i2cStatus;
}
