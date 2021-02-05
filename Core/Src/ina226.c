#include "math.h"
#include "ina226.h"
#include "stm32f4xx_hal_def.h"

static HAL_StatusTypeDef i2c_write(INA226_HandleTypeDef* ina226, uint8_t reg, uint16_t data);
static HAL_StatusTypeDef i2c_read(INA226_HandleTypeDef* ina226, uint8_t reg, uint16_t* data);

HAL_StatusTypeDef INA226_Init(INA226_HandleTypeDef* ina226, I2C_HandleTypeDef* hi2c, uint8_t address)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;

  // try to read the manufacturer ID and the Die ID to make sure the chip works is connected.
  // manufacture id
  i2cStatus = HAL_I2C_Mem_Read(hi2c, address, INA226_REG_MANUF_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t*)ina226->ManufID, 2, 100);
  if(i2cStatus != HAL_OK) return i2cStatus;

  // Die id
  i2cStatus = HAL_I2C_Mem_Read(hi2c, address, INA226_REG_DIE_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t*)ina226->DieID, 2, 100);
  if(i2cStatus != HAL_OK) return i2cStatus;


  // check the id of the chip
  if(ina226->ChipID != 0x60224954) return HAL_ERROR;

  ina226->I2CPort = hi2c;
  ina226->I2CAddress = address;

  // read the current value of the registers.
  INA226_SyncRegistors(ina226);

  // register the callback functions.
  ina226->OnAvgModeChangedCb = &INA226_OnAveragingModeChanged;
  ina226->OnVBUSCTChangedCb = &OnVBUSCTChangedCb;
  ina226->OnVSHCTChangedCb = &OnVSHCTChangedCb;
  ina226->OnOperationModehangedCb = &OnOperationModehangedCb;

  return HAL_OK;
}

HAL_StatusTypeDef INA226_SoftwareReset(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  i2cStatus = i2c_write(ina226, INA226_REG_CONFIG, 0x8000);

  HAL_Delay(10);

  if(i2cStatus == HAL_OK)
  {
    // read the current value of the registers.
    INA226_SyncRegistors(ina226);
  }

  return i2cStatus;
}

void INA226_SyncRegistors(INA226_HandleTypeDef* ina226)
{
  ina226->RegConfiguration = INA226_ReadConfigurationReg(ina226);
  //ina226->RegShuntVoltage = INA226_ReadShuntVoltageReg(ina226);
  //ina226->RegBusVoltage = INA226_ReadBusVoltageReg(ina226);
  //ina226->RegPower = INA226_ReadPowerReg(ina226);
  //ina226->RegCurrent = INA226_ReadCurrentReg(ina226);
  ina226->RegCalibration = INA226_ReadCalibrationReg(ina226);
  ina226->RegMaskEnable = INA226_ReadMaskReg(ina226);
  ina226->RegAlertLimt = INA226_ReadAlertLimitReg(ina226);
}

uint16_t INA226_ReadConfigurationReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_CONFIG, &regval);
  i2cStatus = i2cStatus;
  return regval;
}

float INA226_ReadShuntVoltageReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_SHUNTVOLTAGE, &regval);
  if(i2cStatus == HAL_OK)
      return (int16_t)regval * 0.0000025;
    else
      return NAN;
}

float INA226_ReadBusVoltageReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_BUSVOLTAGE, &regval);

  if(i2cStatus == HAL_OK)
    return (float)regval * 0.00125;
  else
    return NAN;
}

float INA226_ReadPowerReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_POWER, &regval);
  if(i2cStatus == HAL_OK)
    return (int16_t)regval * ina226->CurrentLsbA * 25;
  else
    return NAN;
}

float INA226_ReadCurrentReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_CURRENT, &regval);
  if(i2cStatus == HAL_OK)
    return (int16_t)regval * ina226->CurrentLsbA;
  else
    return NAN;
}

uint16_t INA226_ReadCalibrationReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_CALIBRATION, &regval);
  i2cStatus = i2cStatus;
  return regval;
}

uint16_t INA226_ReadMaskReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_MASK, &regval);
  i2cStatus = i2cStatus;
  return regval;
}

uint16_t INA226_ReadAlertLimitReg(INA226_HandleTypeDef* ina226)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t  regval = 0xffff;

  i2cStatus = i2c_read(ina226, INA226_REG_ALERTLIMT, &regval);
  i2cStatus = i2cStatus;
  return regval;
}


HAL_StatusTypeDef INA226_SetAveragingMode(INA226_HandleTypeDef* ina226, INA226_AvgModeTypeDef mode)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t regval = 0xffff;

  regval = ina226->RegConfiguration;

  // clear the corresponding bits.
  regval &= (~INA226_CONFIG_AVG);
  regval |= (mode << INA226_CONFIG_AVG_POS);

  i2cStatus = i2c_write(ina226, INA226_REG_CONFIG, regval);

  if(i2cStatus == HAL_OK)
  {
    ina226->RegConfiguration = regval;
    if(ina226->OnAvgModeChangedCb != NULL)
      ina226->OnAvgModeChangedCb(ina226, mode);
  }

  return i2cStatus;
}

HAL_StatusTypeDef INA226_SetVBUSCT(INA226_HandleTypeDef* ina226, INA226_VBUSCTTypeDef mode)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t regval = 0xffff;

  regval = ina226->RegConfiguration;

  // clear the corresponding bits.
  regval &= (~INA226_CONFIG_VBUSCT);
  regval |= (mode << INA226_CONFIG_VBUSCT_POS);

  i2cStatus = i2c_write(ina226, INA226_REG_CONFIG, regval);

  if(i2cStatus == HAL_OK)
  {
    ina226->RegConfiguration = regval;
    if(ina226->OnVBUSCTChangedCb != NULL)
      ina226->OnVBUSCTChangedCb(ina226, mode);
  }

  return i2cStatus;
}

HAL_StatusTypeDef INA226_SetVSHCT(INA226_HandleTypeDef* ina226, INA226_VSHCTTypeDef mode)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t regval = 0xffff;

  regval = ina226->RegConfiguration;

  // clear the corresponding bits.
  regval &= (~INA226_CONFIG_VSHCT);
  regval |= (mode << INA226_CONFIG_VSHCT_POS);

  i2cStatus = i2c_write(ina226, INA226_REG_CONFIG, regval);

  if(i2cStatus == HAL_OK)
  {
    ina226->RegConfiguration = regval;
    if(ina226->OnVSHCTChangedCb != NULL)
      ina226->OnVSHCTChangedCb(ina226, mode);
  }

  return i2cStatus;
}

HAL_StatusTypeDef INA226_SetOperationMode(INA226_HandleTypeDef* ina226, INA226_OpModeTypeDef mode)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint16_t regval = 0xffff;

  regval = ina226->RegConfiguration;

  // clear the corresponding bits.
  regval &= (~INA226_CONFIG_MODE);
  regval |= mode;

  i2cStatus = i2c_write(ina226, INA226_REG_CONFIG, regval);

  if(i2cStatus == HAL_OK)
  {
    ina226->RegConfiguration = regval;
    if(ina226->OnOperationModehangedCb != NULL)
      ina226->OnOperationModehangedCb(ina226, mode);
  }

  return i2cStatus;
}


HAL_StatusTypeDef INA226_SetCalibrationRegister(INA226_HandleTypeDef* ina226, float maxExceptedCurrentA, float shuntResOhm)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;

  float currentLSB = maxExceptedCurrentA / 32768.0f;
  float cali = 0.00512 / (currentLSB * shuntResOhm);
  uint16_t caliInt = (uint16_t)cali;

  i2cStatus = i2c_write(ina226, INA226_REG_CALIBRATION, caliInt);
  if(i2cStatus == HAL_OK)
  {
    ina226->RegCalibration = caliInt;
    ina226->CalibrationParam.MaxExceptedCurrentA = maxExceptedCurrentA;
    ina226->CalibrationParam.ShuntResistorOhm = shuntResOhm;
    ina226->CurrentLsbA = maxExceptedCurrentA / 32768.0;
  }

  return i2cStatus;
}


static HAL_StatusTypeDef i2c_write(INA226_HandleTypeDef* ina226, uint8_t reg, uint16_t data)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint8_t tmpBuf[2] = {0xff, 0xff};
  tmpBuf[0] = (data & 0xFF00) >> 8;
  tmpBuf[1] = data & 0x00FF;

  if(ina226->I2CPort == NULL)
    return HAL_ERROR; // the INA226 is not initialized.

  i2cStatus = HAL_I2C_Mem_Write(ina226->I2CPort, ina226->I2CAddress, reg, I2C_MEMADD_SIZE_8BIT, tmpBuf, 2, 100);
  return i2cStatus;
}


static HAL_StatusTypeDef i2c_read(INA226_HandleTypeDef* ina226, uint8_t reg, uint16_t* data)
{
  HAL_StatusTypeDef i2cStatus = HAL_OK;
  uint8_t tmpBuf[2] = {0xff, 0xff};

  i2cStatus = HAL_I2C_Mem_Read(ina226->I2CPort, ina226->I2CAddress, reg, I2C_MEMADD_SIZE_8BIT, tmpBuf, 2, 100);
  if(i2cStatus == HAL_OK)
  {
    *data = 0x0;
    *data |= tmpBuf[0] << 8;
    *data |= tmpBuf[1];
  }

  return i2cStatus;
}

__weak void INA226_OnAveragingModeChanged(struct __INA226_HandleTypeDef* this, int e)
{
  UNUSED(this);
}

__weak void OnVBUSCTChangedCb(struct __INA226_HandleTypeDef* this, int e)
{
  UNUSED(this);
}

__weak void OnVSHCTChangedCb(struct __INA226_HandleTypeDef* this, int e)
{
  UNUSED(this);
}

__weak void OnOperationModehangedCb(struct __INA226_HandleTypeDef* this, int e)
{
  UNUSED(this);
}



