#include "top.h"
#include "gpio.h"
#include "tim.h"
#include "ina226.h"

#include "mb.h"
#include "mbutils.h"


/*
 * @brief         The instance of the INA226.
 *
 * INA226[0]: VCC1
 * INA226[1]: VCC2
 * INA226[2]: VCC3
 * INA226[3]: 24V
 * INA226[4]: TEC
 */
INA226_HandleTypeDef ina226[MAX_INA226_CH];
ADN8835_TypeDef adn8835;
PID_TypeDef pid;
Top_Env_TypeDef *env;

/*
 * @brief       Storage the realtime measurement values such as VCC, ICC.
 */
extern uint16_t usRegInputBuf[100];
extern uint16_t usRegHoldingBuf[100];


static void udpate_ina226(int ch);

void Top_Init(void)
{
  env = (Top_Env_TypeDef*)usRegHoldingBuf;
  Top_LoadEnvFromFlash();

  INA226_HandleTypeDef* pIna226;

  // create the instance of the INA226.
  pIna226 = &ina226[0];
  INA226_Init(pIna226, &hi2c1, INA226_VCC1);
  INA226_SoftwareReset(pIna226);
  INA226_SetVBUSCT(pIna226, INA226_VBUSCT_1100);
  INA226_SetVSHCT(pIna226, INA226_VSHCT_1100);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm
  INA226_SyncRegistors(pIna226);


  pIna226 = &ina226[1];
  INA226_Init(pIna226, &hi2c1, INA226_VCC2);
  INA226_SoftwareReset(pIna226);
  INA226_SetVBUSCT(pIna226, INA226_VBUSCT_1100);
  INA226_SetVSHCT(pIna226, INA226_VSHCT_1100);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm
  INA226_SyncRegistors(pIna226);

  pIna226 = &ina226[2];
  INA226_Init(pIna226, &hi2c1, INA226_VCC3);
  INA226_SoftwareReset(pIna226);
  INA226_SetVBUSCT(pIna226, INA226_VBUSCT_1100);
  INA226_SetVSHCT(pIna226, INA226_VSHCT_1100);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm
  INA226_SyncRegistors(pIna226);

  pIna226 = &ina226[3];
  INA226_Init(pIna226, &hi2c2, INA226_24V);
  INA226_SoftwareReset(pIna226);
  INA226_SetVBUSCT(pIna226, INA226_VBUSCT_1100);
  INA226_SetVSHCT(pIna226, INA226_VSHCT_1100);
  INA226_SetCalibrationRegister(pIna226, 1.0f, 0.03f); // max current: 1A, shunt res: 0.03ohm
  INA226_SyncRegistors(pIna226);

  pIna226 = &ina226[4];
  INA226_Init(pIna226, &hi2c2, INA226_TEC);
  INA226_SoftwareReset(pIna226);
  INA226_SetVBUSCT(pIna226, INA226_VBUSCT_1100);
  INA226_SetVSHCT(pIna226, INA226_VSHCT_1100);
  INA226_SetCalibrationRegister(pIna226, 1.5f, 0.03f); // max current: 1.5A, shunt res: 0.03ohm
  INA226_SyncRegistors(pIna226);

  ADN8835_Init(&adn8835);
}

void Top_LoadEnvFromFlash(void)
{
  // copy the env from the flash to the memory.
  memcpy((uint8_t*)env, (uint8_t*)MEM_BASE_ENV, sizeof(Top_Env_TypeDef));
}

void Top_SaveEnvToFlash(void)
{
  int len = sizeof(Top_Env_TypeDef);
  uint8_t* pdata;
  uint32_t pmem;

  uint32_t sectorErr;

  FLASH_EraseInitTypeDef eraseinit;
  eraseinit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseinit.Banks = FLASH_BANK_1;
  eraseinit.Sector = FLASH_SECTOR_11;
  eraseinit.NbSectors = 1;
  eraseinit.VoltageRange = FLASH_VOLTAGE_RANGE_3;


  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&eraseinit, &sectorErr) == HAL_OK)
  {
    pdata = (uint8_t*)env;
    pmem = MEM_BASE_ENV;

    for(int i = 0; i < len; i++)
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, pmem++, *pdata++);
  }

  HAL_FLASH_Lock();
}

void Top_UpdateStatus(void)
{
  // read all INA226s
  for(int i = 0; i < MAX_INA226_CH; i++)
    udpate_ina226(i);

  // read real-time temperature
  float ntc = ADN8835_ReadTemp(&adn8835);
  xMBUtilFloatToWord(ntc, &usRegInputBuf[44]);
}

static void udpate_ina226(int ch)
{
  /* The definition of CH:
   * INA226 1: VCC1
   * INA226 2: VCC2
   * INA226 3: VCC3
   * INA226 4: 24V
   * INA226 5: TEC
   */

  float vbus = 0, power = 0, shunt = 0, curr = 0;

  vbus = INA226_ReadBusVoltageReg(&ina226[ch]);
  shunt = INA226_ReadShuntVoltageReg(&ina226[ch]);
  curr = INA226_ReadCurrentReg(&ina226[ch]);
  power = INA226_ReadPowerReg(&ina226[ch]);

  xMBUtilFloatToWord(vbus, &usRegInputBuf[ch * 8 + 0]);
  xMBUtilFloatToWord(shunt, &usRegInputBuf[ch * 8 + 2]);
  xMBUtilFloatToWord(curr, &usRegInputBuf[ch * 8 + 4]);
  xMBUtilFloatToWord(power, &usRegInputBuf[ch * 8 + 6]);

  /*
  uint16_t* p = (uint16_t*)&vbus;
  usRegInputBuf[ch * 8 + 0] = *(p+1);
  usRegInputBuf[ch * 8 + 1] = *p;

  p = (uint16_t*)&shunt;
  usRegInputBuf[ch * 8 + 2] = *(p+1);
  usRegInputBuf[ch * 8 + 3] = *p;

  p = (uint16_t*)&curr;
  usRegInputBuf[ch * 8 + 4] = *(p+1);
  usRegInputBuf[ch * 8 + 5] = *p;

  p = (uint16_t*)&power;
  usRegInputBuf[ch * 8 + 6] = *(p+1);
  usRegInputBuf[ch * 8 + 7] = *p;
  */
}

void Top_TurnOnLed(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Top_TurnOffLed(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void Top_ChangeLedBrightness(int brightness)
{
  htim1.Instance->CCR1 = brightness;
}


void Top_TurnOnVcc1(void)
{
  HAL_GPIO_WritePin(SW_VCC1_GPIO_Port, SW_VCC1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VCC1_STA_GPIO_Port, VCC1_STA_Pin, GPIO_PIN_RESET);
}


void Top_TurnOffVcc1(void)
{
  HAL_GPIO_WritePin(SW_VCC1_GPIO_Port, SW_VCC1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC1_STA_GPIO_Port, VCC1_STA_Pin, GPIO_PIN_SET);
}

void Top_TurnOnVcc2(void)
{
  HAL_GPIO_WritePin(SW_VCC2_GPIO_Port, SW_VCC2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VCC2_STA_GPIO_Port, VCC2_STA_Pin, GPIO_PIN_RESET);
}


void Top_TurnOffVcc2(void)
{
  HAL_GPIO_WritePin(SW_VCC2_GPIO_Port, SW_VCC2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC2_STA_GPIO_Port, VCC2_STA_Pin, GPIO_PIN_SET);
}

void Top_TurnOnVcc3(void)
{
  HAL_GPIO_WritePin(SW_VCC3_GPIO_Port, SW_VCC3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VCC3_STA_GPIO_Port, VCC3_STA_Pin, GPIO_PIN_RESET);
}

void Top_TurnOffVcc3(void)
{
  HAL_GPIO_WritePin(SW_VCC3_GPIO_Port, SW_VCC3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC3_STA_GPIO_Port, VCC3_STA_Pin, GPIO_PIN_SET);
}

void Top_TurnOnTec(void)
{
  ADN8835_Enable(&adn8835);
}

void Top_TurnOffTec(void)
{
  ADN8835_Disable(&adn8835);
}

void Top_SetTecNtcCoeffA(float coeff)
{
  adn8835.NTCCoeff.CoA = coeff;
  //env->TECConf.NTCCoeffA = coeff;
}

void Top_SetTecNtcCoeffB(float coeff)
{
  adn8835.NTCCoeff.CoB = coeff;
  //env->TECConf.NTCCoeffB = coeff;
}

void Top_SetTecNtcCoeffC(float coeff)
{
  adn8835.NTCCoeff.CoC = coeff;
  //env->TECConf.NTCCoeffC = coeff;
}

void Top_SetTecMode(TOP_TecMode_TypeDef mode)
{
  switch(mode)
  {
    case TEC_MODE_HEATER:
      ADN8835_SetMode(&adn8835, ADN8835_HEATER_MODE);
      //env->TECConf.Mode = (int)ADN8835_HEATER_MODE;
      break;

    case TEC_MODE_TEC:
      ADN8835_SetMode(&adn8835, ADN8835_TEC_MODE);
      //env->TECConf.Mode = (int)ADN8835_TEC_MODE;
      break;
  }
}

void Top_SetPidKp(float kp)
{
  PID_SetKp(&pid, kp);
  //env->TECConf.P = kp;
}

void Top_SetPidKi(float ki)
{
  PID_SetKi(&pid, ki);
  env->TECConf.I = ki;
}

void Top_SetPidKd(float kd)
{
  PID_SetKd(&pid, kd);
  env->TECConf.D = kd;
}

void Top_SetPidSamplingInterval(uint16_t ms)
{
  PID_SetSamplingInterval(&pid, ms);
  env->TECConf.SamplingIntervalMs = ms;
}

