#include <math.h>
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
Top_Env_TypeDef *env;
Top_SwCtrl_Typedef *swCtrl;
Top_Monitoring_TypeDef *mon;
Top_DutI2cOper_TypeDef *dutI2c;

/*
 * @brief       Storage the realtime measurement values such as VCC, ICC.
 */
extern uint8_t  usCoilBuf[100];
extern uint16_t usRegInputBuf[100];
extern uint16_t usRegHoldingBuf[100];

static float vbus = 0, vshunt = 0, curr = 0, power = 0;
static void udpate_ina226(int ch)
{
  /* The definition of CH:
   * INA226 1: VCC1----------
   * INA226 2: VCC2
   * INA226 3: VCC3
   * INA226 4: 24V
   * INA226 5: TEC
   */

  vbus = INA226_ReadBusVoltageReg(&ina226[ch]);
  vshunt = INA226_ReadShuntVoltageReg(&ina226[ch]);
  curr = INA226_ReadCurrentReg(&ina226[ch]);
  power = INA226_ReadPowerReg(&ina226[ch]);

  mon->INA226Mon[ch].Vcc = vbus;
  mon->INA226Mon[ch].Vshunt = vshunt;
  mon->INA226Mon[ch].Current = curr;
  mon->INA226Mon[ch].Power = power;

}

static void init_monitoring_buff(void)
{
  mon->Tec.TargetTemp = NAN;
  mon->Tec.PidInc = NAN;
  mon->Tec.PidCtlLevel = NAN;
}

static void env_validate(Top_Env_TypeDef* env)
{
  if(isnan(env->TECConf.P))
      env->TECConf.P = 100;

  if(isnan(env->TECConf.I))
      env->TECConf.I = 20;

  if(isnan(env->TECConf.D))
      env->TECConf.D = 100;

  if(isnan(env->TECConf.SamplingIntervalMs))
      env->TECConf.SamplingIntervalMs = 50;

  if(isnan(env->TECConf.NTCCoeffA))
      env->TECConf.NTCCoeffA = 0.00112516f;

  if(isnan(env->TECConf.NTCCoeffB))
      env->TECConf.NTCCoeffB = 0.000234721f;

  if(isnan(env->TECConf.NTCCoeffC))
      env->TECConf.NTCCoeffC = 8.5877e-08;

  if(isnan(env->TECConf.TempProteLow))
    env->TECConf.TempProteLow = -10;

  if(isnan(env->TECConf.TempProteHigh))
      env->TECConf.TempProteHigh = 80;

  env->TECConf.ManualControl = 0;
}

void Top_Init(void)
{
  // map to registers of the modbus.
  env = (Top_Env_TypeDef*)usRegHoldingBuf;
  swCtrl = (Top_SwCtrl_Typedef*)usCoilBuf;
  mon = (Top_Monitoring_TypeDef*)usRegInputBuf;
  dutI2c = (Top_DutI2cOper_TypeDef*)&usRegHoldingBuf[REG_HOLDING_POS_DUT_IIC_OPER];
  init_monitoring_buff();

  // load the env from the flash.
  Top_LoadEnvFromFlash();

  // validate the env and load the default values if the item is NAN.
  env_validate(env);

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

  Top_TurnOffTec();
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
  if(adn8835.IsAutoTuningStarted == 0)
    Top_ReadRealtimeTemp();

  // read VTEC/ITEC
  mon->Tec.Vtec = ADN8835_ReadVTEC(&adn8835, env->TECConf.ADCVref) / 1000; // convert mV to V
  mon->Tec.Itec = ADN8835_ReadITEC(&adn8835, env->TECConf.ADCVref) / 1000; // convert mA to A
}

float Top_ReadRealtimeTemp(void)
{
  float temp = ADN8835_ReadTemp(&adn8835, env->TECConf.ADCVref, env->TECConf.NTCVref, env->TECConf.NTCCoeffA, env->TECConf.NTCCoeffB, env->TECConf.NTCCoeffC);
  mon->Tec.Vntc = adn8835.NtcMonitoring.Volt;
  mon->Tec.Intc = adn8835.NtcMonitoring.Curr;
  mon->Tec.Rntc = adn8835.NtcMonitoring.Ohm;
  mon->Tec.RtTemp = temp;
  return temp;
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
  swCtrl->SwVcc1 = 1;
}


void Top_TurnOffVcc1(void)
{
  HAL_GPIO_WritePin(SW_VCC1_GPIO_Port, SW_VCC1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC1_STA_GPIO_Port, VCC1_STA_Pin, GPIO_PIN_SET);
  swCtrl->SwVcc1 = 0;
}

void Top_TurnOnVcc2(void)
{
  HAL_GPIO_WritePin(SW_VCC2_GPIO_Port, SW_VCC2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VCC2_STA_GPIO_Port, VCC2_STA_Pin, GPIO_PIN_RESET);
  swCtrl->SwVcc2 = 1;
}


void Top_TurnOffVcc2(void)
{
  HAL_GPIO_WritePin(SW_VCC2_GPIO_Port, SW_VCC2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC2_STA_GPIO_Port, VCC2_STA_Pin, GPIO_PIN_SET);
  swCtrl->SwVcc2 = 0;
}

void Top_TurnOnVcc3(void)
{
  HAL_GPIO_WritePin(SW_VCC3_GPIO_Port, SW_VCC3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VCC3_STA_GPIO_Port, VCC3_STA_Pin, GPIO_PIN_RESET);
  swCtrl->SwVcc3 = 1;
}

void Top_TurnOffVcc3(void)
{
  HAL_GPIO_WritePin(SW_VCC3_GPIO_Port, SW_VCC3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VCC3_STA_GPIO_Port, VCC3_STA_Pin, GPIO_PIN_SET);
  swCtrl->SwVcc3 = 0;
}

void Top_TurnOnTec(void)
{
  ADN8835_Enable(&adn8835);
  swCtrl->TecEn = 1;
}

void Top_TurnOffTec(void)
{
  ADN8835_SetControlLevel(&adn8835, 0, env->TECConf.ADCVref);
  ADN8835_Disable(&adn8835);

  // clear the values of the monitoring.
  init_monitoring_buff();

  swCtrl->TecEn = 0;
}

void Top_DutTxDisable(void)
{
  HAL_GPIO_WritePin(DUT_nTXDIS_GPIO_Port, DUT_nTXDIS_Pin, GPIO_PIN_RESET);
  swCtrl->DutTxDis = 0;
}

void Top_DutTxEnable(void)
{
  HAL_GPIO_WritePin(DUT_nTXDIS_GPIO_Port, DUT_nTXDIS_Pin, GPIO_PIN_SET);
  swCtrl->DutTxDis = 1;
}

void Top_DutReset(void)
{
  HAL_GPIO_WritePin(DUT_nRST_GPIO_Port, DUT_nRST_Pin, GPIO_PIN_RESET);
  swCtrl->DutRst = 0;
}

void Top_DutUnreset(void)
{
  HAL_GPIO_WritePin(DUT_nRST_GPIO_Port, DUT_nRST_Pin, GPIO_PIN_SET);
  swCtrl->DutRst = 1;
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

void Top_SetTecControlLevel(float level)
{

  float ctrlLev = 0.0f;

  /* NOTE
   * For the Jupiter TOSA, >1250mV makes it cooller, <1250mV makes it hotter, so the inc should be plused to the controller level.
   * If you want the controller direction reversed, simply change the plus to minus.
   */
  // invert the TEC+/- polarity according to the configuration.
  int invertPolarity = 1;
  if(env->TECConf.Polarity == 1)
   invertPolarity = -1;

  ctrlLev = level;

  switch(env->TECConf.Mode)
  {
    case ADN8835_HEATER_MODE:
      if(ctrlLev < 0)
        ctrlLev = 0;
      else if(ctrlLev > 1250.0f)
        ctrlLev = 1250.0f;
      break;

    case ADN8835_TEC_MODE:
      if(ctrlLev > 1250.0f)
        ctrlLev = 1250.0f;
      else if(ctrlLev < -1250.0f)
        ctrlLev = -1250.0f;
      break;

    default:
      ctrlLev = 0.0f;
      break;
  }

  adn8835.PIDParam.LastTempCtrlLevel = ctrlLev;
  ctrlLev *= invertPolarity;
  mon->Tec.PidCtlLevel = ctrlLev; // the TEC control level in the monitor contains the polarity flag.
  ADN8835_SetControlLevel(&adn8835, ctrlLev, env->TECConf.ADCVref);
}

void Top_TecSetDacVolt(float volt)
{
  // THE METHOD IS ONLY AVAILABLE WHEN THE ManualControl bit IS SET TO TRUE.
  if(env->TECConf.ManualControl)
  {
    // restrict the ADN8835 TEC_SET to 0mV ~ 2500mV.
    if(isnan(volt) || volt < 0)
      volt = 0;
    else if(volt > 2500)
      volt = 2500;

    float ctrlLev = volt - 1250;

    Top_SetTecControlLevel(ctrlLev);
  }
}

void Top_TecTune(void)
{
    float rtTemp = Top_ReadRealtimeTemp();

    // present the TEC configuration flags.
    mon->Tec.TecMode = env->TECConf.Mode;
    mon->Tec.TecPolarity = env->TECConf.Polarity;

    if(rtTemp < env->TECConf.TempProteLow)  // Rt. is too low, turn off the controller.
    {
      Top_TurnOffTec();
      Top_SetErrorCode(ERR_PID_RTTEMP_TOO_LOW);
    }
    else if(rtTemp > env->TECConf.TempProteHigh)  // Rt. is too high, turn off the controller.
    {
      Top_TurnOffTec();
      Top_SetErrorCode(ERR_PID_RTTEMP_TOO_HIGH);
    }
    else
    {
      if(env->TECConf.ManualControl)
      {
        /**
         * Manually control the TEC
         **/

        Top_TecSetDacVolt(env->TECConf.DacOutputMv);
      }
      else
      {
        /**
         * Automatically tuning the TEC
         **/

        if(isnan(rtTemp))
        {
          // if we don't get the correct temperature, stop tuning.
          Top_TurnOffTec();
          Top_SetErrorCode(ERR_PID_INVALID_RTTEMP);
        }
        else if(isnan(env->TECConf.TargetTemp))
        {
          Top_TurnOffTec();
          Top_SetErrorCode(ERR_PID_INVALID_TARGET_TEMP);
        }
        else
        {
          // start to control the temperature.
          float inc = ADN8835_PidTune(&adn8835, rtTemp, env->TECConf.TargetTemp, env->TECConf.P, env->TECConf.I, env->TECConf.D);
          float ctrlLevel = adn8835.PIDParam.LastTempCtrlLevel;
          ctrlLevel += inc;

          Top_SetTecControlLevel(ctrlLevel);

          mon->Tec.PidInc = inc;
          mon->Tec.TargetTemp = env->TECConf.TargetTemp;

        }
      }
    }
}


void Top_SetErrorCode(int16_t errCode)
{
  mon->ErrorCode = errCode;
  //usRegHoldingBuf[REG_HOLDING_POS_ERR_CODE] = (uint16_t)errCode;
}

