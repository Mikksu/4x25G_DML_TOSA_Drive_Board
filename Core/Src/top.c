#include "top.h"
#include "gpio.h"
#include "tim.h"
#include "ina226.h"

/*
 * @brief         How many INA226s are there on the board.
 */
#define MAX_INA226_CH  (5)

/*
 * @brief         The I2C address of the INA226s.
 */
#define INA226_VCC1   (0x40 << 1)
#define INA226_VCC2   (0x41 << 1)
#define INA226_VCC3   (0x44 << 1)
#define INA226_24V    (0x40 << 1)
#define INA226_TEC    (0x45 << 1)

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

/*
 * @brief       Storage the realtime measurement values such as VCC, ICC.
 */
extern uint16_t usRegInputBuf[100];


static void udpate_ina226(int ch);

void Top_Init(void)
{
  INA226_HandleTypeDef* pIna226;
  // create the instance of the INA226.

  pIna226 = &ina226[0];
  INA226_Init(pIna226, &hi2c1, INA226_VCC1);
  INA226_SoftwareReset(pIna226);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm

  pIna226 = &ina226[1];
  INA226_Init(pIna226, &hi2c1, INA226_VCC2);
  INA226_SoftwareReset(pIna226);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm

  pIna226 = &ina226[2];
  INA226_Init(pIna226, &hi2c1, INA226_VCC3);
  INA226_SoftwareReset(pIna226);
  INA226_SetCalibrationRegister(pIna226, 0.328f, 0.1f); // max current: 0.328A, shunt res: 0.1ohm

  pIna226 = &ina226[3];
  INA226_Init(pIna226, &hi2c2, INA226_24V);
  INA226_SoftwareReset(pIna226);
  INA226_SetCalibrationRegister(pIna226, 1.0f, 0.03f); // max current: 0.328A, shunt res: 0.1ohm

  pIna226 = &ina226[4];
  INA226_Init(pIna226, &hi2c2, INA226_TEC);
  INA226_SoftwareReset(pIna226);
  INA226_SetCalibrationRegister(pIna226, 1.5f, 0.03f); // max current: 0.328A, shunt res: 0.1ohm
}

void Top_UpdateStatus(void)
{
  for(int i = 0; i < MAX_INA226_CH; i++)
    udpate_ina226(i);

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

  uint16_t* p = (uint16_t*)&vbus;
  usRegInputBuf[ch * 8] = *(p+1);
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


