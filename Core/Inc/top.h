#ifndef __TOP_H
#define __TOP_H

#include "stdint.h"
#include "ina226.h"
#include "adn8835.h"
#include "pidcontroller.h"

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
#define INA226_TEC    (0x41 << 1)

typedef enum
{
  TEC_MODE_HEATER,
  TEC_MODE_TEC

} TOP_TecMode_TypeDef;

typedef struct
{
  INA226_OpModeTypeDef            OpMode;
  INA226_VSHCTTypeDef             VSHCT;
  INA226_VBUSCTTypeDef            VBUSCT;
  INA226_AvgModeTypeDef           AvgMode;
  INA226_CalParamTypeDef          CalibrationParam;

} TOP_INA226_Conf_TypeDef;

typedef struct
{
  int                             Mode;
  float                           P;
  float                           I;
  float                           D;
  float                           SamplingIntervalMs;
  float                           NTCCoeffA;
  float                           NTCCoeffB;
  float                           NTCCoeffC;

} TOP_TEC_Conf_TypeDef;

typedef struct
{
  TOP_INA226_Conf_TypeDef         INA226Conf[MAX_INA226_CH];
  TOP_TEC_Conf_TypeDef            TECConf;

} Top_Env_TypeDef;


extern INA226_HandleTypeDef ina2261, ina2262, ina2263;
extern ADN8835_TypeDef adn8835;
extern PID_TypeDef pid;
extern Top_Env_TypeDef env;

void Top_Init(void);
void Top_LoadEnvFromFlash(void);
void Top_SaveEnvToFlash(void);
void Top_UpdateStatus(void);

void Top_TurnOnLed(void);
void Top_TurnOffLed(void);

void Top_TurnOnVcc1(void);
void Top_TurnOffVcc1(void);
void Top_TurnOnVcc2(void);
void Top_TurnOffVcc2(void);
void Top_TurnOnVcc3(void);
void Top_TurnOffVcc3(void);

void Top_TurnOnTec(void);
void Top_TurnOffTec(void);
void Top_SetTecNtcCoeffA(float coeff);
void Top_SetTecNtcCoeffB(float coeff);
void Top_SetTecNtcCoeffC(float coeff);
void Top_SetTecMode(TOP_TecMode_TypeDef mode);


void Top_SetPidKp(float kp);
void Top_SetPidKi(float ki);
void Top_SetPidKd(float kd);
void Top_SetPidSamplingInterval(uint16_t ms);

#endif
