#ifndef __TOP_H
#define __TOP_H

#include <stdint.h>
#include <string.h>
#include "ina226.h"
#include "adn8835.h"

#define REG_HOLDING_POS_DAC_OUTPUT      (67)
#define REG_HOLDING_POS_DUT_IIC_OPER    (80)
#define REG_HOLDING_POS_ERR_CODE        (98)
#define REG_HOLDING_POS_EXECUTE         (99)

#define REG_INPUT_POS_RTTEMP            (44)
#define REG_INPUT_POS_TARGETTEMP        (46)
#define REG_INPUT_POS_TEC_INC           (48)
#define REG_INPUT_POS_TEC_DAC           (50)


/*
 * @brief         The maximum size of the buffer for the DUT i2c operation
 */
#define MAX_SIZE_DUT_I2C_BUF            (10)

/*
 * @brief         How many INA226s are there on the board.
 */
#define MAX_INA226_CH  (5)

/*
 * @brief         The base address of the flash where storaging the ENV. (Sector11, 128K)
 */
#define MEM_BASE_ENV   (0x080E0000UL)

/*
 * @brief         The I2C address of the INA226s.
 */
#define INA226_VCC1   (0x40 << 1)
#define INA226_VCC2   (0x41 << 1)
#define INA226_VCC3   (0x44 << 1)
#define INA226_24V    (0x40 << 1)
#define INA226_TEC    (0x41 << 1)

/*
 * @brief         The definition of the Error Code.
 */
#define ERR_NO                          (0)
#define ERR_PID_PARAM                   (-10)         /*!< Invalid PID parameters             */
#define ERR_PID_INVALID_RTTEMP          (-11)         /*!< Invalid real-time temperature read while PID tuning    */
#define ERR_PID_INVALID_TARGET_TEMP     (-12)         /*!< Invalid target temperature    */
#define ERR_PID_RTTEMP_TOO_LOW          (-13)         /*!< The real time temp. is too low       */
#define ERR_PID_RTTEMP_TOO_HIGH         (-14)         /*!< The real time temp. is too high       */
#define ERR_DUT_I2C_NO_ACK              (-20)         /*!< No ack detected on the I2C bus to communicate with the DUT    */
#define ERR_DUT_I2C_BUS_BUSY            (-21)         /*!< The SDA line can not be pull to high while starting the DUT i2c bus        */

#define ERR_UNDEFINED                   (-999)        /*!< Undefined error    */

typedef enum
{
  TEC_MODE_HEATER,
  TEC_MODE_TEC

} TOP_TecMode_TypeDef;

#pragma pack(push)
#pragma pack(1)

typedef struct
{
  union
  {
    uint8_t                     SwCtl;
    struct
    {
      uint8_t                   SwVcc1:1;
      uint8_t                   SwVcc2:1;
      uint8_t                   SwVcc3:1;
      uint8_t                   TecEn:1;
      uint8_t                   DutPs:1;
      uint8_t                   DutTxDis:1;
      uint8_t                   DutRst:1;
    };
  };

} Top_SwCtrl_Typedef;

typedef struct
{
  uint16_t            			    OpMode;
  uint16_t                      VSHCT;
  uint16_t                      VBUSCT;
  uint16_t                      AvgMode;
  INA226_CalParamTypeDef        CalibrationParam;

} TOP_INA226_Conf_TypeDef;

typedef struct
{
  union
  {
    uint16_t                    Config;
    struct
    {
      uint16_t                    Mode:1;             /*!< 0: Heater; 1: TEC                    */
      uint16_t                    Polarity:1;         /*!< 0: Normal; 1:Switch the TEC+/TEC-    */
      uint16_t                    ManualControl:1;    /*!< 0: Manual Control; 1: Auto Control   */
    };
  };
  float                         P;
  float                         I;
  float                         D;
  float                         SamplingIntervalMs;
  float                         NTCCoeffA;
  float                         NTCCoeffB;
  float                         NTCCoeffC;
  float                         NTCRefResistorOhm;
  float                         NTCVref;
  float                         ADCVref;
  float                         TempProteLow;
  float                         TempProteHigh;
  float                         TargetTemp;
  float                         DacOutputMv;

} TOP_TEC_Conf_TypeDef;

typedef struct
{
  TOP_INA226_Conf_TypeDef       INA226Conf[MAX_INA226_CH];
  TOP_TEC_Conf_TypeDef          TECConf;

} Top_Env_TypeDef;

typedef struct 
{
  uint16_t                      SlaveAddress;
  uint16_t                      RegStart;
  uint16_t                      RegLength;
  uint16_t                      Data[MAX_SIZE_DUT_I2C_BUF];

} Top_DutI2cOper_TypeDef;

typedef struct
{
  float                         Vcc;
  float                         Vshunt;
  float                         Current;
  float                         Power;

} Top_Vcc_Mon_TypeDef;

typedef struct
{
  float                         Vtec;
  float                         Itec;
  float                         RtTemp;
  float                         TargetTemp;
  float                         PidInc;
  float                         PidCtlLevel;
  uint16_t                      TecMode;
  uint16_t                      TecPolarity;
  float                         Vntc;
  float                         Intc;
  float                         Rntc;

} Top_Tec_Mon_TypeDef;

typedef struct
{
  Top_Vcc_Mon_TypeDef           INA226Mon[MAX_INA226_CH];
  Top_Tec_Mon_TypeDef           Tec;
  int16_t                       ErrorCode;

} Top_Monitoring_TypeDef;

#pragma pack(pop)


extern INA226_HandleTypeDef ina2261, ina2262, ina2263;
extern ADN8835_TypeDef adn8835;
extern Top_SwCtrl_Typedef *swCtrl;
extern Top_Env_TypeDef *env;
extern Top_DutI2cOper_TypeDef *dutI2c;

void Top_Init(void);
void Top_LoadEnvFromFlash(void);
void Top_SaveEnvToFlash(void);
float Top_ReadRealtimeTemp(void);
void Top_UpdateStatus(void);

void Top_TurnOnLed(void);
void Top_TurnOffLed(void);

void Top_TurnOnVcc1(void);
void Top_TurnOffVcc1(void);
void Top_TurnOnVcc2(void);
void Top_TurnOffVcc2(void);
void Top_TurnOnVcc3(void);
void Top_TurnOffVcc3(void);

void Top_DutTxEnable(void);
void Top_DutTxDisable(void);
void Top_DutUnreset(void);
void Top_DutReset(void);

void Top_TurnOnTec(void);
void Top_TurnOffTec(void);
void Top_SetTecNtcCoeffA(float coeff);
void Top_SetTecNtcCoeffB(float coeff);
void Top_SetTecNtcCoeffC(float coeff);
void Top_SetTecMode(TOP_TecMode_TypeDef mode);
void Top_TecSetDacVolt(float volt);
void Top_SetTecControlLevel(float level);
void Top_TecTune(void);

void Top_SetErrorCode(int16_t errCode);

#endif
