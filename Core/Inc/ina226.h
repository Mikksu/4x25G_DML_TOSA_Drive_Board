#ifndef __INA226_H
#define __INA226_H

#include "stdint.h"
#include "i2c.h"

#define INA226_REG_CONFIG             (0x00)
#define INA226_REG_SHUNTVOLTAGE       (0x01)
#define INA226_REG_BUSVOLTAGE         (0x02)
#define INA226_REG_POWER              (0x03)
#define INA226_REG_CURRENT            (0x04)
#define INA226_REG_CALIBRATION        (0x05)
#define INA226_REG_MASK               (0x06)
#define INA226_REG_ALERTLIMT          (0x07)
#define INA226_REG_MANUF_ID           (0xFE)
#define INA226_REG_DIE_ID             (0xFF)


#define INA226_CONFIG_MODE_POS        (0U)
#define INA226_CONFIG_MODE_MASK       (0b111 << INA226_CONFIG_MODE_POS)                  /*!< 0x0007 */
#define INA226_CONFIG_MODE            INA226_CONFIG_MODE_MASK                            /*!< Operation Mode                        */
#define INA226_CONFIG_VSHCT_POS       (3U)
#define INA226_CONFIG_VSHCT_MASK      (0b111 << INA226_CONFIG_VSHCT_POS)                  /*!< 0x0038 */
#define INA226_CONFIG_VSHCT           INA226_CONFIG_VSHCT_MASK                            /*!< Shunt Voltage Conversion Time         */
#define INA226_CONFIG_VBUSCT_POS      (6U)
#define INA226_CONFIG_VBUSCT_MASK     (0b111 << INA226_CONFIG_VBUSCT_POS)                 /*!< 0x01C0 */
#define INA226_CONFIG_VBUSCT          INA226_CONFIG_VBUSCT_MASK                           /*!< Bus Voltage Conversion Time         */
#define INA226_CONFIG_AVG_POS         (9U)
#define INA226_CONFIG_AVG_MASK        (0b111 << INA226_CONFIG_AVG_POS)                    /*!< 0x0E00 */
#define INA226_CONFIG_AVG             INA226_CONFIG_AVG_MASK                              /*!< Averaging Mode                      */

typedef enum
{
  INA266_AVG_1 =    (0 << INA226_CONFIG_AVG_POS),
  INA266_AVG_4 =    (1 << INA226_CONFIG_AVG_POS),
  INA266_AVG_16 =   (2 << INA226_CONFIG_AVG_POS),
  INA266_AVG_64 =   (3 << INA226_CONFIG_AVG_POS),
  INA266_AVG_128 =  (4 << INA226_CONFIG_AVG_POS),
  INA266_AVG_256 =  (5 << INA226_CONFIG_AVG_POS),
  INA266_AVG_512 =  (6 << INA226_CONFIG_AVG_POS),
  INA266_AVG_1024 = (7 << INA226_CONFIG_AVG_POS)

} INA226_AvgModeTypeDef;


typedef struct
{
    float                       MaxExceptedCurrentA;
    float                       ShuntResistorOhm;
    float                       CurrentLsbA;
} INA226_CalParamTypeDef;


/*
 * @brief         The structure of the INA266 object.
 */
typedef struct __INA226_HandleTypeDef
{
  I2C_HandleTypeDef*            I2CPort;
  uint8_t                       I2CAddress;
  union
  {
    uint16_t                    RegConfiguration;                 /*!< The value of the registor Configuration            */
    struct
    {
      uint16_t                    RegConfiguration_MODE:3;        /*!< Operation Mode                               */
      uint16_t                    RegConfiguration_VSHCT:3;       /*!< Shunt Voltage Conversion Time                */
      uint16_t                    RegConfiguration_VBUSCT:3;      /*!< Bus Voltage Conversion Time                  */
      uint16_t                    RegConfiguration_AVG:3;         /*!< Averaging Mode                               */
      uint16_t                    :3;                             /*!< Reserved                                     */
      uint16_t                    RegConfiguration_RST:1;         /*!< Software Reset                               */
    };
  };
  uint16_t                      RegShuntVoltage;
  uint16_t                      RegBusVoltage;
  uint16_t                      RegPower;
  uint16_t                      RegCurrent;
  uint16_t                      RegCalibration;
  uint16_t                      RegMaskEnable;
  uint16_t                      RegAlertLimt;
  union
  {
    uint32_t                    ChipID;
    struct
    {
      uint8_t                     ManufID[2];
      uint8_t                     DieID[2];
    };
  };

  INA226_CalParamTypeDef        CalibrationParam;                 /*!< The parameters to calculate the Calibration Register  */

  void                          (*OnAvgModeChangedCb)(struct __INA226_HandleTypeDef*, int);

} INA226_HandleTypeDef;


void INA226_OnAveragingModeChanged(struct __INA226_HandleTypeDef* this, int e);



/*
 * @brief         Initialize the INA226
 *
 * @param ina226 			The point to the instance of the INA226.
 * @param hi2c				The point to i2c handler.
 * @param address			The slave address of the i2c of the INA226.
 * 
 */
HAL_StatusTypeDef INA226_Init(INA226_HandleTypeDef* ina226, I2C_HandleTypeDef* hi2c, uint8_t address);


/*
 * @brief         Reset the INA226 by the software.
 *
 * @param ina226      The point to the instance of the INA226.
 */
HAL_StatusTypeDef INA226_SoftwareReset(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read all the registors.
 *
 * @param ina226      The point to the instance of the INA226.
 */
void INA226_ReloadRegistors(INA226_HandleTypeDef* ina226);

/*
 * @brief         Read the value of the Configration registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
uint16_t INA226_ReadConfigurationReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Shunt Voltage registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
float INA226_ReadShuntVoltageReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Bus Voltage registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
float INA226_ReadBusVoltageReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Power registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
float INA226_ReadPowerReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Current registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
float INA226_ReadCurrentReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Calibration registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
uint16_t INA226_ReadCalibrationReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Mask/Enable registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
uint16_t INA226_ReadMaskReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Read the value of the Alert Limit registor.
 *
 * @param ina226      The point to the instance of the INA226.
 */
uint16_t INA226_ReadAlertLimitReg(INA226_HandleTypeDef* ina226);


/*
 * @brief         Set the Averaging mode of the INA226.
 *
 * @param ina226          The point to the instance of the INA226.
 * @param averagingMode   The Averaging Mode.
 */
HAL_StatusTypeDef INA226_SetAveragingMode(INA226_HandleTypeDef* ina226, INA226_AvgModeTypeDef mode);


/*
 * @brief         Calculate and set the Calibration Register.
 *
 * @param ina226                The point to the instance of the INA226.
 * @param maxExceptedCurrentA   The maximum excpeted current in A.
 * @param shuntResOhm           The shunt resistor in ohm.
 */
HAL_StatusTypeDef INA226_SetCalibrationRegister(INA226_HandleTypeDef* ina226, float maxExceptedCurrentA, float shuntResOhm);


#endif
