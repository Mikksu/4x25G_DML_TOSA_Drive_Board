#ifndef __ADN8835_H
#define __ADN8835_H

#include "adc.h"
#define ADN8835_INVALIED_VALUE      -9999

typedef enum
{
    ADN8835_HEATER_MODE,
    ADN8835_TEC_MODE

} ADN8835_Mode_TypeDef;

typedef struct
{
    GPIO_TypeDef*       Gpio;
    unsigned int        Pin;
} ADN8835_ENPin_TypeDef;

typedef struct
{
    ADC_HandleTypeDef*  Handle;
    int                 VrefADC;
    int                 VrefNTC;
    int                 RrefNTC;
    int                 ChannelNTC;
    int                 ChannelVTEC;
    int                 ChannelITEC;
    
} ADN8835_Analog_TypeDef;

typedef struct
{
    int TargetTemperature;
    int LastTempCtrlLevel;
} ADN8835_PIDParam_TypeDef;

typedef struct
{
    float CoA;
    float CoB;
    float CoC;
} NTCCoeff_TypeDef;

typedef struct
{
    int VLIM;
    int ILIM;
} ADN8835_Compliance_TypeDef;

/* The struct contains the parameters to operate the ADN8835 */
typedef struct
{
    ADN8835_Mode_TypeDef        Mode;                   // Note that the way of control is totally differenct between the 2 modes
    unsigned int                IsInitialized;          // Has the struct been initialized?
    unsigned int                IsAutoTuningStarted;    // is the auto tuning process started
    ADN8835_ENPin_TypeDef       EnPin;                  // The gpio of mcu connected to the en pin of adn8835 
    ADN8835_Analog_TypeDef      Analog;                 // The analog component attached to monitor and measure a variety of parameters
    NTCCoeff_TypeDef            NTCCoeff;               // Coefficients of NTC
    ADN8835_Compliance_TypeDef  Compliance;             // Compliance of voltage and current of TEC in percent from 0% - 100%
    ADN8835_PIDParam_TypeDef    PIDParam;               // The parameters used to control the temperature

} ADN8835_TypeDef;

void ADN8835_Init(ADN8835_TypeDef* adn8835);
void ADN8835_Enable(ADN8835_TypeDef* adn8835);
void ADN8835_Disable(ADN8835_TypeDef* adn8835);
void ADN8835_SetMode(ADN8835_TypeDef* adn8835, ADN8835_Mode_TypeDef mode);
void ADN8835_SetTargetTemperture(ADN8835_TypeDef* adn8835, int value);
void ADN8835_SetControlLevel(ADN8835_TypeDef* adn8835, int value);
float ADN8835_ReadTemp(ADN8835_TypeDef* adn8835);
int ADN8835_ReadVTEC(ADN8835_TypeDef* adn8835);
int ADN8835_ReadITEC(ADN8835_TypeDef* adn8835);



#endif
