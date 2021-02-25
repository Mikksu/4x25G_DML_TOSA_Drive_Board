#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "gpio.h"
#include "dac.h"
#include "adn8835.h"


#define ADC_AVERAGE_TIMES   20

extern osSemaphoreId semADBusyHandle;

static float ConvertNTC2Celsius(float Ohm, float coA, float coB, float coC)
{
    // refers to https://www.thermistor.com/calculators?r=sheccr
    
    float ln_ohm = logf(Ohm);
    float temp = 1 / (coA + coB * ln_ohm + coC * powf(ln_ohm, 3.0f)) - 273.15f;
    if(temp < -100.0f || temp > 300.0f)
      temp = NAN;

    return temp;
}

void ADN8835_Init(ADN8835_TypeDef* adn8835)
{
  adn8835->IsInitialized                   = 0;
  adn8835->Mode                            = ADN8835_HEATER_MODE;
  adn8835->EnPin.Gpio                      = GPIOA;
  adn8835->EnPin.Pin                       = GPIO_PIN_5;
  adn8835->Analog.Handle                   = &hadc1;                   // ADC1 is used
  adn8835->Analog.VrefADC                  = 3264;                     // 3.3v voltage is used as the ADC Vref
  adn8835->Analog.VrefNTC                  = 2500;                     // 2.5v is used as the power supply of the NTC
  adn8835->Analog.RrefNTC                  = 10000;                    // 10K resistor is used as the reference of the NTC
  adn8835->Analog.ADCChannelNTC            = ADC_CHANNEL_0;            // Channel 0 is used to measure the TOSA temperature
  adn8835->Analog.ADCChannelVTEC           = ADC_CHANNEL_10;           // Channel 10 is used to measure the VTEC
  adn8835->Analog.ADCChannelITEC           = ADC_CHANNEL_11;           // Channel 11 is used to measure the ITEC
  adn8835->NTCCoeff.CoA                    = 0.001125161025848;
  adn8835->NTCCoeff.CoB                    = 0.000234721098632;
  adn8835->NTCCoeff.CoC                    = 0.000000085877049;
  adn8835->PIDParam.TargetTemperature      = ADN8835_INVALIED_VALUE;
  adn8835->PIDParam.LastTempCtrlLevel      = 0;
  adn8835->IsInitialized                   = 1;
    
  ADN8835_Disable(adn8835);
}

void ADN8835_Enable(ADN8835_TypeDef* adn8835)
{
  // Ven > 2.1v makes the ADN8835 enabled
  HAL_GPIO_WritePin(adn8835->EnPin.Gpio, adn8835->EnPin.Pin, GPIO_PIN_SET);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  adn8835->IsAutoTuningStarted = 1;
}

void ADN8835_Disable(ADN8835_TypeDef* adn8835)
{
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
  HAL_GPIO_WritePin(adn8835->EnPin.Gpio, adn8835->EnPin.Pin, GPIO_PIN_RESET);
  adn8835->IsAutoTuningStarted = 0;
}

void ADN8835_SetTargetTemperture(ADN8835_TypeDef* adn8835, int value)
{
    adn8835->PIDParam.TargetTemperature = value;
}

void ADN8835_SetMode(ADN8835_TypeDef* adn8835, ADN8835_Mode_TypeDef mode)
{
    adn8835->Mode = mode;
}

void ADN8835_SetControlLevel(ADN8835_TypeDef* adn8835, float level, float vref)
{
  // restrict the range of the level to -1250 ~ 1250
  if(level < -1250)
      level = -1250;
  else if(level > 1250)
      level = 1250;

  // convert to level to 0 ~ 2500 for the output of the DAC.
  float volt = level + 1250.0f;

  uint16_t dacHex = volt / vref * 4096.0f;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacHex);
}

float ADN8835_ReadTemp(ADN8835_TypeDef* adn8835, float vrefAdc, float vrefNtc, float coA, float coB, float coC)
{
    if(adn8835->IsInitialized)
    {
        ADC_ChannelConfTypeDef sConfig;
        uint32_t adcHex = 0;
        float voltNTC = 0;
        float ohmNTC = 0;
        float c;

        if(osSemaphoreWait(semADBusyHandle, 200) == osOK)
        {
            sConfig.Channel = adn8835->Analog.ADCChannelNTC;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
            HAL_ADC_ConfigChannel(adn8835->Analog.Handle, &sConfig);

            for(int i = 0; i < ADC_AVERAGE_TIMES; i++)
            {
                HAL_ADC_Start(adn8835->Analog.Handle);
                HAL_ADC_PollForConversion(adn8835->Analog.Handle, 500);
                adcHex += HAL_ADC_GetValue(adn8835->Analog.Handle);
                HAL_ADC_Stop(adn8835->Analog.Handle);
            }

            adcHex /= ADC_AVERAGE_TIMES;

            voltNTC = (float)(adcHex * vrefAdc) / 4095.0f;

            /* IMPORTANT
             *
             * In the case of Vref_ntc<Vref_adc, Vntc may be greater than Vref_ntc, which results in a negative
             * current calculation value, and thus a negative resistance calculation value; therefore, the maximum
             * value of Vntc needs to be constrained within the range of Vref_ntc.
             */
            if(voltNTC > vrefNtc)
              voltNTC = vrefNtc;
						
            float curr = (vrefNtc - voltNTC) / (float)adn8835->Analog.RrefNTC;
            
						ohmNTC = voltNTC / curr;

						adn8835->NtcMonitoring.Volt = voltNTC;
						adn8835->NtcMonitoring.Curr = curr;
						adn8835->NtcMonitoring.Ohm = ohmNTC;

            c =  ConvertNTC2Celsius(ohmNTC, coA, coB, coC);
        }
        else
        {
            // AD is busy
            c = NAN;
        }
        
        osSemaphoreRelease(semADBusyHandle);
        
        return c;
    }
    else
    {
        return NAN;
    }
}

float ADN8835_ReadVTEC(ADN8835_TypeDef* adn8835, float vref)
{
    if(adn8835->IsInitialized)
    {
        ADC_ChannelConfTypeDef sConfig;
        uint32_t adcHex = 0;
        float volt = 0;
        float vtec = 0;
        
        if(osSemaphoreWait(semADBusyHandle, 200) == osOK)
        {
            sConfig.Channel = adn8835->Analog.ADCChannelVTEC;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
            HAL_ADC_ConfigChannel(adn8835->Analog.Handle, &sConfig);
            
            for(int i = 0; i < ADC_AVERAGE_TIMES; i++)
            {
                HAL_ADC_Start(adn8835->Analog.Handle);
                HAL_ADC_PollForConversion(adn8835->Analog.Handle, 500);
                adcHex += HAL_ADC_GetValue(adn8835->Analog.Handle);
                HAL_ADC_Stop(adn8835->Analog.Handle);
            }
            
            adcHex /= ADC_AVERAGE_TIMES;
            
            volt = (float)adcHex * vref / 4096.0f;
            vtec = (volt - 1250.0f) * 4.0f;
        }
        else
        {
            // ADC is busy
            vtec = ADN8835_INVALIED_VALUE;
        }
        
        osSemaphoreRelease(semADBusyHandle);
        
        return vtec;
    }
    else
    {
        return ADN8835_INVALIED_VALUE;
    }
}

float ADN8835_ReadITEC(ADN8835_TypeDef* adn8835, float vref)
{
    if(adn8835->IsInitialized)
    {
        ADC_ChannelConfTypeDef sConfig;
        uint32_t adcHex = 0;
        float volt = 0;
        float itec = 0;
        
        if(osSemaphoreWait(semADBusyHandle, 200) == osOK)
        {

            sConfig.Channel = adn8835->Analog.ADCChannelITEC;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
            HAL_ADC_ConfigChannel(adn8835->Analog.Handle, &sConfig);
            
            for(int i = 0; i < ADC_AVERAGE_TIMES; i++)
            {
                HAL_ADC_Start(adn8835->Analog.Handle);
                HAL_ADC_PollForConversion(adn8835->Analog.Handle, 500);
                adcHex += HAL_ADC_GetValue(adn8835->Analog.Handle);
                HAL_ADC_Stop(adn8835->Analog.Handle);
            }
            
            adcHex /= ADC_AVERAGE_TIMES;
            
            volt = (float)adcHex * vref / 4096.0f;
            itec = (volt - 1250.0f) * 7.0f / 2.0f;
        }
        else
        {
            // ADC is busy
            itec = ADN8835_INVALIED_VALUE;
        }
        
        osSemaphoreRelease(semADBusyHandle);
        
        return itec;
    }
    else
    {
        return ADN8835_INVALIED_VALUE;
    }
}


static float errLast, errLastLast;

float ADN8835_PidTune(ADN8835_TypeDef* adn8835, float measured, float sp, float kp, float ki, float kd)
{
  float err = sp - measured;
  float increment = kp * (err - errLast) + ki * err + kd * (err - 2 * errLast + errLastLast);

  errLastLast = errLast;
  errLast = err;

  return increment;
}

void ADN8835_PidReset(ADN8835_TypeDef* adn8835)
{
    errLast = 0;
    errLastLast = 0;
}
