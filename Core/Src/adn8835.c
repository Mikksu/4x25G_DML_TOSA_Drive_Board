#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "gpio.h"
#include "dac.h"
#include "adn8835.h"


#define ADC_AVERAGE_TIMES   10

extern osSemaphoreId semADBusyHandle;

static float ConvertNTC2Celsius(ADN8835_TypeDef* adn8835, float Ohm)
{
    // refers to https://www.thermistor.com/calculators?r=sheccr
    
    float ln_ohm = logf(Ohm);
    float temp = 1 / (adn8835->NTCCoeff.CoA + adn8835->NTCCoeff.CoB * ln_ohm + adn8835->NTCCoeff.CoC * pow(ln_ohm, 3)) - 273.15;
    return temp;
}

void ADN8835_Init(ADN8835_TypeDef* adn8835)
{
  adn8835->IsInitialized                   = 0;
  adn8835->Mode                            = ADN8835_HEATER_MODE;
  adn8835->EnPin.Gpio                      = GPIOA;
  adn8835->EnPin.Pin                       = GPIO_PIN_5;
  adn8835->Analog.Handle                   = &hadc1;                   // ADC1 is used
  adn8835->Analog.VrefADC                  = 3300;                     // 3.3v voltage is used as the ADC Vref
  adn8835->Analog.VrefNTC                  = 2500;                     // 2.5v is used as the power supply of the NTC
  adn8835->Analog.RrefNTC                  = 10000;                    // 10K resistor is used as the reference of the NTC
  adn8835->Analog.ChannelNTC               = ADC_CHANNEL_0;            // Channel 0 is used to measure the TOSA temperature
  adn8835->Analog.ChannelVTEC              = ADC_CHANNEL_10;           // Channel 10 is used to measure the VTEC
  adn8835->Analog.ChannelITEC              = ADC_CHANNEL_11;           // Channel 11 is used to measure the ITEC
  adn8835->NTCCoeff.CoA                    = 0.001125161025848;
  adn8835->NTCCoeff.CoB                    = 0.000234721098632;
  adn8835->NTCCoeff.CoC                    = 0.000000085877049;
  adn8835->PIDParam.TargetTemperature      = ADN8835_INVALIED_VALUE;
  adn8835->PIDParam.LastTempCtrlLevel      = 0;
  adn8835->IsInitialized                   = 1;
    
    
    
    ADN8835_Disable(adn8835);
    ADN8835_SetControlLevel(adn8835, 0);
}

void ADN8835_Enable(ADN8835_TypeDef* adn8835)
{
  // Ven > 2.1v makes the ADN8835 enabled
  HAL_GPIO_WritePin(adn8835->EnPin.Gpio, adn8835->EnPin.Pin, GPIO_PIN_SET);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

void ADN8835_Disable(ADN8835_TypeDef* adn8835)
{
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
  HAL_GPIO_WritePin(adn8835->EnPin.Gpio, adn8835->EnPin.Pin, GPIO_PIN_RESET);
}

void ADN8835_SetTargetTemperture(ADN8835_TypeDef* adn8835, int value)
{
    adn8835->PIDParam.TargetTemperature = value;
}

void ADN8835_SetMode(ADN8835_TypeDef* adn8835, ADN8835_Mode_TypeDef mode)
{
    adn8835->Mode = mode;
}

void ADN8835_SetControlLevel(ADN8835_TypeDef* adn8835, int level)
{
    if(level < -1250)
        level = -1250;
    else if(level > 1250)
        level = 1250;
    
    //int previousLev = adn8835->PIDParam.LastTempCtrlLevel;
    int volt = 0;

    volt = level + 1250;
    //DAC7558_WriteData(DAC7558_3, DAC7558_CH_G, volt);

    uint16_t dacHex = volt * 3300 / 4096;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacHex);
}

float ADN8835_ReadTemp(ADN8835_TypeDef* adn8835)
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
            sConfig.Channel = adn8835->Analog.ChannelNTC;
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

            voltNTC = (float)(adcHex * adn8835->Analog.VrefADC) / 4096.0f;
						
						// ohmNTC = voltNTC / 0.098; // 0.098 is the output value of the I-Source.
            float curr = ((float)(adn8835->Analog.VrefNTC) - voltNTC) / (float)adn8835->Analog.RrefNTC;
            
						ohmNTC = voltNTC / curr;

            c =  ConvertNTC2Celsius(adn8835, ohmNTC);
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
        return ADN8835_INVALIED_VALUE;
    }
}

int ADN8835_ReadVTEC(ADN8835_TypeDef* adn8835)
{
    if(adn8835->IsInitialized)
    {
        ADC_ChannelConfTypeDef sConfig;
        uint32_t adcHex = 0;
        int volt = 0;
        int vtec = 0;
        
        if(osSemaphoreWait(semADBusyHandle, 200) == osOK)
        {
            sConfig.Channel = adn8835->Analog.ChannelVTEC;
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
            
            volt = adcHex * adn8835->Analog.VrefADC / 4096;
            vtec = (volt - 1250) * 4;
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

int ADN8835_ReadITEC(ADN8835_TypeDef* adn8835)
{
    if(adn8835->IsInitialized)
    {
        ADC_ChannelConfTypeDef sConfig;
        uint32_t adcHex = 0;
        int volt = 0;
        int itec = 0;
        
        if(osSemaphoreWait(semADBusyHandle, 200) == osOK)
        {

            sConfig.Channel = adn8835->Analog.ChannelITEC;
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
            
            volt = adcHex * adn8835->Analog.VrefADC / 4096;
            itec = (volt - 1250) * 7 / 2;
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
