#ifndef __TOP_H
#define __TOP_H

#include "stdint.h"
#include "ina226.h"

extern INA226_HandleTypeDef ina2261, ina2262, ina2263;

void Top_Init(void);
void Top_UpdateStatus(void);

void Top_TurnOnLed(void);
void Top_TurnOffLed(void);

void Top_TurnOnVcc1(void);
void Top_TurnOffVcc1(void);
void Top_TurnOnVcc2(void);
void Top_TurnOffVcc2(void);
void Top_TurnOnVcc3(void);
void Top_TurnOffVcc3(void);

#endif
