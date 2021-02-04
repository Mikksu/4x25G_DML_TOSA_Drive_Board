#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "stdint.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    int SamplingInterval; // in ms

} PID_TypeDef;

/* Initilize the data structures and parameters */
void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd, int interval);

/* Set value of PID parameters */
void PID_SetKp(PID_TypeDef* pid, float data);
void PID_SetKi(PID_TypeDef* pid, float data);
void PID_SetKd(PID_TypeDef* pid, float data);
void PID_SetSamplingInterval(PID_TypeDef* pid, uint16_t data);

/** Reset the status of the controller */
void PID_Reset(PID_TypeDef* pid);

/* Process input data and output the factor */
int PID_Tune(PID_TypeDef* pid, int measured, int sp);

#endif
