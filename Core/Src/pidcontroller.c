#include "pidcontroller.h"

static int errLast, errLastLast;

void PID_Init(PID_TypeDef* pid, float kp, float ki, float kd, int interval)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->SamplingInterval = interval;
    
    errLast = 0;
    errLastLast = 0;
}


void PID_SetSamplingInterval(PID_TypeDef* pid, uint16_t data)
{
    pid->SamplingInterval = data;
}

void PID_Reset(PID_TypeDef* pid)
{
  errLast = 0;
  errLastLast = 0;
}

int PID_Tune(PID_TypeDef* pid, int measured, int sp)
{
    int err = sp - measured;
    int increment = (int)(pid->Kp * (err - errLast) + pid->Ki * err + pid->Kd * (err - 2 * errLast + errLastLast));
    
    errLastLast = errLast;
    errLast = err;
    
    return increment;
}
