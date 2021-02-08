#include "port.h"
#include "mb.h"
#include "mbutils.h"
#include "top.h"
#include "dac.h"

#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 100
static USHORT usRegHoldingStart = REG_HOLDING_START;
USHORT usRegHoldingBuf[REG_HOLDING_NREGS] = {0, };

#ifdef MB_OS_USED
#include "cmsis_os.h"
#include "mb_os_def.h"

osMessageQDef(msgQueueHolding, 5, uint32_t);
osMessageQId msgQueueHoldingHandle;

osPoolDef(poolHoldingMsg, 5, MB_MSG_TypeDef);
osPoolId poolHoldingMsgHandle;

osThreadId regHoldingTaskHandle;

void StartTaskRegHolding(void const * argument)
{
  // create the message queue.
  msgQueueHoldingHandle = osMessageCreate(osMessageQ(msgQueueHolding), NULL);

  // create the message pool.
  poolHoldingMsgHandle = osPoolCreate(osPool(poolHoldingMsg));

  for(;;)
  {
    osEvent evt = osMessageGet(msgQueueHoldingHandle, osWaitForever);
    if(evt.status == osEventMessage)
    {
        // get the pointer of the modbus message struct.
        MB_MSG_TypeDef* msg = evt.value.p;
        if(msg != NULL)
        {
          // some of the input regs are changed.
          int nRegs = msg->NRegs;
          int regIndex = msg->RegIndex;
          uint16_t regVal;

          while(nRegs > 0)
          {
            switch(regIndex)
            {

              case REG_HOLDING_POS_DAC_OUTPUT:
                ;
                uint16_t dacHex = (uint16_t)(env->TECConf.DacOutputMv / env->TECConf.ADCVref * 4096.0f);
                HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacHex);
                HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
                break;

              case REG_HOLDING_POS_EXECUTE:  // Env Operation
                regVal = usRegHoldingBuf[regIndex];
                usRegHoldingBuf[regIndex] = 0x0;

                if (regVal == 17747) // Save Env to the flash
                {
                  Top_SaveEnvToFlash();
                }
                else if (regVal == 17740) // Reload Env from the flash
                {
                  Top_LoadEnvFromFlash();
                }
                else if (regVal == 18770) // Read data from device via I2C
                {

                }
                else if (regVal == 18775) // Write data to device via I2C
                {

                }
            }

            nRegs--;
            regIndex++;
          }

          osPoolFree(poolHoldingMsgHandle, (void*)msg);
        }
    }
  }
}


/*
 * Create the task to process the coil registers.
 */
void CreateMbHoldingProcTask(void)
{
  osThreadDef(regHoldingTask, StartTaskRegHolding, osPriorityNormal, 0, 256);
  regHoldingTaskHandle = osThreadCreate(osThread(regHoldingTask), NULL);
}

#endif


/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;

#ifdef MB_OS_USED
    MB_MSG_TypeDef* msg;
#endif


    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= usRegHoldingStart) && (usAddress + usNRegs <= usRegHoldingStart + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldingStart;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:

#ifdef MB_OS_USED
            // send a message to tell the task there are some registers are set.

            msg = osPoolCAlloc(poolHoldingMsgHandle);
            if(msg != NULL)
            {
               msg->RegIndex = iRegIndex;
               msg->NRegs = usNRegs;
               msg->RegBitIndex = 0;
            }
#endif
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }

#ifdef MB_OS_USED
            if(msg != NULL)
              osMessagePut(msgQueueHoldingHandle, (uint32_t)msg, 100);

#endif
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

