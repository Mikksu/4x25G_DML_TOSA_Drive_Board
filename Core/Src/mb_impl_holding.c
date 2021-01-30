#include "port.h"
#include "mb.h"
#include "mbutils.h"

#define REG_HOLDING_START 0
#define REG_HOLDING_NREGS 100
static USHORT usRegHoldingStart = REG_HOLDING_START;
USHORT usRegHoldingBuf[REG_HOLDING_NREGS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

#ifdef MB_OS_USED
#include "cmsis_os.h"
#include "mb_os_def.h"

osMessageQDef(msgQueueHolding, 5, uint32_t);
osMessageQId msgQueueHoldingHandle;

osPoolDef(poolHoldingMsg, 5, MB_MSG_TypeDef);
osPoolId poolHoldingMsgHandle;

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
            // some of the coil regs are changed.
        }
    }
  }
}

osThreadDef(regHoldingTask, StartTaskRegHolding, osPriorityNormal, 0, 128);
osThreadId regHoldingTaskHandle;

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
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

