#include "port.h"
#include "mb.h"
#include "mbutils.h"

#define COIL_START 0
#define COIL_NCOILS 100
static USHORT usCoilStart = COIL_START;
static UCHAR usCoilBuf[COIL_NCOILS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
#if S_COIL_NCOILS % 8
UCHAR ucSCoilBuf[COIL_NCOILS / 8 + 1];
#else
UCHAR ucSCoilBuf[COIL_NCOILS / 8];
#endif

#ifdef MB_OS_USED

#include "cmsis_os.h"
#include "mb_os_def.h"

/*
 * The message queue to pass the message indicates where and how many registers are modified.
 */
osMessageQDef(msgQueueCoil, 5, uint32_t);
osMessageQId msgQueueCoilHandle;

/*
 * The pool of the message.
 */
osPoolDef(poolCoilMsg, 5, MB_MSG_TypeDef);
osPoolId poolCoilMsgHandle;

/*
 * The handle of the processing task.
 */
osThreadId regCoilTaskHandle;

static void StartTaskRegCoil(void const * argument)
{
  // create the message queue.
  msgQueueCoilHandle = osMessageCreate(osMessageQ(msgQueueCoil), NULL);

  // create the message pool.
  poolCoilMsgHandle = osPoolCreate(osPool(poolCoilMsg));

  for(;;)
  {
    // wait the message.
    osEvent evt = osMessageGet(msgQueueCoilHandle, osWaitForever);
    if(evt.status == osEventMessage)
    {
        // get the pointer of the modbus message struct.
        MB_MSG_TypeDef* msg = evt.value.p;
        if(msg != NULL)
        {
            // some of the coil regs are changed.
        }

        osPoolFree(poolCoilMsgHandle, (void*)msg);
    }
  }
}

/*
 * Create the task to process the coil registers.
 */
void CreateMbCoilProcTask(void)
{
  osThreadDef(regCoilTask, StartTaskRegCoil, osPriorityNormal, 0, 128);
  regCoilTaskHandle = osThreadCreate(osThread(regCoilTask), NULL);
}

#endif

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    iNReg = usNCoils / 8 + 1;

    usAddress--;

    if ((usAddress >= usCoilStart) && (usAddress + usNCoils <= usCoilStart + COIL_NCOILS))
    {
        iRegIndex = (USHORT)(usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT)(usAddress - usCoilStart) % 8;
        switch (eMode)
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, 8, *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&usCoilBuf[iRegIndex++], iRegBitIndex, usNCoils, *pucRegBuffer++);
            }

#ifdef MB_OS_USED
            // send a message to tell the task there are some registers are set.
            MB_MSG_TypeDef* msg = osPoolCAlloc(poolCoilMsgHandle);
            if(msg != NULL)
            {
               msg->Address = usAddress;
               msg->NRegs = usNCoils;
               osMessagePut(msgQueueCoilHandle, (uint32_t)msg, 100);
            }
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

