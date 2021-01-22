#include "port.h"
#include "mb.h"
#include "mbutils.h"


#define REG_INPUT_START 0
#define REG_INPUT_NREGS 100
static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    usAddress--;

    if ((usAddress >= usRegInputStart) && (usAddress + usNRegs <= usRegInputStart + REG_INPUT_NREGS)) //请求地址大于起始地址 && 地址长度小于设定长度
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0)
        {
            *pucRegBuffer++ =
                (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ =
                (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

