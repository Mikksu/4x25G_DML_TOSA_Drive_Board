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
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

