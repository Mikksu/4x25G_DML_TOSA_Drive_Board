#ifndef __MB_OS_DEF_H
#define	__MB_OS_DEF_H

#include "stddef.h"

typedef struct
{
	uint16_t RegIndex;
	uint16_t NRegs;
	uint16_t RegBitIndex;

} MB_MSG_TypeDef;

void CreateMbCoilProcTask(void);
void CreateMbHoldingProcTask(void);

#endif
