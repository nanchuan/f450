#ifndef __UCA_SPI_H
#define __UCA_SPI_H

#include "msp430init.h"

#define CSNDIR P9DIR
#define CSNOUT P9OUT
#define CSNBIT BIT7

void UCA_SPIInit();

uchar SPI_readReg(uchar addr);

uchar SPI_writeReg(uint8_t Address, uint8_t Data);

uchar SPI_rw(uchar d);

uchar SPI_readBuf(uchar reg, uchar *p,uchar n);

uchar SPI_writeBuf(uchar reg, uchar *p,uchar n);














#endif