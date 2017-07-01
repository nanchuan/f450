#include "UCA_SPI.h"


//初始化
/******************************************************************************/
void UCA_SPIInit()
{
  P9SEL |= 0x31;                                // P9.5,4,0 option select
  CSNDIR |= CSNBIT;
  CSNOUT |= CSNBIT; 
  
  UCA2CTL1 |= UCSWRST;                          // **Put state machine in reset**
  UCA2CTL0 |= UCCKPH+ UCMSB+ UCMST+ UCSYNC;     // 3-pin, 8-bit SPI master
  UCA2CTL1 |= UCSSEL_2;                         // SMCLK
  UCA2BR0 = 0x02;                               // /2
  UCA2BR1 = 0;                                  //
  UCA2MCTL = 0;                                 // No modulation
  UCA2CTL1 &= ~UCSWRST;                         // **Initialize USCI state machine**
}

//读寄存器
/******************************************************************************/
uchar SPI_readReg(uchar addr)
{
  uchar rec_val;
  
  CSNOUT &= ~CSNBIT;
  rec_val = UCA2RXBUF;
  while (!(UCA2IFG&UCTXIFG));                   // USCI_A0 TX buffer ready?
  UCA2TXBUF = addr;
  while (!(UCA2IFG&UCRXIFG));
  rec_val = UCA2RXBUF;
  while (!(UCA2IFG&UCTXIFG)); 
  UCA2TXBUF = 0;
  while (!(UCA2IFG&UCRXIFG));
  rec_val = UCA2RXBUF;
  while (UCA2STAT & UCBUSY) ;
  CSNOUT |= CSNBIT;
  
  return rec_val;  
}

//写寄存器
/******************************************************************************/
uchar SPI_writeReg(uint8_t Address, uint8_t Data)
{
  uint8_t Result;
  
  CSNOUT &= ~CSNBIT;
  Result = UCA2RXBUF;
  while (!(UCA2IFG & UCTXIFG)) ;
  UCA2TXBUF = Address;
  while (!(UCA2IFG & UCRXIFG)) ;
  Result = UCA2RXBUF;
  while (!(UCA2IFG & UCTXIFG)) ;
  UCA2TXBUF = Data;
  while (!(UCA2IFG & UCRXIFG)) ;
  Result = UCA2RXBUF;
  while (UCA2STAT & UCBUSY) ;
  CSNOUT |= CSNBIT;
  return Result;
}

//
/******************************************************************************/
uchar SPI_rw(uchar d)
{
  uchar result;
  
  result = UCA2RXBUF;
  while (!(UCA2IFG & UCTXIFG)) ;
  UCA2TXBUF = d;
  while (!(UCA2IFG & UCRXIFG)) ;
  result = UCA2RXBUF;
//  while (UCA2STAT & UCBUSY) ;
  
  return result;
}

//读取多个数据
/******************************************************************************/
uchar SPI_readBuf(uchar reg, uchar *p,uchar n)
{
  uchar a,i;
  CSNOUT &= ~CSNBIT;
  a = UCA2RXBUF;
  while (!(UCA2IFG & UCTXIFG)) ;
  UCA2TXBUF = reg;
  while (!(UCA2IFG & UCRXIFG)) ;
  a = UCA2RXBUF;
  for(i=0;i<n;i++)
  {
    while (!(UCA2IFG & UCTXIFG)) ;
    UCA2TXBUF = 0;
    while (!(UCA2IFG & UCRXIFG)) ;
    p[i] = UCA2RXBUF;
  }
  CSNOUT |= CSNBIT;
  
  return a;
}

//写入多个数据
/******************************************************************************/
uchar SPI_writeBuf(uchar reg, uchar *p,uchar n)
{
  uchar a,i;
  CSNOUT &= ~CSNBIT;
  a = UCA2RXBUF;
  while (!(UCA2IFG & UCTXIFG)) ;
  UCA2TXBUF = reg;
  while (!(UCA2IFG & UCRXIFG)) ;
  a = UCA2RXBUF;
  for(i=0;i<n;i++)
  {
    while (!(UCA2IFG & UCTXIFG)) ;
    UCA2TXBUF = p[i];
    while (!(UCA2IFG & UCRXIFG)) ;
    a = UCA2RXBUF;
  }
  CSNOUT |= CSNBIT;
  
  return a;
}
























