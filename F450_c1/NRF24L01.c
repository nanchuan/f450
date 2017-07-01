#include "NRF24L01.h"
  
uchar TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x44,0x55,0x01};	//本地地址
uchar RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x44,0x55,0x01};	//接收地址 

void Nrf_Init()
{
  UCA_SPIInit();
  delay_us(200);
  IQRIE |= IQRBIT;
  IQRIES |= IQRBIT;
  CEDIR |= CEBIT;
  CEOUT &=~CEBIT;      //ce = 0 
  
  SPI_writeBuf(WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);      // 写本地地址	
  SPI_writeBuf(WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);   // 写接收端地址
  SPI_writeReg(WRITE_REG+EN_AA,0x01);		                //  频道0自动	ACK应答允许
  SPI_writeReg(WRITE_REG+EN_RXADDR,0x01);                       //  允许接收地址只有频道0
  SPI_writeReg(WRITE_REG+SETUP_RETR,0x1a);		        //设置自动重发间隔时间:500us;最大自动重发次数:10次
  SPI_writeReg(WRITE_REG+RF_CH,40);			        //   设置信道工作为2.4GHZ，收发必须一致
//  SPI_writeReg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	        //设置接收数据长度，
  SPI_writeReg(WRITE_REG+RF_SETUP,0x0f);		        //设置发射速率为2MHZ，发射功率为最大值0dB

  
  SPI_writeReg(FLUSH_TX,0xff);
  SPI_writeReg(FLUSH_RX,0xff);
  SPI_writeReg(WRITE_REG + CONFIG, 0x0f);       // IRQ收发完成中断开启,16位CRC,主发送
  
  Spi_RW(0x50);
  Spi_RW(0x73);
  SPI_writeReg(WRITE_REG+0x1c,0x01);
  SPI_writeReg(WRITE_REG+0x1d,0x07);
  
  CEOUT |= CEBIT;
}

void NRF_TxPacket_AP(uchar * tx_buf)
{	
	SPI_CE_L();		 //StandBy I模式	
	SPI_writeBuf(0xa8, tx_buf, 32); 			 // 装载数据
	SPI_CE_H();		 //置高CE
}


uchar Nrf_Check_Event(uchar * NRF24L01_RXDATA)
{
  uchar sta;

  sta = SPI_read(READ_REG + NRFRegSTATUS);
  if(sta & (RX_DR))     //接收中断
  {
    if(SPI_read(R_RX_PL_WID)<33)
    {
      SPI_readBuf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);  // read receive payload from RX_FIFO buffer

    }
    else 
    {
      SPI_rwReg(FLUSH_RX,0xff); //清空缓冲区
    }
  }
  if(sta & (TX_DS))
  {
    
  }
  if(sta & (MAX_RT))
  {
    if(sta & 0x01)              //TX FIFO FULL
    {
      SPI_rwReg(FLUSH_TX,0xff);
    }
  }
  SPI_rwReg(WRITE_REG + NRFRegSTATUS, sta);

  return sta;	
}


//void Set_Rxmod()
//{
//  CEOUT &=~CEBIT;  
////  SPI_writeReg(WRITE_REG+0x07,0x70);
//  SPI_writeReg(WRITE_REG+CONFIG, 0x0f);   
//  CEOUT |= CEBIT;
//  delay_us(130);
//}


//uchar Rx_read(uchar *p)
//{
//  uchar d;
//  d = SPI_readReg(STATUS);
//  if(0x40==(d&0x40))
//  {
//    CEOUT &=~CEBIT;
//    SPI_readBuf(RD_RX_PLOAD,p,TX_PLOAD_WIDTH);
//  }
//  SPI_writeReg(WRITE_REG+STATUS,d);
//  
//  if(0x40==(d&0x40)){
//    Set_Rxmod();
//    return 1;
//  }
//  else
//    return 0;
//}
//
//uint Tx_send(uchar *tx_buf)
//{
//  uchar d = 0;uint i = 0;
//  CEOUT &=~CEBIT;	 	
//  SPI_writeBuf(WRITE_REG+RX_ADDR_P0,TX_ADDRESS,TX_ADR_WIDTH);
//  SPI_writeBuf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
//  SPI_writeReg(WRITE_REG+CONFIG,0x0e);
//  CEOUT |= CEBIT;
//  delay_us(20);  
//  while((!(d&TX_DS))&&(i<1000)){
//    d = SPI_readReg(STATUS);
//    if(d & 0x01)	//TX FIFO FULL
//    {
//       SPI_rwReg(FLUSH_TX,0xff);
//    }
//    SPI_rwReg(WRITE_REG + NRFRegSTATUS, d);
//    i++;//delay_us(1);
//  }
//  Set_Rxmod();
//  if(i<1000)
//    return 0;
//  else
//    return 1;
//}
//
//
//void NRF_TxPacket(uchar * tx_buf)
//{	
//	SPI_CE_L();		 //StandBy I模式
//	SPI_writeBuf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址	
//	SPI_writeBuf(WR_TX_PLOAD, tx_buf, 32); 			 // 装载数据
//	SPI_CE_H();	 //置高CE
//}

//void Nrf24l01_Init(u8 model)
//{
//  UCA_SPIInit();
//  delay_us(200);
//  CEDIR |= CEBIT;
//  CEOUT &=~CEBIT;
////        IQRIE |= IQRBIT;
////        IQRIES |= IQRBIT;
//        
//	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址 
//	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址  
//	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 
//	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 
////	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//设置自动重发间隔时间:500us;最大自动重发次数:10次 
//	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,0);														//设置RF通道为CHANAL
//	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
////	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												//设置TX发射参数,0db增益,1Mbps,低噪声增益开启
///////////////////////////////////////////////////////////
//	if(model==1)				//RX
//	{
//		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
//		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
//	}
//	else if(model==2)		//TX
//	{
//		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
//		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
//	}
//	else if(model==3)		//RX2
//	{
//		NRF_Write_Reg(FLUSH_TX,0xff);
//		NRF_Write_Reg(FLUSH_RX,0xff);
//		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
//		
//		Spi_RW(0x50);
//		Spi_RW(0x73);
//		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
//		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
//	}
//	else								//TX2
//	{
//		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
//		NRF_Write_Reg(FLUSH_TX,0xff);
//		NRF_Write_Reg(FLUSH_RX,0xff);
//		
//		Spi_RW(0x50);
//		Spi_RW(0x73);
//		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
//		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
//	}
//	SPI_CE_H();
//}



/////////////////////////////////////
//uint scop_quat(uchar *rxbuf,float *quat)//,uint a,uint b
//{
//  uint crct = 0;//,i=0;
//  rxbuf[0] = 0x55;
//  rxbuf[1] = 0xAA;
//  rxbuf[2] = 20;
//  rxbuf[3] = 0;
//  rxbuf[4] = 1;
//  rxbuf[5] = BYTE0(quat[0]);
//  rxbuf[6] = BYTE1(quat[0]); 
//  rxbuf[7] = BYTE2(quat[0]);
//  rxbuf[8] = BYTE3(quat[0]);
//  rxbuf[9] = BYTE0(quat[1]);
//  rxbuf[10] = BYTE1(quat[1]); 
//  rxbuf[11] = BYTE2(quat[1]);
//  rxbuf[12] = BYTE3(quat[1]);
//  rxbuf[13] = BYTE0(quat[2]);
//  rxbuf[14] = BYTE1(quat[2]); 
//  rxbuf[15] = BYTE2(quat[2]);
//  rxbuf[16] = BYTE3(quat[2]);
//  rxbuf[17] = BYTE0(quat[3]);
//  rxbuf[18] = BYTE1(quat[3]); 
//  rxbuf[19] = BYTE2(quat[3]);
//  rxbuf[20] = BYTE3(quat[3]);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,18);
//  rxbuf[21] = BYTE1(crct);
//  rxbuf[22] = BYTE0(crct);
////  rxbuf[23] = a/100+ 48;
////  rxbuf[24] = a%100/10+ 48;
////  rxbuf[25] = a%10+ 48;
////  rxbuf[26] = ',';
////  rxbuf[27] = b/100+ 48;
////  rxbuf[28] = b%100/10+ 48;
////  rxbuf[29] = b%10+ 48;
////  rxbuf[30] = ',';
////  rxbuf[31] = '\n';
//  return 0;
//}
//
//uint scop_RAM(uchar *rxbuf,uint n,float f,float f1,float f2)//,uint a,uint b
//{
//  uint crct = 0,i=0;
//  rxbuf[i++] = 0x55;
//  rxbuf[i++] = 0xAA;
//  rxbuf[i++] = 18;
//  rxbuf[i++] = 0;
//  rxbuf[i++] = 0x1d;
//  rxbuf[i++] = 0x12;
//  rxbuf[i++] = n;
//  rxbuf[i++] = BYTE0(f);
//  rxbuf[i++] = BYTE1(f); 
//  rxbuf[i++] = BYTE2(f);
//  rxbuf[i++] = BYTE3(f);
//  rxbuf[i++] = BYTE0(f1);
//  rxbuf[i++] = BYTE1(f1); 
//  rxbuf[i++] = BYTE2(f1);
//  rxbuf[i++] = BYTE3(f1);
//  rxbuf[i++] = BYTE0(f2);
//  rxbuf[i++] = BYTE1(f2); 
//  rxbuf[i++] = BYTE2(f2);
//  rxbuf[i++] = BYTE3(f2);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,16);
//  rxbuf[i++] = BYTE1(crct);
//  rxbuf[i++] = BYTE0(crct);
//  return 0;
//}
//
//uint scop_point(uchar *rxbuf,float f,float f1,float f2,float f3)
//{
//  uint crct = 0,i=0;
//  rxbuf[i++] = 0x55;
//  rxbuf[i++] = 0xAA;
//  rxbuf[i++] = 22;
//  rxbuf[i++] = 0;
//  rxbuf[i++] = 0x1d;
//  rxbuf[i++] = 0x12;
//  rxbuf[i++] = 19;
//  rxbuf[i++] = BYTE0(f);
//  rxbuf[i++] = BYTE1(f); 
//  rxbuf[i++] = BYTE2(f);
//  rxbuf[i++] = BYTE3(f);
//  rxbuf[i++] = BYTE0(f1);
//  rxbuf[i++] = BYTE1(f1); 
//  rxbuf[i++] = BYTE2(f1);
//  rxbuf[i++] = BYTE3(f1);
//  rxbuf[i++] = BYTE0(f2);
//  rxbuf[i++] = BYTE1(f2); 
//  rxbuf[i++] = BYTE2(f2);
//  rxbuf[i++] = BYTE3(f2);
//  rxbuf[i++] = BYTE0(f3);
//  rxbuf[i++] = BYTE1(f3); 
//  rxbuf[i++] = BYTE2(f3);
//  rxbuf[i++] = BYTE3(f3);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,20);
//  rxbuf[i++] = BYTE1(crct);
//  rxbuf[i++] = BYTE0(crct);
//  return 0;
//}
//
//
//uint math_crc16(const void * data,uint16_t len)//
//{
//    const static uint16_t crc_tab[16] =
//    {
//        0x0000 , 0x1021 , 0x2042 , 0x3063 , 0x4084 , 0x50A5 , 0x60C6 , 0x70E7 ,
//        0x8108 , 0x9129 , 0xA14A , 0xB16B , 0xC18C , 0xD1AD , 0xE1CE , 0xF1EF
//    };
//    uint8_t h_crc;
//	uint16_t crc = 0;
//    const uint8_t * ptr = (const uint8_t *)data;
//    //
//    while(len --)
//    {
//        h_crc = (uint8_t)(crc >> 12);
//        crc <<= 4;
//        crc ^= crc_tab[h_crc ^ ((*ptr) >> 4)];
//        //
//        h_crc = crc >> 12;
//        crc <<= 4;
//        crc ^= crc_tab[h_crc ^ ((*ptr) & 0x0F)];
//        //
//        ptr ++;
//    }
//    //
//    return crc;
//}
//
//int com_serial_transmit(uchar *com_serial_transmitBuffer, uint8_t len)//uint8_t PROTOCOL, uint8_t packetType,
//{
////	uchar com_serial_transmitBuffer[5];
//    uint16_t com_serial_transmitCrc = 0; 
//    
//    com_serial_transmitCrc = math_crc16(com_serial_transmitBuffer,len);
//
//    USART1_SendChar(0x55);
//    USART1_SendChar(0xAA);
//    USART1_SendChar(len+2);
////	USART1_SendChar(PROTOCOL);
////	USART1_SendChar(packetType);
//    USART1_SendDatas(com_serial_transmitBuffer,len);
//    USART1_SendChar(BYTE1(com_serial_transmitCrc));
//    USART1_SendChar(BYTE0(com_serial_transmitCrc));
//
//    return 0;
//}






