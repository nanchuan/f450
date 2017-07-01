#include "NRF24L01.h"
  
uchar TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x44,0x55,0x01};	//���ص�ַ
uchar RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x44,0x55,0x01};	//���յ�ַ 

//��ʼ�� IRQ�շ�����жϿ���,16λCRC,������
/******************************************************************************/
void Nrf_Init()
{
  UCA_SPIInit();
  delay_us(200);
  IQRIE |= IQRBIT;
  IQRIES |= IQRBIT;
  CEDIR |= CEBIT;
  CEOUT &=~CEBIT;      //ce = 0 
  
  SPI_writeBuf(WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);     // д���ص�ַ	
  SPI_writeBuf(WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);  // д���ն˵�ַ
  SPI_writeReg(WRITE_REG+EN_AA,0x01);	        //  Ƶ��0�Զ�ACKӦ������
  SPI_writeReg(WRITE_REG+EN_RXADDR,0x01);       //  ������յ�ַֻ��Ƶ��0
  SPI_writeReg(WRITE_REG+SETUP_RETR,0x1a);      //�����Զ��ط����ʱ��:500us;����Զ��ط�����:10��
  SPI_writeReg(WRITE_REG+RF_CH,40);             //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
//  SPI_writeReg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	        //���ý������ݳ��ȣ�
  SPI_writeReg(WRITE_REG+RF_SETUP,0x0f);        //���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB
  
  
  SPI_writeReg(WRITE_REG + CONFIG, 0x0e);       // IRQ�շ�����жϿ���,16λCRC,������
  SPI_writeReg(FLUSH_TX,0xff);
  SPI_writeReg(FLUSH_RX,0xff);
  
  Spi_RW(0x50);
  Spi_RW(0x73);
  SPI_writeReg(WRITE_REG+0x1c,0x01);
  SPI_writeReg(WRITE_REG+0x1d,0x07);
  
  CEOUT |= CEBIT;
}

//״̬���,����
/******************************************************************************/
uchar Nrf_Check_Event(uchar * NRF24L01_RXDATA)
{
  uchar sta;
  
  sta = SPI_read(READ_REG + NRFRegSTATUS);
  if(sta & (RX_DR))     //�����ж�
  {
    if(SPI_read(R_RX_PL_WID)<33)
    {
      SPI_readBuf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);  // read receive payload from RX_FIFO buffer
    }
    else 
    {
      SPI_rwReg(FLUSH_RX,0xff); //��ջ�����
    }
  }
  if(sta & (TX_DS))
  {
            //PC_Debug_Show(1,2);
  }
  if(sta & (MAX_RT))
  {
    if(sta & 0x01)	        //TX FIFO FULL
    {
      SPI_rwReg(FLUSH_TX,0xff);
    }
  }
  SPI_rwReg(WRITE_REG + NRFRegSTATUS, sta);
  
  return sta;	
}

//��������2
/******************************************************************************/
void NRF_TxPacket(uchar * tx_buf)
{	
  SPI_CE_L();		 //StandBy Iģʽ
  SPI_writeBuf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);   // װ�ؽ��ն˵�ַ	
  SPI_writeBuf(WR_TX_PLOAD, tx_buf,32); 			        // װ������
  SPI_CE_H();	        //�ø�CE
}



////����ģʽ����
///******************************************************************************/
//void Set_Rxmod()
//{
//  CEOUT &=~CEBIT;  
////  SPI_writeReg(WRITE_REG+0x07,0x70);
//  SPI_writeReg(WRITE_REG+CONFIG, 0x0f);   
//  CEOUT |= CEBIT;
//  delay_us(130);
//}
//
////��ȡ���ջ���
///******************************************************************************/
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
////��������
///******************************************************************************/
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

////��ʼ��2
///******************************************************************************/
//void Nrf24l01_Init(u8 model)
//{
//  UCA_SPIInit();
//  delay_us(200);
//  CEDIR |= CEBIT;
//  CEOUT &=~CEBIT;
////        IQRIE |= IQRBIT;
////        IQRIES |= IQRBIT;
//        
//  NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//дRX�ڵ��ַ 
//  NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);         //дTX�ڵ��ַ  
//  NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//ʹ��ͨ��0���Զ�Ӧ�� 
//  NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//ʹ��ͨ��0�Ľ��յ�ַ 
////	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
//  NRF_Write_Reg(NRF_WRITE_REG+RF_CH,0);														//����RFͨ��ΪCHANAL
//  NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//����TX�������,0db����,2Mbps,���������濪��
////	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												//����TX�������,0db����,1Mbps,���������濪��
///////////////////////////////////////////////////////////
//  if(model==1)		        //RX
//  {
//    NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
//    NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
//  }
//  else if(model==2)		//TX
//  {
//    NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
//    NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
//  }
//  else if(model==3)		//RX2
//  {
//    NRF_Write_Reg(FLUSH_TX,0xff);
//    NRF_Write_Reg(FLUSH_RX,0xff);
//    NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
//    
//    Spi_RW(0x50);
//    Spi_RW(0x73);
//    NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
//    NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
//  }
//  else		                //TX2
//  {
//    NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
//    NRF_Write_Reg(FLUSH_TX,0xff);
//    NRF_Write_Reg(FLUSH_RX,0xff);
//    
//    Spi_RW(0x50);
//    Spi_RW(0x73);
//    NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
//    NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
//  }
//  SPI_CE_H();
//}

////��������apģʽ
///******************************************************************************/
//void NRF_TxPacket_AP(uchar * tx_buf)
//{	
//  SPI_CE_L();		 //StandBy Iģʽ	
//  SPI_writeBuf(0xa8, tx_buf, 32); 			 // װ������
//  SPI_CE_H();		 //�ø�CE
//}













