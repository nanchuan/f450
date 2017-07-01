#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "UCA_SPI.h"

#define TX_ADR_WIDTH    5   
#define RX_ADR_WIDTH    5   
#define TX_PLOAD_WIDTH  32  
#define RX_PLOAD_WIDTH  32
#define READ_REG        0x00  // ���Ĵ���ָ��
#define WRITE_REG       0x20  // д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61  // ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  // д��������ָ��
#define FLUSH_TX        0xE1  // ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  // ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  // �����ظ�װ������ָ��
#define NOP             0xFF  // ����
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define RX_DR	        0x40  //�жϱ�־
#define TX_DS	        0x20
#define MAX_RT	        0x10
#define R_RX_PL_WID   	0x60

#define CEDIR P9DIR
#define CEOUT P9OUT
#define CEBIT BIT3

#define IQRDIR P2DIR
#define IQRIN P2IN
#define IQRIE P2IE
#define IQRIES P2IES
#define IQRBIT BIT0
#define IN_IQR (IQRIN&IQRBIT)


#define SPI_CE_L() CEOUT &=~CEBIT
#define SPI_writeBuf(a,b,c) SPI_writeBuf(a,b,c)
#define SPI_rwReg(a,b) SPI_writeReg(a,b)
#define NRF_Write_Buf(a,b,c) SPI_writeBuf(a,b,c) 
#define NRF_Write_Reg(a,b) SPI_rwReg(a,b)
#define SPI_read(a)SPI_readReg(a)
#define Spi_RW(a) SPI_rw(a)
#define SPI_CE_H() CEOUT |= CEBIT
#define NRF_WRITE_REG WRITE_REG


void Nrf_Init();

uchar Nrf_Check_Event(uchar *NRF24L01_RXDATA);

void NRF_TxPacket_AP(uchar * tx_buf);

//
//void Set_Rxmod();
//
//uchar Rx_read(uchar *p);
//
//uint Tx_send(uchar *tx_buf);
//
//void Nrf24l01_Init(u8 model);
//
//void NRF_TxPacket(uchar * tx_buf);
//


//uint scop_quat(uchar *rxbuf,float *quat);
//
//int com_serial_transmit(uchar *com_serial_transmitBuffer, uint8_t len);
//
//uint math_crc16(const void * data,uint16_t len);
//
//
//uint scop_RAM(uchar *rxbuf,uint n,float f,float f1,float f2);
//
//uint scop_point(uchar *rxbuf,float f,float f1,float f2,float f3);

#endif