#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "UCA_SPI.h"

#define TX_ADR_WIDTH    5   
#define RX_ADR_WIDTH    5   
#define TX_PLOAD_WIDTH  32  
#define RX_PLOAD_WIDTH  32
#define READ_REG        0x00  // 读寄存器指令
#define WRITE_REG       0x20  // 写寄存器指令
#define RD_RX_PLOAD     0x61  // 读取接收数据指令
#define WR_TX_PLOAD     0xA0  // 写待发数据指令
#define FLUSH_TX        0xE1  // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  // 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  // 定义重复装载数据指令
#define NOP             0xFF  // 保留
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

#define NRFRegSTATUS    0x07  // 状态寄存器
#define RX_DR	        0x40  //中断标志
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