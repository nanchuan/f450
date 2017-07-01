
#ifndef __I2C_H
#define __I2C_H

#include "msp430init.h"


#define    I2CDIR P9DIR		        //IIC引脚定义	
#define    I2COUT P9OUT			
#define    I2CIN  P9IN
#define    I2CSCL BIT2
#define    I2CSDA BIT1
//#define SlaveAddress   0xD0	        //定义器件在IIC总线中的从地址


#define    sclout I2CDIR |= I2CSCL
#define    sdaout I2CDIR |= I2CSDA
#define    sdain  I2CDIR &=~I2CSDA
#define    sda    (I2CIN&I2CSDA)
#define    sda_1  I2COUT |= I2CSDA
#define    sda_0  I2COUT &=~I2CSDA
#define    scl_1  I2COUT |= I2CSCL
#define    scl_0  I2COUT &=~I2CSCL
 


//I2C起始信号
//**************************************
void I2C_Start();

//I2C停止信号
//**************************************
void I2C_Stop();

//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack);

//I2C接收应答信号
//**************************************
uchar I2C_RecvACK();

//向I2C总线发送一个字节数据
//**************************************
uchar I2C_SendByte(uchar dat);

//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte();

//向I2C设备写入一个字节数据
//**************************************
uchar Single_WriteI2C(uchar SlaveAddress,uchar REG_Address,uchar REG_data);

//从I2C设备读取一个字节数据
//**************************************
uchar Single_ReadI2C(uchar SlaveAddress,uchar REG_Address);

//连续读出数据，
//
//*********************************************************
uchar Multiple_readI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n);

//连续写入数据，
//
//*********************************************************
uchar Multiple_writeI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n);





#endif

