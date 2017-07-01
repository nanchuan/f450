
#ifndef __I2C_H
#define __I2C_H

#include "msp430init.h"


#define    I2CDIR P9DIR		        //IIC���Ŷ���	
#define    I2COUT P9OUT			
#define    I2CIN  P9IN
#define    I2CSCL BIT2
#define    I2CSDA BIT1
//#define SlaveAddress   0xD0	        //����������IIC�����еĴӵ�ַ


#define    sclout I2CDIR |= I2CSCL
#define    sdaout I2CDIR |= I2CSDA
#define    sdain  I2CDIR &=~I2CSDA
#define    sda    (I2CIN&I2CSDA)
#define    sda_1  I2COUT |= I2CSDA
#define    sda_0  I2COUT &=~I2CSDA
#define    scl_1  I2COUT |= I2CSCL
#define    scl_0  I2COUT &=~I2CSCL
 


//I2C��ʼ�ź�
//**************************************
void I2C_Start();

//I2Cֹͣ�ź�
//**************************************
void I2C_Stop();

//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack);

//I2C����Ӧ���ź�
//**************************************
uchar I2C_RecvACK();

//��I2C���߷���һ���ֽ�����
//**************************************
uchar I2C_SendByte(uchar dat);

//��I2C���߽���һ���ֽ�����
//**************************************
uchar I2C_RecvByte();

//��I2C�豸д��һ���ֽ�����
//**************************************
uchar Single_WriteI2C(uchar SlaveAddress,uchar REG_Address,uchar REG_data);

//��I2C�豸��ȡһ���ֽ�����
//**************************************
uchar Single_ReadI2C(uchar SlaveAddress,uchar REG_Address);

//�����������ݣ�
//
//*********************************************************
uchar Multiple_readI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n);

//����д�����ݣ�
//
//*********************************************************
uchar Multiple_writeI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n);





#endif

