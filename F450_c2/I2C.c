#include "I2C.h"

//I2C起始信号
//**************************************
void I2C_Start()
{
  sdaout;sclout;    
  sda_1;                        //拉高数据线
  scl_1;                        //拉高时钟线
  delay_us(2);                  //延时
  sda_0;                        //产生下降沿
  delay_us(2);                  //延时
  scl_0;                        //拉低时钟线
}

//I2C停止信号
//**************************************
void I2C_Stop()
{
  sdaout;sclout;  
  sda_0;                        //拉低数据线
  scl_1;                        //拉高时钟线
  delay_us(2);                  //延时
  sda_1;                        //产生上升沿
  delay_us(2);                  //延时
}

//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack)
{
  sdaout;sclout;    
  if(ack)  sda_1;               //写应答信号
  else  sda_0;
  scl_1;                        //拉高时钟线
  delay_us(2);                  //延时
  scl_0;                        //拉低时钟线
  delay_us(2);                  //延时
}

//I2C接收应答信号
//**************************************
uchar I2C_RecvACK()
{
  uchar ack;
  sclout;sdain;  
  scl_1;                        //拉高时钟线
  delay_us(2);                  //延时
  ack = sda;                    //读应答信号
  scl_0;                        //拉低时钟线
  delay_us(2);                  //延时
  return ack;
}

//向I2C总线发送一个字节数据
//**************************************
uchar I2C_SendByte(uchar dat)
{
    uchar i;
    sdaout;sclout;
    for (i=0; i<8; i++)         //8位计数器
    {        
        if(dat&BIT7) sda_1;     //送数据口
        else    sda_0;
        scl_1;                  //拉高时钟线
        delay_us(2);            //延时
        scl_0;                  //拉低时钟线
        delay_us(2);            //延时
        dat <<= 1;              //移出数据的最高位
    }
    return I2C_RecvACK();
}

//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    sdain;sclout;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        scl_1;                  //拉高时钟线
        delay_us(2);            //延时
        if(sda)
        dat |= 0x01;            //读数据               
        scl_0;                  //拉低时钟线
        delay_us(2);            //延时
    }
    return dat;
}

//向I2C设备写入一个字节数据
//**************************************
uchar Single_WriteI2C(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
      I2C_Start();                      //起始信号
      if(I2C_SendByte(SlaveAddress))
        return 1;                       //发送设备地址+写信号
      if(I2C_SendByte(REG_Address))
        return 1;                       //内部寄存器地址，
      if(I2C_SendByte(REG_data))
        return 1;                       //内部寄存器数据，
      I2C_Stop();                       //发送停止信号
      return 0;
}

//从I2C设备读取一个字节数据
//**************************************
uchar Single_ReadI2C(uchar SlaveAddress,uchar REG_Address)
{
      uchar REG_data;
      I2C_Start();                      //起始信号
      if(I2C_SendByte(SlaveAddress))
        return 1;                       //发送设备地址+写信号
      if(I2C_SendByte(REG_Address))
        return 1;                       //发送存储单元地址，从0开始	
      I2C_Start();                      //起始信号
      if(I2C_SendByte(SlaveAddress+1))
        return 1;                       //发送设备地址+读信号
      REG_data=I2C_RecvByte();          //读出寄存器数据
      I2C_SendACK(1);                   //接收应答信号
      I2C_Stop();                       //停止信号
      return REG_data;
}

//连续读出数据，
//*********************************************************
uchar Multiple_readI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n)
{   
    uchar i;
    I2C_Start();                        //起始信号
    if(I2C_SendByte(SlaveAddress))
      return 1;                         //发送设备地址+写信号
    if(I2C_SendByte(REG_Address))
      return 1;                         //发送存储单元地址，从REG_Address开始	
    I2C_Start();                        //起始信号
    if(I2C_SendByte(SlaveAddress+1))
      return 1;                         //发送设备地址+读信号
    for (i=0; i<n; i++)                 //连续读取n个地址数据，存储中BUF
    {
      if(i) I2C_SendACK(0);             //回应ACK   
      buf[i] = I2C_RecvByte();          //BUF[0]存储0x32地址中的数据               
    }
    I2C_SendACK(1);                     //最后一个数据需要回NOACK
    I2C_Stop();                         //停止信号
//    delay_ms(5);
    return 0;
}

//连续写入数据，
//*********************************************************
uchar Multiple_writeI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n)
{
  uchar i;
  I2C_Start();                          //起始信号
  if(I2C_SendByte(SlaveAddress))
    return 1;                           //发送设备地址+写信号
  if(I2C_SendByte(REG_Address))
    return 1;                           //内部寄存器地址，  
  for(i=0;i<n;i++)
  {
    if(I2C_SendByte(buf[i]))
      return 1;                         //内部寄存器数据，
  }
  I2C_Stop();                           //发送停止信号
  return 0;
}



















