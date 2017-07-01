#include "I2C.h"

//I2C��ʼ�ź�
//**************************************
void I2C_Start()
{
  sdaout;sclout;    
  sda_1;                        //����������
  scl_1;                        //����ʱ����
  delay_us(2);                  //��ʱ
  sda_0;                        //�����½���
  delay_us(2);                  //��ʱ
  scl_0;                        //����ʱ����
}

//I2Cֹͣ�ź�
//**************************************
void I2C_Stop()
{
  sdaout;sclout;  
  sda_0;                        //����������
  scl_1;                        //����ʱ����
  delay_us(2);                  //��ʱ
  sda_1;                        //����������
  delay_us(2);                  //��ʱ
}

//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack)
{
  sdaout;sclout;    
  if(ack)  sda_1;               //дӦ���ź�
  else  sda_0;
  scl_1;                        //����ʱ����
  delay_us(2);                  //��ʱ
  scl_0;                        //����ʱ����
  delay_us(2);                  //��ʱ
}

//I2C����Ӧ���ź�
//**************************************
uchar I2C_RecvACK()
{
  uchar ack;
  sclout;sdain;  
  scl_1;                        //����ʱ����
  delay_us(2);                  //��ʱ
  ack = sda;                    //��Ӧ���ź�
  scl_0;                        //����ʱ����
  delay_us(2);                  //��ʱ
  return ack;
}

//��I2C���߷���һ���ֽ�����
//**************************************
uchar I2C_SendByte(uchar dat)
{
    uchar i;
    sdaout;sclout;
    for (i=0; i<8; i++)         //8λ������
    {        
        if(dat&BIT7) sda_1;     //�����ݿ�
        else    sda_0;
        scl_1;                  //����ʱ����
        delay_us(2);            //��ʱ
        scl_0;                  //����ʱ����
        delay_us(2);            //��ʱ
        dat <<= 1;              //�Ƴ����ݵ����λ
    }
    return I2C_RecvACK();
}

//��I2C���߽���һ���ֽ�����
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    sdain;sclout;
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        scl_1;                  //����ʱ����
        delay_us(2);            //��ʱ
        if(sda)
        dat |= 0x01;            //������               
        scl_0;                  //����ʱ����
        delay_us(2);            //��ʱ
    }
    return dat;
}

//��I2C�豸д��һ���ֽ�����
//**************************************
uchar Single_WriteI2C(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
      I2C_Start();                      //��ʼ�ź�
      if(I2C_SendByte(SlaveAddress))
        return 1;                       //�����豸��ַ+д�ź�
      if(I2C_SendByte(REG_Address))
        return 1;                       //�ڲ��Ĵ�����ַ��
      if(I2C_SendByte(REG_data))
        return 1;                       //�ڲ��Ĵ������ݣ�
      I2C_Stop();                       //����ֹͣ�ź�
      return 0;
}

//��I2C�豸��ȡһ���ֽ�����
//**************************************
uchar Single_ReadI2C(uchar SlaveAddress,uchar REG_Address)
{
      uchar REG_data;
      I2C_Start();                      //��ʼ�ź�
      if(I2C_SendByte(SlaveAddress))
        return 1;                       //�����豸��ַ+д�ź�
      if(I2C_SendByte(REG_Address))
        return 1;                       //���ʹ洢��Ԫ��ַ����0��ʼ	
      I2C_Start();                      //��ʼ�ź�
      if(I2C_SendByte(SlaveAddress+1))
        return 1;                       //�����豸��ַ+���ź�
      REG_data=I2C_RecvByte();          //�����Ĵ�������
      I2C_SendACK(1);                   //����Ӧ���ź�
      I2C_Stop();                       //ֹͣ�ź�
      return REG_data;
}

//�����������ݣ�
//*********************************************************
uchar Multiple_readI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n)
{   
    uchar i;
    I2C_Start();                        //��ʼ�ź�
    if(I2C_SendByte(SlaveAddress))
      return 1;                         //�����豸��ַ+д�ź�
    if(I2C_SendByte(REG_Address))
      return 1;                         //���ʹ洢��Ԫ��ַ����REG_Address��ʼ	
    I2C_Start();                        //��ʼ�ź�
    if(I2C_SendByte(SlaveAddress+1))
      return 1;                         //�����豸��ַ+���ź�
    for (i=0; i<n; i++)                 //������ȡn����ַ���ݣ��洢��BUF
    {
      if(i) I2C_SendACK(0);             //��ӦACK   
      buf[i] = I2C_RecvByte();          //BUF[0]�洢0x32��ַ�е�����               
    }
    I2C_SendACK(1);                     //���һ��������Ҫ��NOACK
    I2C_Stop();                         //ֹͣ�ź�
//    delay_ms(5);
    return 0;
}

//����д�����ݣ�
//*********************************************************
uchar Multiple_writeI2C(uchar SlaveAddress,uchar *buf,uchar REG_Address,uchar n)
{
  uchar i;
  I2C_Start();                          //��ʼ�ź�
  if(I2C_SendByte(SlaveAddress))
    return 1;                           //�����豸��ַ+д�ź�
  if(I2C_SendByte(REG_Address))
    return 1;                           //�ڲ��Ĵ�����ַ��  
  for(i=0;i<n;i++)
  {
    if(I2C_SendByte(buf[i]))
      return 1;                         //�ڲ��Ĵ������ݣ�
  }
  I2C_Stop();                           //����ֹͣ�ź�
  return 0;
}



















