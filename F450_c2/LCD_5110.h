#ifndef __Lcd_5110_H
#define __Lcd_5110_H

//�˿�����
#define rstbit BIT4
#define cebit BIT3
#define dcbit BIT2
#define dinbit BIT1
#define clkbit BIT0
//#define lightbit BIT0

//msp430�˿ڶ���
#define rstout P8DIR |= rstbit
#define rst_1  P8OUT |= rstbit
#define rst_0  P8OUT &=~rstbit
#define ceout  P8DIR |= cebit
#define ce_1   P8OUT |= cebit
#define ce_0   P8OUT &=~cebit
#define dcout  P8DIR |= dcbit
#define dc_1   P8OUT |= dcbit
#define dc_0   P8OUT &=~dcbit
#define dinout P8DIR |= dinbit
#define din_1  P8OUT |= dinbit
#define din_0  P8OUT &=~dinbit
#define clkout P8DIR |= clkbit
#define clk_1  P8OUT |= clkbit
#define clk_0  P8OUT &=~clkbit
/*#define lightout P10DIR |= lightbit
#define light_1  P10OUT |= lightbit
#define light_0  P10OUT &=~lightbit*/

//�ֽ���ʾ��
#define l1 0
#define l2 6
#define l3 12
#define l4 18
#define l5 24
#define l6 30
#define l7 36
#define l8 42
#define l9 48
#define l10 54
#define l11 60
#define l12 66
#define l13 72
#define l14 78

#include "msp430init.h"

void Lcd_Write_Byte(uchar dt,uchar com);//

void Lcd_Set_xy(uint x,uint y);//0-5,0-84

void Lcd_Clear();

void Lcd_Char(uint x,uint y,uchar c);//��ʾ

void Lcd_Num(uint x,uint y,int num);//��ʾ���֣�int����

void Lcd_Pic(uint x,uint y,uint w,uint h,uchar *p);//ͼƬ

void Lcd_String(uint x,uint y,uchar *s);//�ַ���

void Lcd_Chinese(uint x,uint y,uchar c);//����

void Lcd_Init();//��ʼ��

void Lcd_Num_0x(uint x,uint y,int num);//���֣�16����

//void Lcd_numn(uint x,uint y,int m,uint num);

void Lcd_Numh(uint x,uint y,uint num);//���֣�char����

void Lcd_Num_b(uint x,uint y,uchar num);//���֣�2���ƣ�char����

void RTC_Lcd(void);//ʵʱʱ��

void Lcd_bar(uint x,uint y,int n);

#endif



