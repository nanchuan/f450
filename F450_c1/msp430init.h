
#ifndef __MSP430INIT_H
#define __MSP430INIT_H


#include <msp430f5438a.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CPU_F ((unsigned long)20)               //定义CPU 的时钟频率为25M
#define delay_us(x) __delay_cycles((CPU_F*(unsigned int)x))       //定义延时1us 
#define delay_ms(x) __delay_cycles((CPU_F*(unsigned int)x*1000))  //定义延时1ms

#define uchar unsigned char  // 定义无符号字符型变量
#define uint  unsigned int   // 定义无符号整型变量

//#define uint8_t uchar  
//#define uint16_t uint
//#define int16_t  int
//#define int8_t  char
//#define int32_t long int 
//#define uint32_t unsigned long int 

#define BYTE uchar      //字节变量
#define WORD uint       //字变量
#define u8 uchar

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))  //高八位

//KEY
#define Key_Init() P1DIR &=~(BIT7);P1REN |= (BIT7);P1OUT |= (BIT7)/*按键初始化*/
#define Key_IE() P1IE |= (BIT7);P1IES |= (BIT7)        //按键中断使能
#define key1in (P1IN&key1)         //
//#define key2in (P1IN&BIT6)
//#define key3in (P1IN&BIT4)
//#define key4in (P1IN&BIT2)
#define key1 BIT7
//#define key2 BIT6
//#define key3 BIT4
//#define key4 BIT2

//LED
#define Led_Init() P6DIR |= (BIT4+ BIT6);P7DIR |= (BIT4+ BIT6);Led1_1();\
  Led2_1();Led3_1();Led4_1()    //LED初始化
#define Led1_1()  P6OUT |= BIT4 //LED拉高
#define Led2_1()  P6OUT |= BIT6
#define Led3_1()  P7OUT |= BIT4
#define Led4_1()  P7OUT |= BIT6
#define Led1_0()  P6OUT &=~BIT4 //LED拉低
#define Led2_0()  P6OUT &=~BIT6
#define Led3_0()  P7OUT &=~BIT4
#define Led4_0()  P7OUT &=~BIT6
#define Led1_01()  P6OUT ^= BIT4        //亮灭交替
#define Led2_01()  P6OUT ^= BIT6
#define Led3_01()  P7OUT ^= BIT4
#define Led4_01()  P7OUT ^= BIT6

////红外
//#define tcrc1 BIT1
//#define tcrc2 BIT2
//#define tcrc3 BIT3
//#define tcrc4 BIT4
//#define tcrc P2IN
//#define tcrcdir P2DIR
//#define tcrcie P2IE
//#define tcrcies P2IES
//#define tcrc1in (tcrc&tcrc1)
//#define tcrc2in (tcrc&tcrc2)
//#define tcrc3in (tcrc&tcrc3)
//#define tcrc4in (tcrc&tcrc4)

////红外遥控
//#define irc BIT0
//#define Ircin() P1DIR &=~irc
//#define irc_i (P1IN & irc)
//#define Irc_Start() P1DIR &=~(irc);P1OUT |= (irc);P1REN |= irc;P1IES |= irc;P1IFG &=~(irc);P1IE |= irc
//typedef uchar IR_Buf[5];
//
////bell
//#define Bell_Init() P5DIR |= bell;P5OUT &=~bell
//#define bell BIT4

//int mark
/*#define UART1 (uchar)254
#define TIMERA0 (uchar)253
#define TIMERA1 (uchar)252
#define PORT2_0 (uchar)251
#define PORT1_3 (uchar)250
#define PORT1_0 (uchar)249
#define PORT1_6 (uchar)248
#define PORT1_4 (uchar)247
#define PORT1_2 (uchar)246
#define ADC12_6 (uchar)245
#define RTC_2 (uchar)244
#define TIMERB0_1 (uchar)243
#define TIMERB0_3 (uchar)242
#define WDT_TIMER (uchar)241

#define My_Lpm0Exit(); if(0==sta){LPM0_EXIT;} */

//#include "UCB_I2C.h"
#include "I2C.h"

#define min(a,b) ((a<b)?a:b)    //

#define Flash_ptr (int *)0x1880 //flash读写地址
 
#define Pwm_max (Pwm_min+ 1000) //PWM最大值
#define Pwm_min 1500            //
    
    
    
void Scop_update(uchar *tx,int roll,int pitch,int yaw,int high,
                 int THROT,int ROLL,int PITCH,int YAW,
                   uchar pwm1,uchar pwm2,uchar pwm3,uchar pwm4,
                   uchar avin,int lock,int dat1,
                   int dat2,int dat3,int dat4);
void PID_update(uchar *tx,uchar p1,uchar i1,uchar d1,
                uchar p2,uchar i2,uchar d2,
                uchar p3,uchar i3,uchar d3);

void Flash_read(int *p,int *fp,uchar n);

void Flash_write(int *p,int *fp,uchar n);

void WDT_Init();

void Clock_Init(void);

void Pwm_Init();

void Timera0_Init(uint t);

void Timera1_Init(uint t);

void Uart1_Init();

uchar Uart1_Send_Char(uchar c);

#define USART1_SendChar(a) Uart1_Send_Char(a)

void USART1_SendDatas(uchar *com_serial_transmitBuffer,uchar len);

void Uart1_Send_String(uchar *s);

void Adc12_Init();

#define Adc12_Start() ADC12CTL0 |= ADC12SC

void Pwm_Com(int v1,int v2,int v3,int v4);

void Clock_Init_DCO(void);

//typedef struct  //PID计算
//{
//  int derr1;
//  int derr2;
//  long int pst;
////  int pidout;
//}PID;//PID
//
//typedef struct  //PID参数
//{
//  int KP;
//  int KI;
//  int KD;
////  int KL;  
////  int MAX,MIN;
//}PID_C;//PID_c
//
//void PID_Init(PID_C *pid_c,int kp,int ki,int kd); 
//
//void PID_Clear(PID *pid);
//
//int PID_Cclt(PID *pid,PID_C *pid_c,int point,int fb);

//typedef struct
//{
//  long int x0,x1;
//  long int p0,p1;
//  int k;
//  int Q,R;
//}Kalman_Struct;
//
//int Kalman_Cclt(Kalman_Struct *kalman,int u,int z);
//
//void Kalman_Init(Kalman_Struct *kalman,int q,int r);
//void List_Choise(int *a,int n);

//void Uart1_Send_Numh(uchar num);
//
//void Uart1_Send_Num(int num);
//
//void ANO_scop(int *scop_data, int n);
//
//uint ANO_scopdatAF(uchar *tx,int *acc,int *gryo,int roll,int pitch,int yaw,
//                   uchar count);
//
//uint ANO_scopdatAE(uchar *tx,int THROT,int YAW,int ROLL,int PITCH,int high,
//                   int lock,int pwm1,int pwm2,int pwm3,int pwm4,int avin);
//
//uint ANO_scopdatAC(uchar *tx,uint rol_p,uint rol_i,uint rol_d
//                   ,uint pit_p,uint pit_i,uint pit_d
//                     ,uint yaw_p,uint yaw_i,uint yaw_d);
//
//unsigned short CRC_Check(unsigned char *Buf, unsigned char CRC_CNT);
//
//void Digital_Scope(int *temp);

//uchar Pwm_Com(int l,int r);

//void Timerb0_Init(void);
//
//void RTC_Init(void);

//void RTC_Lcd(void);

//uint RTC_Read(uint addr);

//void Uart0_Init();

//uchar IR_Ctr(IR_Buf tcrcom);
//
//void Bell(int ni);

//void Sg90_Up(int d);
//
//void Sg90_Init(void);
//
//uint scop_RAM(uchar *rxbuf,uint n,float f,float f1,float f2);
//
//uint scop_point(uchar *rxbuf,float f,float f1,float f2,float f3);
//
//uint scop_quat(uchar *rxbuf,float *quat);
//
//int com_serial_transmit(uchar *com_serial_transmitBuffer, uint8_t len);
//
//uint math_crc16(const void * data,uint16_t len);



#endif



