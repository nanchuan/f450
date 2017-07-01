#include "msp430init.h"


//
void Scop_update(uchar *tx,int roll,int pitch,int yaw,int high,
                 int THROT,int ROLL,int PITCH,int YAW,
                   uchar pwm1,uchar pwm2,uchar pwm3,uchar pwm4,
                   uchar avin,int lock,int dat1,
                   int dat2,int dat3,int dat4)
{
  uchar i = 0;
  
  tx[i++] = 0x88;               //0
  tx[i++] = BYTE1(roll);        //1
  tx[i++] = BYTE0(roll);        //2
  tx[i++] = BYTE1(pitch);       //3
  tx[i++] = BYTE0(pitch);       //4
  tx[i++] = BYTE1(yaw);         //5
  tx[i++] = BYTE0(yaw);
  tx[i++] = BYTE1(high);        //7
  tx[i++] = BYTE0(high);
  tx[i++] = BYTE1(THROT);       //9
  tx[i++] = BYTE0(THROT);
  tx[i++] = BYTE1(ROLL);        //11
  tx[i++] = BYTE0(ROLL);
  tx[i++] = BYTE1(PITCH);       //13
  tx[i++] = BYTE0(PITCH);
  tx[i++] = BYTE1(YAW);         //15
  tx[i++] = BYTE0(YAW);
  tx[i++] = pwm1;               //17
  tx[i++] = pwm2;
  tx[i++] = pwm3;
  tx[i++] = pwm4;
  tx[i++] = avin;               //21
  tx[i++] = BYTE1(lock);        //22
  tx[i++] = BYTE0(lock);
  tx[i++] = BYTE1(dat1);        //24
  tx[i++] = BYTE0(dat1);
  tx[i++] = BYTE1(dat2);        //26
  tx[i++] = BYTE0(dat2);
  tx[i++] = BYTE1(dat3);        //28
  tx[i++] = BYTE0(dat3);
  tx[i++] = BYTE1(dat4);        //30
  tx[i++] = BYTE0(dat4);
}

void Clock_Init(void)   //时钟初始化
{
  WDTCTL = WDTPW + WDTHOLD;     //关闭看门狗
 // P11DIR |= 0x07;             //时钟输出
 // P11SEL |= 0x07;
  __bis_SR_register(SCG0);      //close FLL
  
  P5SEL |= 0x0c;
  P7SEL |= 0x03;                //开晶振端口功能
  
  UCSCTL6 &= ~(XT1OFF+ XT2OFF); //开xt1,xt2
  UCSCTL6 |= XCAP_3;            //cap selection
  do
  {
    UCSCTL7 &= ~(XT2OFFG+ XT1LFOFFG+ XT1HFOFFG+ DCOFFG);
    SFRIFG1 &= ~OFIFG;
  }while (SFRIFG1&OFIFG);
  
  UCSCTL6 &= ~XT2DRIVE_2;               //20m晶振驱动
//  UCSCTL5 |= DIVS_0;                  //smclk
  UCSCTL4 |= SELS_5 + SELM_5;           //smclk:20m, alk:32768
}

void Clock_Init_DCO(void)
{
  WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
//  P1DIR |= BIT0;                          // P1.0 output
//  P11DIR |= 0x07;                         // ACLK, MCLK, SMCLK set out to pins
//  P11SEL |= 0x07;                         // P11.0,1,2 for debugging purposes.

    // Initialize LFXT1
  P7SEL |= 0x03;                            // Select XT1
  UCSCTL6 &= ~(XT1OFF);                     // XT1 On
  UCSCTL6 |= XCAP_3;                        // Internal load cap

  // Loop until XT1 fault flag is cleared
  do
  {
    UCSCTL7 &= ~XT1LFOFFG;                  // Clear XT1 fault flags
  }while (UCSCTL7&XT1LFOFFG);               // Test XT1 fault flag
  
  UCSCTL3 |= SELREF_0;                      // Set DCO FLL reference = REFO  
  UCSCTL4 |= SELA_0;                        // Set ACLK = 32k,smclk = 25m, mclk = 25m
  
  __bis_SR_register(SCG0);                  // Disable the FLL control loop
  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
  UCSCTL1 = DCORSEL_7;                      // Set RSELx for DCO = 25 MHz
  UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (762 + 1) * 32768 = 25MHz
                                            // Set FLL Div = fDCOCLK/2
  __bic_SR_register(SCG0);                  // Enable the FLL control loop  

  // Loop until XT1,XT2 & DCO fault flag is cleared
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
                                            // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag  
}



void Timera0_Init(uint t)//timera0初始化, init 0-65535 (us)
{
  TA0CTL |= TACLR;                       //clear
  TA0CTL |= TASSEL_2+ ID_2 + MC_1;      //+ TAIE     //smclk,4分频,up mode  
  TA0EX0 = TAIDEX_4;                    //5分频
  TA0CCR0 = t;                          //定时 t us
  TA0CCTL0 |= CCIE;                     //ta0.0 interrupt
}

void Timera1_Init(uint t)//timera1 init 0-65535 (us)
{
  TA1CTL |= TACLR;                      //clear
  TA1CTL |= TASSEL_2+ ID_2+ MC_1;       //+ TAIE   //smclk,4分频,up mode
  TA1EX0 = TAIDEX_4;                    //5分频
  TA1CCR0 = t;                          //定时 t us
  TA1CCTL0 |= CCIE;                     //ta1.0 interrupt
}


void Uart1_Init()//串口1初始化
{
//  UCA1CTL0 |= 0;
  UCA1CTL1 |= UCSWRST;                  //打开 可写 控制位
//  UCA1CTL1 = UCSSEL_1;                //smclk
//  UCA1BR0 = 0x03;
//  UCA1BR1 = 0;
//  UCA1MCTL = UCBRS_3+ UCBRF_0;        //波特率设置 9600
  UCA1CTL1 = UCSSEL_2;                  //smclk
  UCA1BR0 = 173;
  UCA1BR1 = 0;
  UCA1MCTL = UCBRS_5+ UCBRF_0;          //波特率设置 115200
//  P5DIR |= BIT6 + BIT7;
  P5REN |= BIT6 + BIT7;
  P5OUT |= BIT6 + BIT7;
  P5SEL |= BIT6 + BIT7;                 //端口第二功能
// UCA1STAT |=
// UCA1RXBUF =
// UCA1TXBUF =
// UCA1IRTCTL |=  
// UCA1IRRCTL |=
// UCA1ABCTL |=
  UCA1CTL1 &= ~UCSWRST;         //关闭可写控制
  UCA1IE |= UCRXIE;//+ UCTXIE   //开接收中断
//  UCA1IFG |= UCTXIFG;         //UCRXIFG+  
}

uchar Uart1_Send_Char(uchar c)  //串口1发送字符 c
{
  while(!(UCA1IFG&UCTXIFG));    //等待发送空闲
  UCA1TXBUF = c;
  UCA1IFG &= ~UCTXIFG;
  return c;
}

void Uart1_Send_String(uchar *s)//串口1发送数组 s
{
  while(*s)     //是否为空
  {
    Uart1_Send_Char(*s++);
  }
}

void USART1_SendDatas(uchar *com_serial_transmitBuffer,uchar len)
{
  int i;
  for(i=0;i<len;i++)
  {
      USART1_SendChar(*(com_serial_transmitBuffer + i));	
  }
}


//**************************************************************
// flash 写
// 原数据地址 p
// 目标地址 fp
// 数据大小 n
//**************************************************************
void Flash_write(int *p,int *fp,uchar n)
{
  uint gieStatus,i;
  gieStatus = __get_SR_register() & GIE;
  __disable_interrupt();
  FCTL3 = FWKEY;                        // Clear Lock bit
  FCTL1 = FWKEY+ERASE; 
  *(int *)fp = 0;
  FCTL1 = FWKEY+WRT;  
  for (i = 0; i < n; i++)
  {
    *fp++ = *p++;                       // Write value to flash
  }
  FCTL1 = FWKEY;                        // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                   // Set LOCK bit
  __bis_SR_register(gieStatus);
}

//*********************************************************
// flash 读
// 目标地址 p
// flash 数据地址 fp
// 数据大小 n
//*********************************************************
void Flash_read(int *p,int *fp,uchar n)
{
  uint i;
  for (i = 0; i < n; i++)
  {
    *p++ = *fp++;                 // Write value to flash
  }
}

//**********************************
// 看门狗定时
//**********************************
void WDT_Init(){
  WDTCTL = WDT_MDLY_32;  //****/20 = ***
//  SFRIE1 |= WDTIE;
}

void Adc12_Init()//ADC初始化
{
  P6SEL |= BIT4+ BIT5+ BIT6+ BIT7;P7SEL |= BIT4; 
  ADC12CTL0 &= ~ADC12ENC;                       //清除寄存器保护位
//  REFCTL0 |= REFMSTR+ REFVSEL_2+ REFON;       //INOR 2.5vREF
  ADC12CTL0 |= ADC12ON+ ADC12MSC+ ADC12SHT0_8+ ADC12SHT1_8;       // 
  ADC12CTL1 |= ADC12SHP+ ADC12CONSEQ_1;         //
  ADC12MCTL0 |= ADC12INCH_4;       //
  ADC12MCTL1 |= ADC12INCH_5;
  ADC12MCTL2 |= ADC12INCH_6;  
  ADC12MCTL3 |= ADC12INCH_7; 
  ADC12MCTL4 |= ADC12INCH_12+ ADC12EOS; 
  ADC12IE |= ADC12IE4;          //开中断  
  ADC12CTL0 |= ADC12ENC;        //开始保护
//  Adc12_Start();
}


void Bell_IE_start()    
{
  SFRIE1 |= WDTIE;
}

void Bell_IE()  
{
  P5OUT ^= bell;
}

void Bell_IE_end()      
{
  SFRIE1 &= ~WDTIE;
  P5OUT &= ~bell;
}

//********************************
// bell init 
//********************************
void Bell(int ni)       
{
  int n;
  for(n=0;n<ni;n++)
  {
    P5OUT |= bell;
    delay_us(100);
    P5OUT &= ~bell;
//    delay_us(200);
  }
}


////**********************************************
//// SG90舵机初始化,ta1 20ms up model
////**********************************************
//void Sg90_Init(void)
//{
//  P7DIR |= BIT3;
//  P7SEL |= BIT3;
//  TA1CCR2 = (uint)1480;//1.5ms 0‘
//  TA1CCTL2 |= OUTMOD_6;
//}
//
////******************************************
//// SG90舵机控制，-900~900
//// 输入:d,0.1/bit
////******************************************
//void Sg90_Up(int d)     
//{
//  if(d>900){
//    TA1CCR2 = (uint)2380;
//  }
//  else if(d<-900){
//    TA1CCR2 = (uint)580;
//  }
//  else{
//    TA1CCR2 = (uint)1480+ d;
//  }
//}
//


////*******************************************
//// 排序,选择法
//// 
////*******************************************
//void List_Choise(int *a,int n)  
//{
//  int i,j,k,temp;
//  for(i=0;i<n-1;i++){
//    k = i;
//    for(j=i+1;j<n;j++){
//      if(a[k]>a[j]){
//        k = j;
//      }
//    }
//    if(i!=k){
//      temp = a[i];
//      a[i] = a[k];
//      a[k] = temp;
//    }  
//  }
//}
//

//
//
////****************************************
//// 红外解码
////****************************************
//uchar IR_Ctr(IR_Buf ircom)
//{  
//  uint i,j,n=0;
//  Ircin();
//  delay_ms(1);
//  if(irc_i)
//  { 
//    return 0;
//  }
//  while(!irc_i)
//  {
//    delay_us(50);
//    n++;
//  }
//  if(n<100)
//  { 
//    return 0;
//  }
//  n=0;
//  while(irc_i)
//    delay_us(140);
//  for(j=0;j<4;j++)
//  {
//    for(i=0;i<8;i++)
//    {     
//      while(!irc_i)
//        delay_us(140); 
//      while(irc_i)
//      {
//        delay_us(140);
//        n++;
//        if(n>30)
//        {   
//          return 0;
//        }
//      }
//      ircom[j]>>=1;
//      if(n>7)
//        ircom[j] |= 0x80;
//      n=0;
//    }
//  }
////  delay_ms(200);
//  if((ircom[2]+ircom[3])!=255)
//  { 
//    return 0;
//  }
//  else
//  {
//    ircom[4] = ircom[2];
//    return 1;
//  }
//}


///////////////////////////////////////////////////////////
//void Kalman_Init(Kalman_Struct *kalman,int q,int r)
//{
//  kalman->Q = q;
//  kalman->R = r;
//}
//
//int Kalman_Cclt(Kalman_Struct *kalman,int u,int z)
//{
//  kalman->x1 = kalman->x0+ u;
//  kalman->p1 = kalman->p0+ kalman->Q;
//  
//  kalman->k = (kalman->p1*1000/(kalman->p1+ kalman->R));
//  kalman->x0 = (int)(kalman->x1+ (kalman->k*(z- kalman->x1))/1000);
//  kalman->p0 = (int)(((1000- kalman->k)*kalman->p1)/1000);
//  return kalman->x0;
//}

///**/
///////////////////////////////////////////////////////
//void PID_Init(PID_C *pid_c,int kp,int ki,int kd)//,int max,int min,int klpid init KP KI KD KL
//{
//  pid_c->KP = kp;
//  pid_c->KI = ki;
//  pid_c->KD = kd;
////  pid_c->KL = kl;  
////  pid_c->MAX = max;
////  pid_c->MIN = min;
////  pid->point = 0;
////  pid->pidout = 0;              //pid clear
////  pid->pst = 0;
////  pid->derr1 = 0;
////  pid->derr2 = 0;
//}
//
//void PID_Clear(PID *pid)//pid clear
//{
////  pid->pidout = 0;
//  pid->pst = 0;
//  pid->derr1 = 0;
//  pid->derr2 = 0;
//}
//
//int PID_Cclt(PID *pid,PID_C *pid_c,int point,int fb)//pid calculate point feedback
//{
//  long err,derr;
//  err = point- fb;                              //总误差
////  if(!(((pid->pst>=pid_c->MAX)&&(err>0))||((pid->pst<=pid_c->MIN)&&(err<0)))) 
//    pid->pst += err;                            //带线积分
//  derr = pid->derr1- pid->derr2;                //微分
//  pid->derr2 = pid->derr1;                      //
//  pid->derr1 = err;                             //
//  return (int)(((err*pid_c->KP/100+ pid->pst*pid_c->KI/1000+ derr*pid_c->KD)));//pid调节(int)
////  pid->pidout = (int)err;//((pid_c->KL*(long)pid->pidout+ (100- pid_c->KL)*err)/100);//低通，延长微分作用(int)
////  return((int)err);
//}
//
////*****************************************************************
//// 实时时钟初始化
////*****************************************************************
//void RTC_Init(void)
//{
//  RTCCTL01 |= RTCTEVIE+ RTCRDYIE+ RTCHOLD+ RTCMODE; //+ RTCBCD
//  //分钟事件，可读，保持时间，日历模式
///*  RTCSEC = 0;                 //秒
//  RTCMIN = 0;                   //分
//  RTCHOUR = 0;                  //时
//  RTCDOW = 1;                   //星期
//  RTCDAY = 1;                   //日
//  RTCMON = 1;                   //月
//  RTCYEARL = 12;                //
//  RTCYEARH = 20;*/              //年
//  //RTCAMIN,RTCAHOUR,RTCADOW,RTCDAY     //闹钟
//  //RTCPS1CTL |= RT1IP_6+ RT1PSIE;      //间隔事件
//  RTCCTL01 &= ~RTCHOLD;         //开始运行
//}
//
///*
//void RTC_Lcd(void)//时间显示
//{
////  while(!(RTCCTL1&RTCRDY));
////  for(;RTCCTL1&RTCRDY;)
////  Lcd_Char(36,5,RTCHOUR/10+48);
////  Lcd_Char(42,5,RTCHOUR%10+48);//时
////  Lcd_Char(48,5,':');
//  Lcd_Char(54,5,RTCMIN/10+48);
//  Lcd_Char(60,5,RTCMIN%10+48);//分
//  Lcd_Char(66,5,':');
//  Lcd_Char(l13,5,RTCSEC/10+48);
//  Lcd_Char(l14,5,RTCSEC%10+48);//秒
//}*/
////////////////////////////////////////////////////////////////////////////
//void Timerb0_Init(void)//timerb0 初始化，测量速度脉宽
//{
//  TB0CTL |= TBCLR;                //clear
//  TB0CTL |= TBSSEL_1+ ID_3 + MC_2;//+ TBIE ;     //aclk:32768,8分频,连续模式
////  TB0EX0 |= TBIDEX_1;                   //clk/2，共16分频
//  TB0CCTL1 |= CM_2+ CAP+ CCIE;          //下降沿捕获，开中断
//  TB0CCTL3 |= CM_2+ CAP+ CCIE;  
//  P4SEL |= BIT1+ BIT3;          //开端口第二功能
//  P4DIR &= ~(BIT1+ BIT3);       // +BIT2+ BIT4
////  P4OUT &= ~(BIT1+ BIT3);
////  P4REN |= (BIT1+ BIT3);
//}
///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//unsigned short CRC_Check(unsigned char *Buf, unsigned char CRC_CNT)//串口示波器CRC_CHECK生成
//{
//    unsigned short CRC_Temp;
//    unsigned char i,j;
//    CRC_Temp = 0xffff;
//
//    for (i=0;i<CRC_CNT; i++){      
//        CRC_Temp ^= Buf[i];
//        for (j=0;j<8;j++) {
//            if (CRC_Temp & 0x01)
//                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
//            else
//                CRC_Temp = CRC_Temp >> 1;
//        }
//    }
//    return(CRC_Temp);
//}
//
//void Digital_Scope(int *temp)//串口示波器数据帧编码
//{
//  unsigned char databuf[10] = {0};
//  unsigned char i;
//  unsigned short CRC16 = 0;
//  
////  for(i=0;i<4;i++)
////   { 
////     temp[i] = temp[i];    
////   }
//
//  for(i=0;i<4;i++) 
//  {
//    databuf[i*2]   = ((unsigned int)temp[i]%256);
//    databuf[i*2+1] = ((unsigned int)temp[i]/256);//(unsigned char)
//  }
//  
//  CRC16 = CRC_Check(databuf,8);
//  databuf[8] = CRC16%256;
//  databuf[9] = CRC16/256;
//  
//  for(i=0;i<10;i++)
//    Uart1_Send_Char(databuf[i]); 
//}
//
//void ANO_scop(int *scop_data, int n)
//{
//  uchar sum = 0,i = 0;
//  n *= 2;
//  sum += Uart1_Send_Char(0x88);
//  sum += Uart1_Send_Char(0xa1);
//  sum += Uart1_Send_Char(n);
//  while(i<n){
//    sum += Uart1_Send_Char((uint)scop_data[i]/256);
//    sum += Uart1_Send_Char((uchar)scop_data[i]);
//    i++;
//  }
//  Uart1_Send_Char(sum);
//}
//
//uint ANO_scopdatAF(uchar *tx,int *acc,int *gryo,int roll,int pitch,int yaw)
//{
//  uchar i = 0,sum = 0;
//  tx[i++] = 0x88;
//  sum += tx[i];
//  tx[i++] = 0xaf;
//  sum += tx[i];
//  tx[i++] = 0x1c;
//  sum += tx[i];
//  tx[i++] = BYTE1(acc[0]);
//  sum += tx[i];
//  tx[i++] = BYTE0(acc[0]);
//  sum += tx[i];
//  tx[i++] = BYTE1(acc[1]);
//  sum += tx[i];
//  tx[i++] = BYTE0(acc[1]);
//  sum += tx[i];
//  tx[i++] = BYTE1(acc[2]);
//  sum += tx[i];
//  tx[i++] = BYTE0(acc[2]);
//  sum += tx[i];
//  tx[i++] = BYTE1(gryo[0]);
//  sum += tx[i];
//  tx[i++] = BYTE0(gryo[0]);
//  sum += tx[i];
//  tx[i++] = BYTE1(gryo[1]);
//  sum += tx[i];
//  tx[i++] = BYTE0(gryo[1]);
//  sum += tx[i];
//  tx[i++] = BYTE1(gryo[2]);
//  sum += tx[i];
//  tx[i++] = BYTE0(gryo[2]);
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = BYTE1(roll);
//  sum += tx[i];
//  tx[i++] = BYTE0(roll);
//  sum += tx[i];
//  tx[i++] = BYTE1(pitch);
//  sum += tx[i];
//  tx[i++] = BYTE0(pitch);
//  sum += tx[i];
//  tx[i++] = BYTE1(yaw);
//  sum += tx[i];
//  tx[i++] = BYTE0(yaw);
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = sum;
//  return i;
//}
//
//uint ANO_scopdatAE(uchar *tx,int THROT,int YAW,int ROLL,int PITCH,
//                   int pwm1,int pwm2,int pwm3,int pwm4)
//{
//  uchar i = 0,sum = 0;
//  tx[i++] = 0x88;
//  sum += tx[i];
//  tx[i++] = 0xae;
//  sum += tx[i];
//  tx[i++] = 0x12;
//  sum += tx[i];
//  tx[i++] = BYTE1(THROT);
//  sum += tx[i];
//  tx[i++] = BYTE0(THROT);
//  sum += tx[i];
//  tx[i++] = BYTE1(YAW);
//  sum += tx[i];
//  tx[i++] = BYTE0(YAW);
//  sum += tx[i];
//  tx[i++] = BYTE1(ROLL);
//  sum += tx[i];
//  tx[i++] = BYTE0(ROLL);
//  sum += tx[i];
//  tx[i++] = BYTE1(PITCH);
//  sum += tx[i];
//  tx[i++] = BYTE0(PITCH);
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = BYTE1(pwm1);
//  sum += tx[i];
//  tx[i++] = BYTE0(pwm1);
//  sum += tx[i];
//  tx[i++] = BYTE1(pwm2);
//  sum += tx[i];
//  tx[i++] = BYTE0(pwm2);
//  sum += tx[i];
//  tx[i++] = BYTE1(pwm3);
//  sum += tx[i];
//  tx[i++] = BYTE0(pwm3);
//  sum += tx[i];
//  tx[i++] = BYTE1(pwm4);
//  sum += tx[i];
//  tx[i++] = BYTE0(pwm4);
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = sum;
//  return i;
//}
//
//uint ANO_scopdatAC(uchar *tx,uint rol_p,uint rol_i,uint rol_d
//                   ,uint pit_p,uint pit_i,uint pit_d
//                     ,uint yaw_p,uint yaw_i,uint yaw_d)
//{
//  uchar i = 0,sum = 0;
//  tx[i++] = 0x88;
//  sum += tx[i];
//  tx[i++] = 0xac;
//  sum += tx[i];
//  tx[i++] = 0x1c;
//  sum += tx[i];
//  tx[i++] = 0xad;
//  sum += tx[i];
//  tx[i++] = BYTE1(rol_p);
//  sum += tx[i];
//  tx[i++] = BYTE0(rol_p);
//  sum += tx[i];
//  tx[i++] = BYTE1(rol_i);
//  sum += tx[i];
//  tx[i++] = BYTE0(rol_i);
//  sum += tx[i];
//  tx[i++] = BYTE1(rol_d);
//  sum += tx[i];
//  tx[i++] = BYTE0(rol_d);
//  sum += tx[i];
//  tx[i++] = BYTE1(pit_p);
//  sum += tx[i];
//  tx[i++] = BYTE0(pit_p);
//  sum += tx[i];
//  tx[i++] = BYTE1(pit_i);
//  sum += tx[i];
//  tx[i++] = BYTE0(pit_i);
//  sum += tx[i];
//  tx[i++] = BYTE1(pit_d);
//  sum += tx[i];
//  tx[i++] = BYTE0(pit_d);
//  sum += tx[i];
//  tx[i++] = BYTE1(yaw_p);
//  sum += tx[i];
//  tx[i++] = BYTE0(yaw_p);
//  sum += tx[i];
//  tx[i++] = BYTE1(yaw_i);
//  sum += tx[i];
//  tx[i++] = BYTE0(yaw_i);
//  sum += tx[i];
//  tx[i++] = BYTE1(yaw_d);
//  sum += tx[i];
//  tx[i++] = BYTE0(yaw_d);
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = 0;
//  sum += tx[i];
//  tx[i++] = sum;
//  return i;
//}
//




//void Uart1_Send_Num(int num)
//{
//  if(num<0){
//    Uart1_Send_Char('-');
//    num = -num;
//  }
//  else{
//    Uart1_Send_Char(' ');
//  }
//  Uart1_Send_Char(num/10000+ 48);
//  Uart1_Send_Char(num%10000/1000+ 48);
//  Uart1_Send_Char(num%1000/100+ 48);
//  Uart1_Send_Char(num%100/10+ 48);
//  Uart1_Send_Char(num%10+ 48);
//  Uart1_Send_Char(' ');
//}
//
//void Uart1_Send_Numh(uchar num)
//{  
//  Uart1_Send_Char(num/100+ 48);
//  Uart1_Send_Char(num%100/10+ 48);
//  Uart1_Send_Char(num%10+ 48);
//  Uart1_Send_Char(' ');
//}



/////////////////////////////////////////////////////////////////////
/*
void Uart0_Init()//串口0初始化,做红外接收用
{
//  UCA0CTL0 |= 0;
  UCA0CTL1 |= UCSWRST;          //打开 可写 控制位
  UCA0CTL1 = UCSSEL_2;          //smclk
//  UCA0BR0 = 0x03;
//  UCA0BR1 = 0;
//  UCA0MCTL = UCBRS_3+ UCBRF_0;  //波特率设置 9600
  UCA0IRTCTL |= UCIRTXPL5+ UCIREN;//(UCIRTXPL5+1)/(2*f), encoder
  UCA0IRRCTL |= UCIRRXFL4+ UCIRRXPL+ UCIRRXFE;//(UCIRRXFL4+4)/(2*f), low pulse, receive filter
// UCA1STAT |=
// UCA1RXBUF =
// UCA1TXBUF =
// UCA1IRTCTL |=  
// UCA1IRRCTL |=
// UCA1ABCTL |=
  UCA0CTL1 &= ~UCSWRST;         //关闭可写控制
  UCA0IE |= UCRXIE;//+ UCTXIE   //开接收中断
  P3OUT |= BIT5; 
  P3DIR &= ~BIT5; 
  P3REN |= BIT5; 
  P3SEL |= BIT5;         //端口第二功能 + BIT7
  //UCA1IFG = UCRXIFG+ UCTXIFG;  
}*/



//void Pwm_Init()//PWM init
//{
//  P8DIR |= BIT1+ BIT2+ BIT3+ BIT4;
//  P8OUT |= BIT1+ BIT2+ BIT3+ BIT4;//端口选择
//  P8SEL |= BIT1+ BIT2+ BIT3+ BIT4;
//  TA0CCR1 = 0;
//  TA0CCR2 = 0;
//  TA0CCR3 = 0;
//  TA0CCR4 = 0;                  //
//  TA0CCTL1 |= OUTMOD_6;
//  TA0CCTL2 |= OUTMOD_6;
//  TA0CCTL3 |= OUTMOD_6;
//  TA0CCTL4 |= OUTMOD_6;         //翻转模式
//}
/*void Pwm_Init()//PWM init
{
  P8DIR |= BIT1+ BIT2+ BIT3+ BIT4;
  P8OUT |= BIT1+ BIT2+ BIT3+ BIT4;//端口选择
  TA0CCR1 = 0;
  TA0CCR2 = 0;
  TA0CCR3 = 0;
  TA0CCR4 = 0;                  //
  TA0CCTL1 |= OUTMOD_2;
  TA0CCTL2 |= OUTMOD_2;
  TA0CCTL3 |= OUTMOD_2;
  TA0CCTL4 |= OUTMOD_2;         //翻转模式
}*/

//void Pwm_Com(uint v1,uint v2,uint v3,uint v4)//PWM 控制， 需要开timera0 调节范围 0-TA0CCR0
//{ 
//  if((v1>=Pwm_min)&&(v1<=Pwm_max)){
//    TA0CCR1 = v1;
//  }
//  if((v2>=Pwm_min)&&(v2<=Pwm_max)){
//    TA0CCR2 = v2;
//  }
//  if((v3>=Pwm_min)&&(v3<=Pwm_max)){
//    TA0CCR3 = v3;
//  }
//  if((v4>=Pwm_min)&&(v4<=Pwm_max)){
//    TA0CCR4 = v4;
//  }
//  /*
//  if(v2<Pwm_min){ 
//    TA0CCR2 = Pwm_min;
//  }
//  else if(v2>Pwm_max){
//    TA0CCR2 = Pwm_max;
//  }
//  else{
//    TA0CCR2 = v2;
//  }
//  
//  if(v3<Pwm_min){ 
//    TA0CCR3 = Pwm_min;
//  }
//  else if(v3>Pwm_max){
//    TA0CCR3 = Pwm_max;
//  }
//  else{
//    TA0CCR3 = v3;
//  }
//  
//  if(v4<Pwm_min){ 
//    TA0CCR4 = Pwm_min;
//  }
//  else if(v4>Pwm_max){
//    TA0CCR4 = Pwm_max;
//  }
//  else{
//    TA0CCR4 = v4;
//  }*/
//}

/*uchar Pwm_Com(int vl,int vr)//PWM 控制， 需要开timera0 调节范围 0-TA0CCR0
{
  uint t0;
  uchar SPD=0;
  t0 = TA0CCR0;
  if(vl<0)
  {
    vl = -vl;
    if(vl>t0)vl=t0;
    TA0CCR2 = vl;
    P8SEL &= ~BIT1;    
    P8OUT |= BIT1;
    P8SEL |= BIT2;
    SPD |= 0x01;    
  }
  else
  {
    if(vl>t0)vl=t0;
    TA0CCR1 = vl;
    P8SEL &= ~BIT2;
    P8OUT |= BIT2;
    P8SEL |= BIT1; 
    SPD &= ~0x01;   
  }
  if(vr<0)
  {
    vr = -vr;
    if(vr>t0)vr=t0;
    TA0CCR4 = vr;
    P8SEL &= ~BIT3;
    P8OUT |= BIT3;
    P8SEL |= BIT4; 
    SPD |= 0x02;   
  }
  else
  {
    if(vr>t0)vr=t0;
    TA0CCR3 = vr;
    P8SEL &= ~BIT4;
    P8OUT |= BIT4;
    P8SEL |= BIT3; 
    SPD &= ~0x02;   
  }
  return SPD;           //返回方向信息
}*/
/////////////////////////////////////////////////////////////////////////
/*void Clock_Init(void)//时钟初始化
{
  WDTCTL = WDTPW + WDTHOLD;     //关闭看门狗
 // P11DIR |= 0x07;             //时钟输出
 // P11SEL |= 0x07;
  __bis_SR_register(SCG0);      //close FLL
  
  P5SEL |= 0x0c;
  P7SEL |= 0x03;                //开晶振端口功能
  
  UCSCTL6 &= ~(XT1OFF+ XT2OFF); //开xt1,xt2
  UCSCTL6 |= XCAP_3;            //cap selection
  do
  {
    UCSCTL7 &= ~(XT2OFFG+ XT1LFOFFG+ XT1HFOFFG+ DCOFFG);
    SFRIFG1 &= ~OFIFG;
  }while (SFRIFG1&OFIFG);
  
  UCSCTL6 &= ~XT2DRIVE_2;       //16m晶振驱动
  UCSCTL5 |= DIVS_1;            //smclk/2
  UCSCTL4 |= SELS_5 + SELM_5;   //smclk:8m, alk:32768
}*/








