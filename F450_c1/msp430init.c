#include "msp430init.h"



//ʱ�ӳ�ʼ�� smclk:20m, aclk:32768
void Clock_Init(void)
{
  WDTCTL = WDTPW + WDTHOLD;     //�رտ��Ź�
 // P11DIR |= 0x07;             //ʱ�����
 // P11SEL |= 0x07;
  __bis_SR_register(SCG0);      //close FLL
  
  P5SEL |= 0x0c;
  P7SEL |= 0x03;                //������˿ڹ���
  
  UCSCTL6 &= ~(XT1OFF+ XT2OFF); //��xt1,xt2
  UCSCTL6 |= XCAP_3;            //cap selection
  do
  {
    UCSCTL7 &= ~(XT2OFFG+ XT1LFOFFG+ XT1HFOFFG+ DCOFFG);
    SFRIFG1 &= ~OFIFG;
  }while (SFRIFG1&OFIFG);
  
  UCSCTL6 &= ~XT2DRIVE_2;       //20m��������
//  UCSCTL5 |= DIVS_0;          //smclk
  UCSCTL4 |= SELS_5 + SELM_5;   //mclk��smclk:20m, aclk:32768
}

//ACLK = 32k,smclk = 25m, mclk = 25m
void Clock_Init_DCO(void)
{
  WDTCTL = WDTPW+WDTHOLD;                       // Stop WDT
//  P1DIR |= BIT0;                              // P1.0 output
//  P11DIR |= 0x07;                             // ACLK, MCLK, SMCLK set out to pins
//  P11SEL |= 0x07;                             // P11.0,1,2 for debugging purposes.

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


///////////////////////////////////////////////////////////////////////////
//timera0��ʼ��, init 0-65535 (us)
void Timera0_Init(uint t)
{
  TA0CTL |= TACLR;      //clear
  TA0CTL |= TASSEL_2+ ID_2 + MC_1;//+ TAIE      //smclk,4��Ƶ,up mode  
  TA0EX0 = TAIDEX_4;    //5��Ƶ
  TA0CCR0 = t;          //��ʱ t us
  TA0CCTL0 |= CCIE;     //ta0.0 interrupt
}

//timera1 init 0-65535 (us)
void Timera1_Init(uint t)
{
  TA1CTL |= TACLR;                      //clear
  TA1CTL |= TASSEL_2+ ID_2+ MC_1;       //+ TAIE   //smclk,4��Ƶ,up mode
  TA1EX0 = TAIDEX_4;                    //5��Ƶ
  TA1CCR0 = t;                          //��ʱ t us
  TA1CCTL0 |= CCIE;                     //ta1.0 interrupt
}

//PWM init
void Pwm_Init()
{
  P8DIR |= BIT1+ BIT2+ BIT3+ BIT4;
  P8OUT |= BIT1+ BIT2+ BIT3+ BIT4;      //�˿�ѡ��
  P8SEL |= BIT1+ BIT2+ BIT3+ BIT4;
  TA0CCR1 = 0;
  TA0CCR2 = 0;
  TA0CCR3 = 0;
  TA0CCR4 = 0;                  //
  TA0CCTL1 |= OUTMOD_6;
  TA0CCTL2 |= OUTMOD_6;
  TA0CCTL3 |= OUTMOD_6;
  TA0CCTL4 |= OUTMOD_6;         //��תģʽ
}

//PWM ���ƣ� ��Ҫ��timera0 ���ڷ�Χ 0-TA0CCR0
void Pwm_Com(int v1,int v2,int v3,int v4)
{ 
  if(v1<Pwm_min)
    TA0CCR1 = Pwm_min;
  else if(v1>Pwm_max)
    TA0CCR1 = Pwm_max;
  else
    TA0CCR1 = v1;
  
  if(v2<Pwm_min)
    TA0CCR2 = Pwm_min;
  else if(v2>Pwm_max)
    TA0CCR2 = Pwm_max;
  else
    TA0CCR2 = v2;
  
  if(v3<Pwm_min)
    TA0CCR3 = Pwm_min;
  else if(v3>Pwm_max)
    TA0CCR3 = Pwm_max;
  else
    TA0CCR3 = v3;
  
  if(v4<Pwm_min)
    TA0CCR4 = Pwm_min;
  else if(v4>Pwm_max)
    TA0CCR4 = Pwm_max;
  else
    TA0CCR4 = v4;
    
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
}

//����1��ʼ��
void Uart1_Init()
{
//  UCA1CTL0 |= 0;
  UCA1CTL1 |= UCSWRST;          //�� ��д ����λ
  UCA1CTL1 = UCSSEL_1;          //ack
  UCA1BR0 = 0x03;
  UCA1BR1 = 0;
  UCA1MCTL = UCBRS_3+ UCBRF_0;  //���������� 9600
//  UCA1CTL1 = UCSSEL_2;        //smclk
//  UCA1BR0 = 173;
//  UCA1BR1 = 0;
//  UCA1MCTL = UCBRS_5+ UCBRF_0;        //���������� 115200
  P5REN |= BIT6 + BIT7;
  P5OUT |= BIT6 + BIT7;
  P5SEL |= BIT6 + BIT7;         //�˿ڵڶ�����
// UCA1STAT |=
// UCA1RXBUF =
// UCA1TXBUF =
// UCA1IRTCTL |=  
// UCA1IRRCTL |=
// UCA1ABCTL |=
  UCA1CTL1 &= ~UCSWRST;         //�رտ�д����
  UCA1IE |= UCRXIE;//+ UCTXIE   //�������ж�
//  UCA1IFG |= UCTXIFG;         //UCRXIFG+  
}

/////////////////////////////////////////////////////////
//����1�����ַ�(����c)����c
uchar Uart1_Send_Char(uchar c)
{
  while(!(UCA1IFG&UCTXIFG));    //�ȴ����Ϳ���
  UCA1TXBUF = c;
  UCA1IFG &= ~UCTXIFG;
  return c;
}

//����1�����ַ���(�ַ���)
void Uart1_Send_String(uchar *s)
{
  while(*s)     //�Ƿ�Ϊ��
  {
    Uart1_Send_Char(*s++);
  }
}

//����1������������(���ݵ�ַ,���ݴ�С)
void USART1_SendDatas(uchar *com_serial_transmitBuffer,uchar len)
{
  int i;
  for(i=0;i<len;i++)
  {
      USART1_SendChar(*(com_serial_transmitBuffer + i));	
  }
}


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

//
void PID_update(uchar *tx,uchar p1,uchar i1,uchar d1,
                uchar p2,uchar i2,uchar d2,
                uchar p3,uchar i3,uchar d3)
{
  uchar i = 0;
  
  tx[i++] = 0x84;       //0
  tx[i++] = p1;         //1
  tx[i++] = i1;   
  tx[i++] = d1;  
  tx[i++] = p2;         //4
  tx[i++] = i2;   
  tx[i++] = d2;
  tx[i++] = p3;         //7
  tx[i++] = i3;   
  tx[i++] = d3;
}

///////////////////////////////////////////////////////////////////
//дflash(����,Ŀ���ַ,д���ݸ���)
void Flash_write(int *p,int *fp,uchar n)
{
  uint gieStatus,i;
  gieStatus = __get_SR_register() & GIE;
  __disable_interrupt();
  FCTL3 = FWKEY;                                // Clear Lock bit
  FCTL1 = FWKEY+ERASE; 
  *(int *)fp = 0;
  FCTL1 = FWKEY+WRT;  
  for (i = 0; i < n; i++)
  {
    *fp++ = *p++;                               // Write value to flash
  }
  FCTL1 = FWKEY;                                // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                           // Set LOCK bit
  __bis_SR_register(gieStatus);
}

//��FLASF(Ŀ���ַ,���ݵ�ַ,��ȡ����)
void Flash_read(int *p,int *fp,uchar n)
{
  uint i;
  for (i = 0; i < n; i++)
  {
    *p++ = *fp++;                       // Read value from flash
  }
}

/////////////////////////////////////////////////////////////////////
//ADC��ʼ��
void Adc12_Init()
{
  P6SEL |= BIT5;        //+ BIT4+ BIT6+ BIT7;P7SEL |= BIT4;�˿ڳ�ʼ��
  ADC12CTL0 &= ~ADC12ENC;               //����Ĵ�������λ
//  REFCTL0 |= REFMSTR+ REFVSEL_2+ REFON;       //INOR 2.5vREF
  ADC12CTL0 |= ADC12ON+ ADC12MSC+ ADC12SHT0_8+ ADC12SHT1_8;     // 
  ADC12CTL1 |= ADC12SHP+ ADC12CONSEQ_1;         //
//  ADC12MCTL0 |= ADC12INCH_4;          //
//  ADC12MCTL1 |= ADC12INCH_5;
//  ADC12MCTL2 |= ADC12INCH_6;  
//  ADC12MCTL3 |= ADC12INCH_7; 
  ADC12MCTL4 |= ADC12INCH_5+ ADC12EOS; 
  ADC12IE |= ADC12IE4;          //���ж�  
  ADC12CTL0 |= ADC12ENC;        //��ʼ����
//  Adc12_Start();
}

//////////////////////////////
//���Ź���ʱ
void WDT_Init()
{
  WDTCTL = WDT_ADLY_250;        //aclk,512��Ƶ,250ms  
  SFRIE1 |= WDTIE;
}



/////////////////////////////////////////////////////////////////
////SG90�����ʼ�� ta1 20ms up model
//void Sg90_Init(void)
//{
//  P7DIR |= BIT3;
//  P7SEL |= BIT3;
//  TA1CCR2 = (uint)1480;//1.5ms 0��
//  TA1CCTL2 |= OUTMOD_6;
//}
//
////�������(-900~900)
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

///////////////////////////////////////////////////
////ѡ������(����,���ݴ�С)С->��
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
/////////////////////////////////////////////////////
////bell init(times)
//void Bell(int ni)        
//{
//  int n;
//  for(n=0;n<ni;n++)
//  {
//    P5OUT |= bell;
//    delay_us(100);
//    P5OUT &= ~bell;
//    delay_us(200);
//  }
//}
//
//////////////////////////////////////
////����ң�ؽ���(���ݴ�ŵ�ַ)
//uchar IR_Ctr(IR_Buf ircom)
//{  
//  uint i,j,n=0;
//  Ircin();      //�˿ڳ�ʼ��
//  delay_ms(1);
//  if(irc_i)     //����
//  { 
//    return 0;
//  }
//  while(!irc_i) //
//  {
//    delay_us(50);
//    n++;
//  }
//  if(n<100)     //���ǰ������
//  { 
//    return 0;
//  }
//  n=0;
//  while(irc_i)  //�ȴ�����λ
//    delay_us(140);
//  for(j=0;j<4;j++)      //���ֽ�����
//  {
//    for(i=0;i<8;i++)    //��λ
//    {     
//      while(!irc_i)
//        delay_us(140); 
//      while(irc_i)
//      {
//        delay_us(140);  
//        n++;
//        if(n>30)        //ȡ����Ч����
//        {   
//          return 0;
//        }
//      }
//      ircom[j]>>=1;
//      if(n>7)   //��ȡ'1'
//        ircom[j] |= 0x80;
//      n=0;
//    }
//  }
////  delay_ms(200);
//  if((ircom[2]+ircom[3])!=255)  //����У��
//  { 
//    return 0;
//  }
//  else
//  {
//    ircom[4] = ircom[2];
//    return 1;   //����ɹ�
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

/////////////////////////////////////////////////////////////////
////PID������ʼ��(���ʼ����pid����,��������,��������,΢������)
//void PID_Init(PID_C *pid_c,int kp,int ki,int kd)
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
////pid clear(�������pid)
//void PID_Clear(PID *pid)
//{
////  pid->pidout = 0;
//  pid->pst = 0;
//  pid->derr1 = 0;
//  pid->derr2 = 0;
//}
//
////pid����(pid,���ò���,Ŀ����,ʵ����)
//int PID_Cclt(PID *pid,PID_C *pid_c,int point,int fb)
//{
//  long err,derr;
//  err = point- fb;      //�����
////  if(!(((pid->pst>=pid_c->MAX)&&(err>0))||((pid->pst<=pid_c->MIN)&&(err<0)))) 
//    pid->pst += err;    //���߻���
//  derr = pid->derr1- pid->derr2;        //΢��
//  pid->derr2 = pid->derr1;      
//  pid->derr1 = err;     
//  return (int)(((err*pid_c->KP/100+ pid->pst*pid_c->KI/1000+ derr*pid_c->KD)));//\
//  pid����(int)
////  pid->pidout = (int)err;//((pid_c->KL*(long)pid->pidout+ (100- pid_c->KL)*err)/100);//\
//    ��ͨ���ӳ�΢������(int)
////  return((int)err);
//}
//
////////////////////////////////////////////////////////////////////////////////////////////////
////ʵʱʱ�ӳ�ʼ��
//void RTC_Init(void)
//{
//  RTCCTL01 |= RTCTEVIE+ RTCRDYIE+ RTCHOLD+ RTCMODE; //+ RTCBCD\
//  �����¼����ɶ�������ʱ�䣬����ģʽ
////  RTCSEC = 0;                   //��
////  RTCMIN = 0;                   //��
////  RTCHOUR = 0;                  //ʱ
////  RTCDOW = 1;                   //����
////  RTCDAY = 1;                   //��
////  RTCMON = 1;                   //��
////  RTCYEARL = 12;                //
////  RTCYEARH = 20;                //��
//  //RTCAMIN,RTCAHOUR,RTCADOW,RTCDAY     //����
//  //RTCPS1CTL |= RT1IP_6+ RT1PSIE;      //����¼�
//  RTCCTL01 &= ~RTCHOLD;                 //��ʼ����
//}
//
///*
//void RTC_Lcd(void)//ʱ����ʾ
//{
////  while(!(RTCCTL1&RTCRDY));
////  for(;RTCCTL1&RTCRDY;)
////  Lcd_Char(36,5,RTCHOUR/10+48);
////  Lcd_Char(42,5,RTCHOUR%10+48);//ʱ
////  Lcd_Char(48,5,':');
//  Lcd_Char(54,5,RTCMIN/10+48);
//  Lcd_Char(60,5,RTCMIN%10+48);//��
//  Lcd_Char(66,5,':');
//  Lcd_Char(l13,5,RTCSEC/10+48);
//  Lcd_Char(l14,5,RTCSEC%10+48);//��
//}*/
//
////////////////////////////////////////////////////////////////////////////
////timerb0 ��ʼ���������ٶ�����
//void Timerb0_Init(void)
//{
//  TB0CTL |= TBCLR;                //clear
//  TB0CTL |= TBSSEL_1+ ID_3 + MC_2;//+ TBIE aclk:32768,8��Ƶ,����ģʽ
////  TB0EX0 |= TBIDEX_1;                 //clk/2����16��Ƶ
//  TB0CCTL1 |= CM_2+ CAP+ CCIE;          //�½��ز��񣬿��ж�
//  TB0CCTL3 |= CM_2+ CAP+ CCIE;  
//  P4SEL |= BIT1+ BIT3;          //���˿ڵڶ�����
//  P4DIR &= ~(BIT1+ BIT3);       // +BIT2+ BIT4
////  P4OUT &= ~(BIT1+ BIT3);
////  P4REN |= (BIT1+ BIT3);
//}



///////////////////////////////////////////////////////////////////////////////
////����ʾ����CRC_CHECK����
//unsigned short CRC_Check(unsigned char *Buf, unsigned char CRC_CNT)
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
////����ʾ��������֡����(���ֽ�����)
//void Digital_Scope(int *temp)
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
//  for(i=0;i<4;i++)      //
//  {
//    databuf[i*2]   = ((unsigned int)temp[i]%256);
//    databuf[i*2+1] = ((unsigned int)temp[i]/256);//(unsigned char)
//  }
//  
//  CRC16 = CRC_Check(databuf,8); //����У����
//  databuf[8] = CRC16%256;
//  databuf[9] = CRC16/256;
//  
//  for(i=0;i<10;i++)     //����
//    Uart1_Send_Char(databuf[i]); 
//}
//
////////////////////////////////////////////////////////////////
////������λ��ʾ����(����,��С)
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

////������λ��af����ʹ�����ݰ��ϳ�(��ַ,���ٶ�����,����������,roll,pitch,yaw)
//uint ANO_scopdatAF(uchar *tx,int *acc,int *gryo,int roll,int pitch,int yaw,uchar count)
//{
//  uchar i = 0,sum = 0;
//  tx[i] = 0x88;
//  sum += tx[i];i++;
//  tx[i] = 0xaf;
//  sum += tx[i];i++;
//  tx[i] = 0x1c;
//  sum += tx[i];i++;
//  tx[i] = BYTE1(acc[0]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(acc[0]);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(acc[1]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(acc[1]);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(acc[2]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(acc[2]);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(gryo[0]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(gryo[0]);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(gryo[1]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(gryo[1]);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(gryo[2]);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(gryo[2]);
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = BYTE1(roll);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(roll);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pitch);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pitch);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(yaw);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(yaw);
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = count;
//  sum += tx[i];i++;
//  tx[i] = sum;
//  return i;
//}
//
////������λ��ae����ʹ�����ݰ��ϳ�(��ַ,����,yawĿ��ֵ,rollĿ��ֵ,pitchĿ��ֵ,�߶�,\
//���1,���2,���3,���4,��Դ��ѹ)
//uint ANO_scopdatAE(uchar *tx,int THROT,int YAW,int ROLL,int PITCH,int high,int lock,
//                   int pwm1,int pwm2,int pwm3,int pwm4,int avin)
//{
//  uchar i = 0,sum = 0;
//  tx[i] = 0x88;
//  sum += tx[i];i++;
//  tx[i] = 0xae;
//  sum += tx[i];i++;
//  tx[i] = 28;
//  sum += tx[i];i++;
//  tx[i] = BYTE1(THROT);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(THROT);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(YAW);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(YAW);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(ROLL);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(ROLL);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(PITCH);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(PITCH);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(high);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(high);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(lock);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(lock);
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pwm1);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pwm1);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pwm2);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pwm2);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pwm3);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pwm3);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pwm4);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pwm4);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(avin);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(avin);
//  sum += tx[i];i++;
//  tx[i] = sum;
////  sum += tx[i];i++;
////  tx[i] = 0;
////  sum += tx[i];i++;
////  tx[i] = 0;
////  sum += tx[i];i++;
////  tx[i] = 0;
////  sum += tx[i];i++;
////  tx[i] = sum;
//  return i;
//}
//
////������λ��ac����ʹ�����ݰ��ϳ�(��ַ,roll��������,roll��������,roll΢������\
//,pitch��������,pitch��������,pitch΢������\
//  ,yaw��������,yaw��������,yaw΢������)
//uint ANO_scopdatAC(uchar *tx,uint rol_p,uint rol_i,uint rol_d
//                   ,uint pit_p,uint pit_i,uint pit_d
//                     ,uint yaw_p,uint yaw_i,uint yaw_d)
//{
//  uchar i = 0,sum = 0;
//  tx[i] = 0x88;
//  sum += tx[i];i++;
//  tx[i] = 0xac;
//  sum += tx[i];i++;
//  tx[i] = 0x1c;
//  sum += tx[i];i++;
//  tx[i] = 0xad;
//  sum += tx[i];i++;
//  tx[i] = BYTE1(rol_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(rol_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(rol_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(rol_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(rol_d);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(rol_d);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pit_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pit_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pit_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pit_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(pit_d);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(pit_d);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(yaw_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(yaw_p);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(yaw_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(yaw_i);
//  sum += tx[i];i++;
//  tx[i] = BYTE1(yaw_d);
//  sum += tx[i];i++;
//  tx[i] = BYTE0(yaw_d);
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = 0;
//  sum += tx[i];i++;
//  tx[i] = sum;
//  return i;
//}
//
////Բ�㲩ʿ��λ�����ݰ��ϳ�()
///******************************************************************************/
//uint scop_quat(uchar *rxbuf,float *quat)
//{
//  uint crct = 0;//,i=0;
//  rxbuf[0] = 0x55;
//  rxbuf[1] = 0xAA;
//  rxbuf[2] = 20;
//  rxbuf[3] = 0;
//  rxbuf[4] = 1;
//  rxbuf[5] = BYTE0(quat[0]);
//  rxbuf[6] = BYTE1(quat[0]); 
//  rxbuf[7] = BYTE2(quat[0]);
//  rxbuf[8] = BYTE3(quat[0]);
//  rxbuf[9] = BYTE0(quat[1]);
//  rxbuf[10] = BYTE1(quat[1]); 
//  rxbuf[11] = BYTE2(quat[1]);
//  rxbuf[12] = BYTE3(quat[1]);
//  rxbuf[13] = BYTE0(quat[2]);
//  rxbuf[14] = BYTE1(quat[2]); 
//  rxbuf[15] = BYTE2(quat[2]);
//  rxbuf[16] = BYTE3(quat[2]);
//  rxbuf[17] = BYTE0(quat[3]);
//  rxbuf[18] = BYTE1(quat[3]); 
//  rxbuf[19] = BYTE2(quat[3]);
//  rxbuf[20] = BYTE3(quat[3]);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,18);
//  rxbuf[21] = BYTE1(crct);
//  rxbuf[22] = BYTE0(crct);
////  rxbuf[23] = a/100+ 48;
////  rxbuf[24] = a%100/10+ 48;
////  rxbuf[25] = a%10+ 48;
////  rxbuf[26] = ',';
////  rxbuf[27] = b/100+ 48;
////  rxbuf[28] = b%100/10+ 48;
////  rxbuf[29] = b%10+ 48;
////  rxbuf[30] = ',';
////  rxbuf[31] = '\n';
//  return 0;
//}
//
////Բ�㲩ʿ��λ�����ݰ��ϳ�()
///******************************************************************************/
//uint scop_RAM(uchar *rxbuf,uint n,float f,float f1,float f2)
//{
//  uint crct = 0,i=0;
//  rxbuf[i++] = 0x55;
//  rxbuf[i++] = 0xAA;
//  rxbuf[i++] = 18;
//  rxbuf[i++] = 0;
//  rxbuf[i++] = 0x1d;
//  rxbuf[i++] = 0x12;
//  rxbuf[i++] = n;
//  rxbuf[i++] = BYTE0(f);
//  rxbuf[i++] = BYTE1(f); 
//  rxbuf[i++] = BYTE2(f);
//  rxbuf[i++] = BYTE3(f);
//  rxbuf[i++] = BYTE0(f1);
//  rxbuf[i++] = BYTE1(f1); 
//  rxbuf[i++] = BYTE2(f1);
//  rxbuf[i++] = BYTE3(f1);
//  rxbuf[i++] = BYTE0(f2);
//  rxbuf[i++] = BYTE1(f2); 
//  rxbuf[i++] = BYTE2(f2);
//  rxbuf[i++] = BYTE3(f2);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,16);
//  rxbuf[i++] = BYTE1(crct);
//  rxbuf[i++] = BYTE0(crct);
//  return 0;
//}
//
////Բ�㲩ʿ��λ�����ݰ��ϳ�()
///******************************************************************************/
//uint scop_point(uchar *rxbuf,float f,float f1,float f2,float f3)
//{
//  uint crct = 0,i=0;
//  rxbuf[i++] = 0x55;
//  rxbuf[i++] = 0xAA;
//  rxbuf[i++] = 22;
//  rxbuf[i++] = 0;
//  rxbuf[i++] = 0x1d;
//  rxbuf[i++] = 0x12;
//  rxbuf[i++] = 19;
//  rxbuf[i++] = BYTE0(f);
//  rxbuf[i++] = BYTE1(f); 
//  rxbuf[i++] = BYTE2(f);
//  rxbuf[i++] = BYTE3(f);
//  rxbuf[i++] = BYTE0(f1);
//  rxbuf[i++] = BYTE1(f1); 
//  rxbuf[i++] = BYTE2(f1);
//  rxbuf[i++] = BYTE3(f1);
//  rxbuf[i++] = BYTE0(f2);
//  rxbuf[i++] = BYTE1(f2); 
//  rxbuf[i++] = BYTE2(f2);
//  rxbuf[i++] = BYTE3(f2);
//  rxbuf[i++] = BYTE0(f3);
//  rxbuf[i++] = BYTE1(f3); 
//  rxbuf[i++] = BYTE2(f3);
//  rxbuf[i++] = BYTE3(f3);
//  crct = 0;
//  crct = math_crc16(rxbuf+3,20);
//  rxbuf[i++] = BYTE1(crct);
//  rxbuf[i++] = BYTE0(crct);
//  return 0;
//}
//
////Բ�㲩ʿ��λ�����ݰ�crc����������
///******************************************************************************/
//uint math_crc16(const void * data,uint16_t len)
//{
//    const static uint16_t crc_tab[16] =
//    {
//        0x0000 , 0x1021 , 0x2042 , 0x3063 , 0x4084 , 0x50A5 , 0x60C6 , 0x70E7 ,
//        0x8108 , 0x9129 , 0xA14A , 0xB16B , 0xC18C , 0xD1AD , 0xE1CE , 0xF1EF
//    };
//    uint8_t h_crc;
//	uint16_t crc = 0;
//    const uint8_t * ptr = (const uint8_t *)data;
//    //
//    while(len --)
//    {
//        h_crc = (uint8_t)(crc >> 12);
//        crc <<= 4;
//        crc ^= crc_tab[h_crc ^ ((*ptr) >> 4)];
//        //
//        h_crc = crc >> 12;
//        crc <<= 4;
//        crc ^= crc_tab[h_crc ^ ((*ptr) & 0x0F)];
//        //
//        ptr ++;
//    }
//    //
//    return crc;
//}
//
////Բ�㲩ʿ��λ�����ݷ���
///******************************************************************************/
//int com_serial_transmit(uchar *com_serial_transmitBuffer, uint8_t len)
//{
////	uchar com_serial_transmitBuffer[5];
//    uint16_t com_serial_transmitCrc = 0; 
//    
//    com_serial_transmitCrc = math_crc16(com_serial_transmitBuffer,len);
//
//    USART1_SendChar(0x55);
//    USART1_SendChar(0xAA);
//    USART1_SendChar(len+2);
////	USART1_SendChar(PROTOCOL);
////	USART1_SendChar(packetType);
//    USART1_SendDatas(com_serial_transmitBuffer,len);
//    USART1_SendChar(BYTE1(com_serial_transmitCrc));
//    USART1_SendChar(BYTE0(com_serial_transmitCrc));
//
//    return 0;
//}

////����1������ֵ(��ֵ) ��ֵת�ַ� 16λ
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

////����1������ֵ(��ֵ) ��ֵת�ַ� 8λ
//void Uart1_Send_Numh(uchar num)
//{  
//  Uart1_Send_Char(num/100+ 48);
//  Uart1_Send_Char(num%100/10+ 48);
//  Uart1_Send_Char(num%10+ 48);
//  Uart1_Send_Char(' ');
//}



/////////////////////////////////////////////////////////////////////
/*
void Uart0_Init()//����0��ʼ��,�����������
{
//  UCA0CTL0 |= 0;
  UCA0CTL1 |= UCSWRST;          //�� ��д ����λ
  UCA0CTL1 = UCSSEL_2;          //smclk
//  UCA0BR0 = 0x03;
//  UCA0BR1 = 0;
//  UCA0MCTL = UCBRS_3+ UCBRF_0;  //���������� 9600
  UCA0IRTCTL |= UCIRTXPL5+ UCIREN;//(UCIRTXPL5+1)/(2*f), encoder
  UCA0IRRCTL |= UCIRRXFL4+ UCIRRXPL+ UCIRRXFE;//(UCIRRXFL4+4)/(2*f), low pulse, receive filter
// UCA1STAT |=
// UCA1RXBUF =
// UCA1TXBUF =
// UCA1IRTCTL |=  
// UCA1IRRCTL |=
// UCA1ABCTL |=
  UCA0CTL1 &= ~UCSWRST;         //�رտ�д����
  UCA0IE |= UCRXIE;//+ UCTXIE   //�������ж�
  P3OUT |= BIT5; 
  P3DIR &= ~BIT5; 
  P3REN |= BIT5; 
  P3SEL |= BIT5;         //�˿ڵڶ����� + BIT7
  //UCA1IFG = UCRXIFG+ UCTXIFG;  
}*/

/*void Pwm_Init()//PWM init
{
  P8DIR |= BIT1+ BIT2+ BIT3+ BIT4;
  P8OUT |= BIT1+ BIT2+ BIT3+ BIT4;//�˿�ѡ��
  TA0CCR1 = 0;
  TA0CCR2 = 0;
  TA0CCR3 = 0;
  TA0CCR4 = 0;                  //
  TA0CCTL1 |= OUTMOD_2;
  TA0CCTL2 |= OUTMOD_2;
  TA0CCTL3 |= OUTMOD_2;
  TA0CCTL4 |= OUTMOD_2;         //��תģʽ
}*/


/*uchar Pwm_Com(int vl,int vr)//PWM ���ƣ� ��Ҫ��timera0 ���ڷ�Χ 0-TA0CCR0
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
  return SPD;           //���ط�����Ϣ
}*/

/////////////////////////////////////////////////////////////////////////
//ʱ�ӳ�ʼ��
/*void Clock_Init(void)
{
  WDTCTL = WDTPW + WDTHOLD;     //�رտ��Ź�
 // P11DIR |= 0x07;             //ʱ�����
 // P11SEL |= 0x07;
  __bis_SR_register(SCG0);      //close FLL
  
  P5SEL |= 0x0c;
  P7SEL |= 0x03;                //������˿ڹ���
  
  UCSCTL6 &= ~(XT1OFF+ XT2OFF); //��xt1,xt2
  UCSCTL6 |= XCAP_3;            //cap selection
  do
  {
    UCSCTL7 &= ~(XT2OFFG+ XT1LFOFFG+ XT1HFOFFG+ DCOFFG);
    SFRIFG1 &= ~OFIFG;
  }while (SFRIFG1&OFIFG);
  
  UCSCTL6 &= ~XT2DRIVE_2;       //16m��������
  UCSCTL5 |= DIVS_1;            //smclk/2
  UCSCTL4 |= SELS_5 + SELM_5;   //smclk:8m, alk:32768
}*/









