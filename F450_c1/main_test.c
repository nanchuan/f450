
#include "msp430init.h"
//#include "LCD_5110.h"
#include "NRF24L01.h"
#include "MPU6050.h"


#define TA0Mark 0x01    //定时器a0状态位
#define TA1Mark 0x02    //定时器a1状态位
#define ERROR   0x04    //飞行器锁定状态位
#define PIDC    0x08    //pid数据发送状态位
#define NRFIQ   0x10    //无线中断状态位
#define KEYIQ   0x20    //按键中断
#define HIGH    0x40    //高度数据发送状态
#define LOST    0x80
#define AUTO    0x100

#define TA0US (uint)4000        //定时器0时间us
#define TA1US (uint)30000       //定时器1时间us

#define FLASHL 7        //flash读写数据长度
#define q0 quatf[0]     //四元数计算
#define q1 quatf[1]
#define q2 quatf[2]
#define q3 quatf[3]

#define FLTC    2
#define ACC_2   (int)1574


#define Flash_w() flashd[0] = KP;flashd[1] = KPH;flashd[2] = KD;\
  flashd[3] = KPZ;flashd[4] = KIH;flashd[5] = KDZ;flashd[6] = KDH    //写flash
#define Flash_r() KP = flashd[0];KPH = flashd[1];KD = flashd[2];\
  KPZ = flashd[3];KIH = flashd[4];KDZ = flashd[5];KDH = flashd[6]    //读flash


uint Mark = 0;                          //状态记录
int urat_temp = 0,high_tmp = 0;       //串口中断全局变量,高度计算
int ad_tmp[4];                          //模数转换寄存

void main(void)
{
  uchar rxbuf[RX_PLOAD_WIDTH]={0},txbuf[TX_PLOAD_WIDTH]={0};    //无线收发
  
  int point1=0,point2=0,point3=0,point4=0;                      //姿态控制
  
  MPU6050DMPDATA;                               //MPU6050所需变量定义
  float quatf[4];                               //四元数
  int Pitch,Roll,Yaw,Yaw_tmp,Yaw_offst=0;       //
  
  int error1=0,error2=0,error3=0;       //pid误差
  int derr11=0,derr21=0;                //pid微分计算
  int KP,KD,KPZ,KDZ,KIH,KPH,KDH;        //PID参数 55 0 12 60 0 10
  int acc2_tmp=0,shkabsprt=ACC_2;
  int throttle_out=Pwm_min,pidout1=0,pidout2=0,pidout3=0;       //pid计算输出
  
  int pwm1=Pwm_min,pwm2=Pwm_min,pwm3=Pwm_min,pwm4=Pwm_min;      //pwm 电机
  
  int flashd[FLASHL];
  
  int hight=0,avin,hight_cc,hight_ccn=0,hight_p=0,hight_p_2=0,hight_d=0;
  int throttle_p=0,throttle_d=0,throttle=Pwm_min;//,hight_i=0,throttle_tmp;
  
  uint ta1_tmp=0;       //定时器a1中断全局变量,uart_tt=0,高度采集时间
  uchar count=0;        //lock,锁定状态，消息记录，用于遥控反馈
  int tt=0;             //
  
  
  Clock_Init();         //20mhz
  Timera0_Init(TA0US);  //0-65535(us)
  Timera1_Init(TA1US);  //0-65535(us)
  Uart1_Init();         //9600 
  Adc12_Init();         //
  
  Led_Init();
  Key_Init();Key_IE();
  
  Pwm_Init(); 
  Pwm_Com(Pwm_max,Pwm_max,Pwm_max,Pwm_max);
  delay_ms(1000);       //电机上电初始化  
  
  Flash_read(flashd,Flash_ptr,FLASHL);  //读取pid参数
  Flash_r();
  
  while(mpu605dmp_init());      //MPU6050初始化
  Nrf_Init();                   //无线初始化
  
  Pwm_Com(Pwm_min,Pwm_min,Pwm_min,Pwm_min);     //电机清零,初始化完成
  
  Mark |= PIDC+ NRFIQ+ ERROR; Led2_0();         //状态初始化  
  hight_cc = 0;
  
  _BIS_SR(GIE); //开总中断 
  
  while(1)      //主循环
  {  
    if(Mark&NRFIQ)      //无线事件中断
    {
      Mark &=~NRFIQ;    //清除状态标记
      if((RX_DR)& Nrf_Check_Event(rxbuf))       //有无接收到数据
      { 
        Led1_1();
        switch(rxbuf[0]){
        case 1:                                                 //接收到控制量          
          if(!(Mark&LOST)){                     //no losting
            point4 = rxbuf[1]*256+ rxbuf[2];    //read control data
            point3 = rxbuf[3]*256+ rxbuf[4];
            point2 = rxbuf[5]*256+ rxbuf[6];
            point1 = rxbuf[7]*256+ rxbuf[8];
            
            if(rxbuf[9]){                       //auto
              if(!(Mark&AUTO)){                 //change to auto
                Mark |= AUTO;
                throttle = throttle_out;
              }            
            }
            else{                               //man
              if(Mark&AUTO){                    //change
                Mark &=~AUTO;
              }
              throttle_out = point4+ Pwm_min;
            }
          }
          else if(rxbuf[10]){                   //capture
            Mark &=~LOST;
            Led4_1();
          } 
          break;
        case 2:                                                 //写pid参数到flash
          Flash_w();
          Flash_write(flashd,Flash_ptr,FLASHL);
          Led1_0();count++;
          break;
        case 3:                                                 //解锁
          Yaw_offst = Yaw_tmp;                  //clear
          throttle = Pwm_min;
          throttle_out = Pwm_min;
          if(rxbuf[9]){
            Mark |= AUTO;
          }
          else{
            Mark &= ~AUTO;
          }
          Mark &= ~ERROR;
          Led2_1(); 
          break;
        case 4:                                                 //锁定
          Mark |= ERROR;
          Led2_0(); 
          break;
        case 5:                                                 //姿态清零,校准
          if(Mark&ERROR){
            while(mpu605dmp_init());
            Yaw_offst = 0;
          }
	  Led1_0();count++;
          break;
        case 6:                                                 //接收到pid参数
          KP = rxbuf[11];
//          KI = rxbuf[12];
          KD = rxbuf[13];
          KPZ = rxbuf[14];
//          KIZ = rxbuf[15];
          KDZ = rxbuf[16];
          KPH = rxbuf[17];
          KIH = rxbuf[18];
          KDH = rxbuf[19];
          Led1_0();count++;
          break;
        case 7:                                                 //发送现pid参数
          Mark |= PIDC;
          Led1_0();count++;
          break;
        default:
          break;
        }rxbuf[0] = 0;
        tt++;           //记录接收数据次数        
      } //if((RX_DR)& Nrf_Check_Event(rxbuf))
    }//if(Mark&NRFIQ)
    
    dmp_read_fifo(gyro,accel,quatl,&sensors,&more);     //读取MPU6050DMP数据
//    if(sensors&INV_WXYZ_QUAT) //MPU6050DMP数据正确
//    {
//    }
    
    if(Mark&TA0Mark)            //定时器a0时间到达,4ms     
    {
      Mark &=~TA0Mark;          //清除状态标记
      
      quatf[0] = quatl[0]/q30;  //四元数归一化
      quatf[1] = quatl[1]/q30;
      quatf[2] = quatl[2]/q30;
      quatf[3] = quatl[3]/q30;
      
      Roll  = (int)(asin(2*q1*q3- 2*q0*q2)*573.14);                     //roll -90~90
      
      Pitch = (int)(atan2(2*q2*q3+ 2*q0*q1,
                          -2*q1*q1- 2*q2*q2+ 1)*573.14);                //pitch -180~180
      Yaw_tmp = (int)(atan2(2*(q1*q2+ q0*q3),
                            q0*q0+ q1*q1- q2*q2- q3*q3)*573.14);        //Yaw -180~180
      Yaw =Yaw_tmp- Yaw_offst;  //776us
      
//      gravity = (long)accel[0]*accel[0];//(int)sqrt((long)accel[0]^2+ (long)accel[1]^2+ (long)accel[2]^2);
//      gravity += (long)accel[1]*accel[1];
//      gravity += (long)accel[2]*accel[2];
//      gravity /= 100000;        //2256      
      
      if(!(Mark&ERROR))
      { 
        if((Pitch>300)||(Roll>300)||(Pitch<-300)||(Roll<-300))  
        {
          Mark |= ERROR;        //姿态角大于30度,锁定飞行器
          Led2_0(); 
        }
        else{            
          derr11 = error1;
          error1 = point1- Pitch;       //pid for Pitch
//          pst1 += error1;          
          pidout1 = ((long)error1*KP- (long)(gyro[0])*KD)/100+ (error1- derr11);//+ pst1*KI
          
          derr21 = error2;
          error2 = point2- Roll;        //pid for Roll
//          pst2 += error2;
          pidout2 = ((long)error2*KP+ (long)(gyro[1])*KD)/100+ (error2- derr21);//+ pst2*KI
          
          error3 = point3- Yaw;         //pid for Yaw
          if((error3<450)&&(error3>-450))
            pidout3 = ((long)error3*KPZ- (long)gyro[2]*KDZ)/100; 
        
          acc2_tmp = (acc2_tmp*9+ accel[2]/10)/10;
          shkabsprt = (acc2_tmp- ACC_2)/10;
          
          pwm1 = throttle_out- pidout2- pidout3- shkabsprt;      //控制融合
          pwm2 = throttle_out- pidout1+ pidout3- shkabsprt;     
          pwm3 = throttle_out+ pidout2- pidout3- shkabsprt;      
          pwm4 = throttle_out+ pidout1+ pidout3- shkabsprt; 
        }
      }
      else{
        pwm1 = Pwm_min;pwm2 = Pwm_min;
        pwm3 = Pwm_min;pwm4 = Pwm_min;
      }
      Pwm_Com(pwm1,pwm2,pwm3,pwm4);     //输出到电机
    }//if((Mark&TA0Mark))               //运行时间：963us     
    
    if((Mark&TA1Mark))                  //定时器a1时间到达,30ms
    {
      Mark &=~TA1Mark;                  //清除状态标记
      
      if(Mark&KEYIQ){ 
        Mark &=~KEYIQ;
        Mark |= ERROR;                  //按键中断事件
        Led2_0(); 
      }  
      
      ad_tmp[3] = ad_tmp[2];
      ad_tmp[2] = ad_tmp[1];
      ad_tmp[1] = ad_tmp[0];
      avin = (ad_tmp[3]+ ad_tmp[2]+ ad_tmp[1]+ ad_tmp[0])/4;    //ADC滤波，电池电压
      avin /= 27;
      Adc12_Start();    //开启模数转换        

      if(!(Mark&HIGH)){
        Mark |= HIGH;
        hight_cc = high_tmp- hight; 
        if(((hight_cc>-100)&&(hight_cc<100))||(hight_ccn>10)){
          hight = (hight*(10- FLTC)+ high_tmp*FLTC)/10;
          hight_ccn = 0;
        }
        else{
          hight_ccn++;
        }
        
        if((!(Mark&ERROR))&&(Mark&AUTO)){          
          if(hight<3000){
            hight_p_2 = hight_p;
            hight_p = point4- hight;  
            hight_d = hight_p- hight_p_2;
            
            if((hight_p<150)&&(hight_p>-150)){
              throttle_p = KPH*hight_p/100;
              throttle_d = KDH*hight_d/10;
            }
            
            if(hight_d>0)
            {
              if((hight_p>50)&&(throttle<Pwm_max)){              
                  throttle += KIH;
              }
            }
            else if((hight_p<-50)&&(throttle>Pwm_min)){              
               throttle -= KIH;
            }
//              if(!((hight_i>1000)&&(hight_c>0))){
//                hight_i += hight_c/100;              
//              }
//              throttle += ((long)hight_i*KIH)/100;              
            throttle_out = throttle_p+ throttle+ throttle_d;
          }
          else
          {
            if(hight_cc>0)
              throttle_out -= KIH;
          }
        }
      }
      
      urat_temp = 0;
      Uart1_Send_Char(0x55);    //开启超声波测量   
      
      
      if(Mark&PIDC){
        Mark &=~PIDC;
        PID_update(txbuf,KP,0,KD,KPZ,0,KDZ,KPH,KIH,KDH);        //发送pid参数
      }
      else{
        Scop_update(txbuf,Roll,Pitch,Yaw,hight,throttle_out,point2,point1,point3,
                   (pwm1-Pwm_min)/10,(pwm2-Pwm_min)/10,(pwm3-Pwm_min)/10,(pwm4-Pwm_min)/10,
                   avin,Mark,count,
                   accel[2],gyro[1],gyro[2]);
      }
      
      NRF_TxPacket(txbuf);      //启用无线发送
      Led3_01();
      
      ta1_tmp++;
      if(ta1_tmp>=20){
        ta1_tmp = 0;
        if(0==tt){              //长时间无接收,锁定飞行器
          Mark|= LOST;Led4_0();
          point4 = 0;
          point1 = 0;
          point2 = 0;         
        }tt = 0;
      }
      if(Mark&LOST){
        if(hight<150){
          Mark |= ERROR;
          Led2_0(); 
        }
        else if(!(Mark&AUTO)){                     
          Mark |= AUTO;
          throttle = throttle_out;
        }   
      } 
    }//if((Mark&TA1Mark))    
  }//while(1)
}//main





#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timera0_0(void)
{
  Mark |= TA0Mark;  
}

//定时器 a1 中断
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timera1_0(void)
{  
  Mark |= TA1Mark;  
}

//串口1中断
#pragma vector=USCI_A1_VECTOR
__interrupt void uart1(void)
{  
  if(UCA1IFG&UCRXIFG)
  {
    if(0==urat_temp){
      high_tmp = UCA1RXBUF;      
      urat_temp++;
    }
    else{
      high_tmp *= 256;
      high_tmp |= UCA1RXBUF;
//      urat_temp = 0;
      Mark &=~HIGH;
    }
//    urat_temp = UCA1RXBUF;
//    Uart1_Send_Char(urat_temp);
    UCA1IFG &= ~UCRXIFG;
  }
}

//p1中断
#pragma vector = PORT1_VECTOR 
__interrupt void Port1(void)
{
  if(P1IFG&key1){
    Mark |= KEYIQ;
    P1IFG &=~key1;
  }
}

//p2中断
#pragma vector = PORT2_VECTOR 
__interrupt void Port2(void)
{
  if(P2IFG&BIT0){
    Mark |= NRFIQ;
    P2IFG &=~BIT0;
  }
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  switch(__even_in_range(ADC12IV,34))
  {
//  case  0: break;                           // Vector  0:  No interrupt
//  case  2: break;                           // Vector  2:  ADC overflow
//  case  4: break;                           // Vector  4:  ADC timing overflow
//  case  6: break;                           // Vector  6:  ADC12IFG0
//  case  8: break;                           // Vector  8:  ADC12IFG1
//  case 10: break;                               // Vector 10:  ADC12IFG2
//  case 12: break;                            // Vector 12:  ADC12IFG3
  case 14: 
    ad_tmp[0] = ADC12MEM4;
    break;
//    __bic_SR_register_on_exit(LPM4_bits);   // Exit active CPU, SET BREAKPOINT HERE                              
//  case 16: break;                           // Vector 16:  ADC12IFG5
//  case 18: break;                           // Vector 18:  ADC12IFG6
//  case 20: break;                           // Vector 20:  ADC12IFG7
//  case 22: break;                           // Vector 22:  ADC12IFG8
//  case 24: break;                           // Vector 24:  ADC12IFG9
//  case 26: break;                           // Vector 26:  ADC12IFG10
//  case 28: break;                           // Vector 28:  ADC12IFG11
//  case 30: break;                           // Vector 30:  ADC12IFG12
//  case 32: break;                           // Vector 32:  ADC12IFG13
//  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break; 
  }
}


//void Flash_w(void)
//{
//  flashd[0] = KP;
//  flashd[1] = KIH;
//  flashd[2] = KD;
//  flashd[3] = KPZ;
////  flashd[4] = KI;
//  flashd[5] = KDZ;
////  flashd[6] = (gyrost[0]>>16);
////  flashd[7] = (gyrost[0]);
////  flashd[8] = (gyrost[1]>>16);
////  flashd[9] = (gyrost[1]);
////  flashd[10] = (gyrost[2]>>16);
////  flashd[11] = (gyrost[2]);
////  flashd[12] = (accelst[0]>>16);
////  flashd[13] = (accelst[0]);
////  flashd[14] = (accelst[1]>>16);
////  flashd[15] = (accelst[1]);
////  flashd[16] = (accelst[2]>>16);
////  flashd[17] = (accelst[2]);
//}

//void Flash_r(void)
//{
//  KP = flashd[0];
//  KIH = flashd[1];
//  KD = flashd[2];
//  KPZ = flashd[3];
////  KI = flashd[4];
//  KDZ = flashd[5];
////  gyrost[0] = (long)flashd[6]<<16 | flashd[7];
////  gyrost[1] = (long)flashd[8]<<16 | flashd[9];
////  gyrost[2] = (long)flashd[10]<<16 | flashd[11];
////  accelst[0] = (long)flashd[12]<<16 | flashd[13];
////  accelst[1] = (long)flashd[14]<<16 | flashd[15];
////  accelst[2] = (long)flashd[16]<<16 | flashd[17];
//}

/*
void Contr_Check(){
  
    if((RX_DR)& Nrf_Check_Event(rxbuf))//Rx_read(rxbuf)
    { 
      switch(rxbuf[0]){
      case 1:
        if(Mark&ERROR){
          while(mpu605dmp_init());
          NRF_Write_Reg(FLUSH_RX,0xff);
        }
        else{
          point4++;
        } 
        Led1_0();count++;
        break;
      case 5:
        if(Mark&ERROR){
        }
        else{
          point4--;
        }  
        Led1_0();count++;
        break;
      case 2:
        if(Mark&ERROR){
        }
        else{
          point2 = 50;
        }
        Led1_0();count++;
        break;
      case 6:
        if(Mark&ERROR){
        }
        else{
          point2 = -50;
        }
        Led1_0();count++;
        break;      
      case 3:
        if(Mark&ERROR){
          KPZ++;Mark |= PIDC;
        }
        else{
          point1 = -50;
        }        
        Led1_0();count++;
        break;
      case 4:
        if(Mark&ERROR){
          KDZ++;Mark |= PIDC;
        }
        else{
          point1 = 50;   
        }     
        Led1_0();count++;
        break;
      case 7:
        if(Mark&ERROR){
          KPZ--;Mark |= PIDC;    
        } 
        else{
          point3+=10;
          if(point3>900){
            point3 = -900;
          }
        }  
        Led1_0();count++;
        break;
      case 8:
        if(Mark&ERROR){
          KDZ--;Mark |= PIDC;  
        }
        else{
          point3-=10;
          if(point3<-900){
            point3 = 900;
          }
        }
        Led1_0();count++;
        break;
      case 9:
        if(Mark&ERROR){
          KP++;Mark |= PIDC;   
        }
        else{
          point4+=10;
        }     
        Led1_0();count++;
        break;
      case 10:
        if(Mark&ERROR){
          KI++;Mark |= PIDC;
        }
        Led1_0();count++;
        break;
      case 11:
        if(Mark&ERROR){
          KD++;Mark |= PIDC;
        }
        Led1_0();count++;
        break;
      case 13:
        if(Mark&ERROR){
          KP--;Mark |= PIDC;
        }
        else{
          point4-=10;
        }
        Led1_0();count++;
        break;
      case 14:
        KI--;Mark |= PIDC;
        Led1_0();count++;
        break;
      case 15:
        if(Mark&ERROR){
          KD--;Mark |= PIDC;
        }
        Led1_0();count++;
        break;
      case 12:
        if(!delay_t)
        {
          Flash_w();
          Flash_write(flashd,Flash_ptr,FLASHL);
          delay_t = 100;
          Led1_0();count++;
        }
        break;
      case 16:
        if(!delay_t)
        {
          if(Mark&ERROR){
            Mark &= ~ERROR;Led2_1(); 
          }
          else{
            Pwm_Com(Pwm_min,Pwm_min,Pwm_min,Pwm_min);
            point4 = Pwm_min;
            pst1 = 0;
            pst2 = 0;
            point3 = 0;           
            Mark |= ERROR;Led2_0(); 
          }delay_t = 100;
          Led1_0();count++;
        }
        break;
      default:
        point1 = point2 = 0;
        break;
      }rxbuf[0] = 0;
      tt++;
    } 
}

*/

/*
//        else if(Mark&PIDZC){
//          Mark &=~PIDZC;
//          scop_RAM(txbuf,26,(float)KPZ,0,(float)KDZ);
//        }
//        else if(Mark&POINTS){
//          Mark &=~POINTS;
//          scop_point(txbuf,(float)point4,(float)point1,(float)point2,(float)point3);
////          scop_RAM(txbuf,26,gyro[0],gyro[1],gyro[2]);
//        }
      else if((ta1_tmp%2)){
//        gyro[1] = shkabsprt;
//        gyro[1] = gravity;
//        accel[2] = acc2_tmp;
        ANO_scopdatAF(txbuf,(int*)accel,(int*)gyro,Roll*10,Pitch*10,Yaw,count);     //
      }
      else{
//          scop_quat(txbuf,quatf); 
        ANO_scopdatAE(txbuf,throttle_out- 500,point3+1500,1500-point2,point1+1500,hight,Mark,
                      (pwm1-Pwm_min)/10,(pwm2-Pwm_min)/10,(pwm3-Pwm_min)/10,(pwm4-Pwm_min)/10,avin);
      }*/







