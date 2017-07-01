
#include "msp430init.h"
#include "LCD_5110.h"
#include "NRF24L01.h"
#include "MPU6050.h"

#define FPN 80          //AD滤波常数
#define TA0Mark 0x01    //TA0定时器中断标记(事件发生,标志位置"1")
#define TA1Mark 0x02    //TA1定时器中断标记
#define FSTA    0x04    //飞行器锁定标记,"1"--锁定,"0"--飞行
#define KEY1    0x08    //按键 1 标记
#define KEY2    0x10    //按键 2 标记
#define KEY3    0x20    //按键 3 标记
#define KEY4    0x40    //按键 4 标记
#define NRFIQ   0x80    //无线中断标记
#define DISK1   0x100
#define DISK2   0x200
#define LOST    0x400
#define AUTO    0x800
#define AUTOA   0x1000
#define CNKT    0x2000

#define DOCONTR         (uchar)1
#define DOWTFLS         (uchar)2
#define DOUNLOCK        (uchar)3
#define DOLOCK          (uchar)4
#define DORST           (uchar)5
#define DOSENDPID       (uchar)6
#define DOREADPID       (uchar)7
#define DOCAPTURE       (uchar)8
#define DOCGTOAUTO      (uchar)9
#define DOCGTOMANU      (uchar)10


#define TA0US (uint)30000       //TA0定时时间(us)
#define TA1US (uint)5000        //TA1定时时间(us)

#define PID_TXBUF() txbuf[0]=DOSENDPID;\
  txbuf[11]=p1;txbuf[12]=i1;txbuf[13]=d1;\
    txbuf[14]=p2;txbuf[16]=d2;\
      txbuf[17]=KPH;txbuf[18]=KIH;txbuf[19]=KDH
        //PID参数更新至无线发射缓存
    
#define acc_x_tmp mpudat[0]
#define acc_y_tmp mpudat[1]
#define acc_z_tmp mpudat[2]
#define gyro_x_tmp mpudat[4]
#define gyro_y_tmp mpudat[5]  
#define gyro_z_tmp mpudat[6]  

#define q0 quatf[0]     //四元数计算
#define q1 quatf[1]
#define q2 quatf[2]
#define q3 quatf[3]


uint Mark = 0;          //标记记录
uint adc_tmp[5];        //AD滤波记录
int indi=0,indi_2=0,indi_3=0;   //
uchar rxbuf[RX_PLOAD_WIDTH]={0},txbuf[TX_PLOAD_WIDTH]={0};      //无线接收\发射缓存
int p1,i1,d1,p2,d2,KIH,KPH,KDH;                                    //PID参数寄存


void LCD_DisplayChange(void);


void main(void)         //main
{
  int point1=0,point2=0,point3=0,point4=0,point4_tmp=0;         //飞行器 4通道数据(油门+,Pitch+,Roll+,Yaw+)
  int tdelay=0;                                 //延时
  int ad_tmp1,ad_tmp2,ad_tmp3,ad_tmp4,avin=0;   //AD数据寄存  
  int Pitch,Roll,Yaw;                           //无线接收数据寄存:飞行器姿态
  int pwm1=0,pwm2=0,pwm3=0,pwm4=0,vot,high=0;   //无线接收数据寄存:电机转速,电池电压,飞行器高度
//  int* adj[7] = {&i2,&p1,&i1,&d1,&p2,&i2,&d2};        //LCD显示,选择显示(调整)变量地址
//  MPU6050_BUF mpudat;
//  int acc_x,acc_y,acc_z;
//  int gyro_x,gyro_y,gyro_z;
//  int acc_x_offset,acc_y_offset,acc_z_offset;
//  int gyro_x_offset,gyro_y_offset,gyro_z_offset;
  
  MPU6050DMPDATA;                               //MPU6050所需变量定义
  float quatf[4];                               //四元数
  int Pitch_c,Roll_c,Yaw_c,Yaw_tmp,Yaw_offst=0;       //
  uchar attd[32] = {0}; 
  
  Clock_Init();                         //时钟初始化,20mhz
  Timera0_Init(TA0US);                  //定时器A0初始化,0-65535(us)
  Timera1_Init(TA1US);                  //定时器A1初始化,0-65535(us)
  WDT_Init();   //delay for bell
  Uart1_Init();                         //串口1初始化,115200 
//  Uart1_Send_String("luyuexin");
  
  Lcd_Init();                           //5110LCD初始化
//  Lcd_String(36,5,"luyuexin");
  
  Led_Init();                           //led初始化
  Bell_Init();                          //蜂鸣器初始化
  Key_Init();Key_IE();                  //按键初始化
  
  Adc12_Init();                         //ADC初始化
  
//  delay_ms(500);
  while(mpu605dmp_init());                 //姿态传感器MPU6050初始化
  Led1_01();  
  Nrf_Init();                           //无线模块初始化
  
  Mark |= NRFIQ+ FSTA;//;               //标记初始
  Lcd_String(l12,4,"LC ");
  Lcd_Char(28,5,'m');
  LCD_DisplayChange(); 
  
  _BIS_SR(GIE);                         //打开总中断
  ///////////////////////
  while(1)              //while(1)
  {
    if(Mark&NRFIQ)              //无线中断事件发生
    {                                   
      Mark &=~NRFIQ;                    //清除中断标记
      Led1_1();
      if(RX_DR & Nrf_Check_Event(rxbuf))//有接收到数据
      {
        if(0x88==rxbuf[0])
        { 
          Roll = rxbuf[1]*256+ rxbuf[2];
          Pitch= rxbuf[3]*256+ rxbuf[4];
          Yaw  = rxbuf[5]*256+ rxbuf[6];
          high = rxbuf[7]*256+ rxbuf[8];          
          pwm1 = rxbuf[17];
          pwm2 = rxbuf[18];
          pwm3 = rxbuf[19];
          pwm4 = rxbuf[20];
          vot  = rxbuf[21];   
          
          if(rxbuf[23]&0x04){                           //lock??
            if(!(Mark&FSTA)){
              Mark |= FSTA; Led1_0();          
              Lcd_String(l12,4,"LC "); 
            }
          }
          else{
            if(Mark&FSTA){
              Mark &=~FSTA; Led1_0(); 
              Lcd_String(l12,4,"ULC");
            }
          }
          
          if(rxbuf[23]&0x80){
            if(!(Mark&AUTO)){
              Mark |= AUTO; Led1_0(); 
              point4 = high;
//              txbuf[1] = BYTE1(point4);         //txbuf
//              txbuf[2] = BYTE0(point4);
              LCD_DisplayChange();
            }
          }
          else{
            if(Mark&AUTO){
              Mark &=~AUTO; Led1_0(); 
              point4_tmp = (pwm1+ pwm2+ pwm3+ pwm4)/4*10- (2048- ad_tmp4)/8;
//              txbuf[1] = BYTE1(point4_tmp);         //txbuf
//              txbuf[2] = BYTE0(point4_tmp);
              LCD_DisplayChange();
            }
          }
          
          if(rxbuf[22]&0x01){                           //lost
            txbuf[0] = DOCAPTURE; Led1_0(); 
//            point3 = rxbuf[15]*256+ rxbuf[16];
          }
          
          if((rxbuf[22]&0x02)&&(Mark&AUTO)){                           //heigh error
            txbuf[0] = DOCGTOMANU; Led1_0(); 
          }
          
          Lcd_bar(12,5,(pwm1+12)/7);            //LCD显示更新
          Lcd_bar(14,5,(pwm2+12)/7);
          Lcd_bar(16,5,(pwm3+12)/7);
          Lcd_bar(18,5,(pwm4+12)/7);
          Lcd_Numh(l8,5,vot);
          Lcd_Numh(l8,4,high/10);          
          Lcd_Numh(l12,3,rxbuf[25]);
          Lcd_Num(0,1,Roll);                   
          Lcd_Num(0,2,Pitch);
          Lcd_Num(0,3,Yaw);
        }
        if(0x84==rxbuf[0])
        {                                       //接收到当前PID参数
          p1 = rxbuf[1];
          i1 = rxbuf[2];
          d1 = rxbuf[3];
          p2 = rxbuf[4];
//          i2 = rxbuf[5];
          d2 = rxbuf[6];
          KPH = rxbuf[7];
          KIH = rxbuf[8];
          KDH = rxbuf[9]; 
          LCD_DisplayChange();         
        }        
//        NRF_TxPacket_AP(txbuf);                 //发送新控制数据
//        txbuf[0] = DOCONTR;                     //txbuf[0] = 1 
//        Led1_01();                              
        USART1_SendDatas(rxbuf,32);             //将接收数据发送至电脑        
        if(!(Mark&CNKT))
          Mark |= CNKT;
      }                                 //end 有接收到数据
    }                           //end 无线中断事件发生   
//////////////////////////    
    
    dmp_read_fifo(gyro,accel,quatl,&sensors,&more);     //读取MPU6050DMP数据
    
    if((Mark&TA1Mark))          //定时器 1ms
    {                                           
      Mark &=~TA1Mark;      
                                        //ADC数据滤波,      
      ad_tmp4 = ((long)ad_tmp4*FPN+ (long)adc_tmp[0]*(100- FPN))/100;
      ad_tmp3 = ((long)ad_tmp3*FPN+ (long)adc_tmp[1]*(100- FPN))/100;
      ad_tmp2 = ((long)ad_tmp2*FPN+ (long)adc_tmp[2]*(100- FPN))/100;
      ad_tmp1 = ((long)ad_tmp1*FPN+ (long)adc_tmp[3]*(100- FPN))/100;
      avin = ((long)avin*FPN+ (long)adc_tmp[4]*(100- FPN))/100;
      
      Adc12_Start();                    //开启一次ADC转换     
    }                           //end 定时器 5ms    
////////////////////////////    
    if((Mark&TA0Mark))          //定时器 30ms
    {                                   //
      Mark &=~TA0Mark;                  //清除标记
      
      quatf[0] = quatl[0]/q30;  //四元数归一化
      quatf[1] = quatl[1]/q30;
      quatf[2] = quatl[2]/q30;
      quatf[3] = quatl[3]/q30;
      
      Roll_c  = (int)(asin(2*q1*q3- 2*q0*q2)*573.14);                     //roll -90~90
      
      Pitch_c = (int)(atan2(2*q2*q3+ 2*q0*q1,
                          -2*q1*q1- 2*q2*q2+ 1)*573.14);                //pitch -180~180
      Yaw_tmp = (int)(atan2(2*(q1*q2+ q0*q3),
                            q0*q0+ q1*q1- q2*q2- q3*q3)*573.14);        //Yaw -180~180
      Yaw_c =Yaw_tmp- Yaw_offst;  //776us
      
      if(!(Mark&CNKT)){
        Scop_update(attd,-Pitch_c,Roll_c,Yaw_c,0,0,0,0,0,0,0,0,0,0,0,0,
                    Roll_c,-gyro[1]/10,-accel[0]/10);
        USART1_SendDatas(attd,32);
      }  
///////////////////////      
      tdelay++;                         //软件延时(tdelay*TA0US)
      if(tdelay>6)                      //if(tdelay>5) 
      {
        tdelay = 0;                             //计数清零        
        if(Mark&FSTA)                           //lock
        {  
          if((Mark&KEY3)&&(!key3in)){                   //key3
            Mark &=~(KEY3);
            if(Mark&DISK1){                                     //display--1
              indi_2++;
              if(indi_2>5) indi_2 = 0;
            }
            else if(Mark&DISK2){                                //display--2
              indi_3++;
              if(indi_3>3) indi_3 = 0;
            }
            else{                                               //display--0
              indi++;
              if(indi>5) indi = 0; 
            }
            LCD_DisplayChange();                                //lcd change
          }                                             //end key3
          else if((Mark&KEY4)&&(!key4in)){              //key4
            Mark &=~(KEY4);            
            if(Mark&DISK1){                                     //display--1
              indi_2--;
              if(indi_2<0) indi_2 = 5;
            }
            else if(Mark&DISK2){                                //display--2
              indi_3--;
              if(indi_3<0) indi_3 = 3;
            }
            else{                                               //display--0
              indi--;
              if(indi<0) indi = 5; 
            } 
            LCD_DisplayChange();                                //lcd change     
          }                                             //end key4 
//////////////////////////          
          switch(indi){                                 //switch(indi)
          case 0:                                       //0                                              
            if((Mark&KEY1)&&(!key1in)){                         //do unlock
              Mark &=~(KEY1);               
              if(     //(point4<50)&&((point3<100)||(point3>-100))&&
                ((point2<100)||(point2>-100))&&((point1<100)||(point1>-100))) 
              {                
                if(Mark&AUTO){
                  point4 = high;
                }
                else{
                  point4_tmp = 0;              
                }
                Yaw_offst = Yaw_tmp;
                txbuf[0] = DOUNLOCK;
              }
            }                                                   //end do unlock
            else if((Mark&KEY2)&&(!key2in)){                         //auto change
              Mark &=~(KEY2);
              if(Mark&AUTO){
                txbuf[0] = DOCGTOMANU;
              }
              else{
                txbuf[0] = DOCGTOAUTO;
              }
              LCD_DisplayChange();
            }                                                   //end auto change
            break;
          case 1:                                       //1
            if((Mark&KEY2)&&(!key2in)){                         //clear cmd
              Mark &=~(KEY2);
              Yaw_offst = Yaw_tmp;
              txbuf[0] = DORST;
            }
            break;
          case 2:                                       //2       
            if(Mark&DISK1){                                     //if(Mark&DISK1)
              switch(indi_2){                                           //change pid  
              case 0:                                                           //p1  
                if(!key2in){
                  p1--;
                  Lcd_Numh(l10,0,p1);
                }
                else if(!key1in){
                  p1++;
                  Lcd_Numh(l10,0,p1);
                }                
                break;
              case 1:                                                           //i1  
                if(!key2in){
                  i1--;
                  Lcd_Numh(l10,0,i1);
                }
                else if(!key1in){
                  i1++;
                  Lcd_Numh(l10,0,i1);
                }                
                break;
              case 2:                                                           //d1               
                if(!key2in){
                  d1--;
                  Lcd_Numh(l10,0,d1);
                }
                else if(!key1in){
                  d1++;
                  Lcd_Numh(l10,0,d1);
                }
                break;
              case 3:                                                           //p2     
                if(!key2in){
                  p2--;
                  Lcd_Numh(l11,0,p2);
                }
                else if(!key1in){
                  p2++;
                  Lcd_Numh(l11,0,p2);
                }                
                break;
              case 4:                                                           //d2      
                if(!key2in){
                  d2--;
                  Lcd_Numh(l11,0,d2);
                }
                else if(!key1in){
                  d2++;
                  Lcd_Numh(l11,0,d2);
                }                
                break;
              case 5:                                                           //back
                if((Mark&KEY2)&&(!key2in)){
                  Mark &=~(KEY2);
                  Mark &=~DISK1;
                  PID_TXBUF();
                  Lcd_String(l7,0,">>     "); 
                }
                break;
              default:
                break;
              }                                                         //end change pid
            }                                                   
            else if((Mark&KEY2)&&(!key2in)){
              Mark &=~(KEY2);
              txbuf[0] = DOREADPID;
              Mark |= DISK1;
              indi_2 = 0;
              LCD_DisplayChange();
            }                                                   //end if(Mark&DISK1)        
            break;
          case 3:                                       //3
            if(Mark&DISK2){                                     //if(Mark&DISK2)
              switch(indi_3){                                           //change pidh
              case 0:                                                           //kph       
                if(!key2in){
                  KPH--;
                  Lcd_Numh(l11,0,KPH);
                }
                else if(!key1in){
                  KPH++;
                  Lcd_Numh(l11,0,KPH);
                }                
                break;
              case 1:                                                           //kih        
                if(!key2in){
                  KIH--;
                  Lcd_Numh(l11,0,KIH);
                }
                else if(!key1in){
                  KIH++;
                  Lcd_Numh(l11,0,KIH);
                }                
                break;
              case 2:                                                           //kdh   
                if(!key2in){
                  KDH--;
                  Lcd_Numh(l11,0,KDH);
                }
                else if(!key1in){
                  KDH++;
                  Lcd_Numh(l11,0,KDH);
                }                
                break;
              case 3:                                                           //back
                if((Mark&KEY2)&&(!key2in)){
                  Mark &=~(KEY2);
                  Mark &=~DISK2;
                  PID_TXBUF();
                  Lcd_String(l7,0,">>     "); 
                }
                break;
              default:
                break;
              }                                                         //end change pidh
            }
            else if((Mark&KEY2)&&(!key2in)){
              Mark &=~(KEY2);
              txbuf[0] = DOREADPID;
              Mark |= DISK2;
              indi_3 = 0;
              LCD_DisplayChange();
            }                                                   //end if(Mark&DISK2)
            break;
          case 4:                                       //4
            if((Mark&KEY2)&&(!key2in)){                         //flash w
              Mark &=~(KEY2);
              txbuf[0] = DOWTFLS;
            }
            break;
          case 5:
            if((Mark&KEY2)&&(!key2in)){                         //flash w
              Mark &=~(KEY2);
              if(Mark&AUTOA){
                Mark &=~AUTOA;
                Lcd_Char(28,5,'m');
              }
              else{
                Mark |= AUTOA;
                Lcd_Char(28,5,'a');
              }
            }
            break;
          default:
            break;
          }                                             //end switch(indi) 
        }                                       //end lock
////////////////////        
        else                                    //unlock
        {
          Bell(1);                                      //bell 
          if((Mark&KEY3)&&(!key3in))                    //do lock
          {
            Mark &=~(KEY3);
            txbuf[0] = DOLOCK;
          }
          if((Mark&KEY4)&&(!key4in)){                   //change auto
            Mark &=~(KEY4);
            if(Mark&AUTO){ 
              txbuf[0] = DOCGTOMANU;  
            }
            else{
              txbuf[0] = DOCGTOAUTO;
            }
            LCD_DisplayChange();
          }                                             //end change auto
          if(!key1in)                                   //key1
          {
            if(Mark&AUTO)
              Yaw_offst += 20;
            else
              point4_tmp -= 10;
          }                                             //end key1
          else if(!key2in)                              //key2
          {
            if(Mark&AUTO)
              Yaw_offst -= 20;
            else
              point4_tmp += 10;
          }                                             //end key2          
        }                                       //end unlock
      }                                 //end if(tdelay>5) 
////////////////////////////      
      if(Mark&AUTO){                    //point4
        point4 += (2048- ad_tmp4)/256;
        if(point4<0){
          point4 = 0;
        }
        if(point4>3072){
          point4 = 3072;
        }
        Lcd_bar(8,5,point4/192);
      }
      else{
        point4 = point4_tmp+ (2048- ad_tmp4)/16;
        if(point4<0){
          point4 = 0;
        }
        if(point4>1000){
          point4 = 1000;
        }        
        Lcd_bar(8,5,point4/62);      
      }                                 //end point4  
      if(Mark&AUTOA){
        point1 = Roll_c/2;
        point2 = -Pitch_c/2; 
      }
      else{
        point1 = (ad_tmp2- 1958)/17;    //point1   
        point2 = (ad_tmp1- 2143)/17;    //point2    
      }
      point3 = Yaw_c;                   //point3
      
      
      txbuf[1] = BYTE1(point4);         //txbuf
      txbuf[2] = BYTE0(point4);
      txbuf[3] = BYTE1(point3);
      txbuf[4] = BYTE0(point3);
      txbuf[5] = BYTE1(point2);
      txbuf[6] = BYTE0(point2);
      txbuf[7] = BYTE1(point1);
      txbuf[8] = BYTE0(point1);
      NRF_TxPacket(txbuf);
      txbuf[0] = DOCONTR;
      //发送新控制数据
//      if((avin<=1944)||(vot<=110)){
//        if(!(SFRIE1&WDTIE))
//          Bell_IE_start();
//      }
//      else if(SFRIE1&WDTIE){
//        Bell_IE_end();
//      } 
      
//      Lcd_Numh(l12,1,point3/10);        //lcd  
      Lcd_Numh(l8,3,point4/10);    
      Lcd_Numh(l12,5,avin/27);     
      Lcd_bar(0,5,16);
      Lcd_bar(20,5,1);
      Lcd_bar(10,5,1);
      Lcd_bar(2,5,8+point1/32);      
      Lcd_bar(4,5,8-point2/32);
      Lcd_bar(6,5,8+point3/225);
      Lcd_bar(22,5,8-gyro[1]/128);
      Lcd_bar(24,5,8+gyro[0]/128);
      Lcd_bar(26,5,8+gyro[2]/128);
//      Lcd_Num(0,3,MPU6050_GetData(GYRO_XOUT_H)); 
    }                           //end 定时器 30ms
  }                     //end while(1)
}               //end main



void LCD_DisplayChange(void)
{
  Lcd_String(0,0,"              ");     
  switch(indi){                                       //lcd switch
  case 0:                                                     //0 
    Lcd_String(0,0,"Unlock");                                 
    if(Mark&AUTO)
      Lcd_String(l8,0,"AUTO");
    else
      Lcd_String(l8,0,"MANU");
    break;
  case 1:                                                     //1
    Lcd_String(0,0,"Clear");
    break;
  case 2:                                                     //2
    Lcd_String(0,0,"PID_R");
    if(Mark&DISK1){                                           //lcd display--1                                                  
      switch(indi_2){                                                 //lcd display--1 swtich                                       
      case 0:                                                         
        Lcd_String(l7,0,"KP:"); 
        Lcd_Numh(l10,0,p1);
        break;
      case 1:
        Lcd_String(l7,0,"KI:"); 
        Lcd_Numh(l10,0,i1); 
        break;
      case 2:
        Lcd_String(l7,0,"KD:"); 
        Lcd_Numh(l10,0,d1); 
        break;
      case 3:
        Lcd_String(l7,0,"KPZ:"); 
        Lcd_Numh(l11,0,p2);
        break;
      case 4:
        Lcd_String(l7,0,"KDZ:");  
        Lcd_Numh(l11,0,d2);
        break;
      case 5:
        Lcd_String(l7,0,"BACK <<"); 
        break;
      default:
        break;
      }                                                               //end lcd display--1 swtich 
    }
    else{
      Lcd_String(l7,0,">>     ");     
    }                                                         //end lcd display--1  
    break;
  case 3:                                                     //3
    Lcd_String(0,0,"K_H_R");            
    if(Mark&DISK2){                                           //lcd display--2
      switch(indi_3){                                                 //lcd display--2 switch
      case 0:
        Lcd_String(l7,0,"KPH:"); 
       Lcd_Numh(l11,0,KPH); 
        break;
      case 1:
        Lcd_String(l7,0,"KIH:"); 
       Lcd_Numh(l11,0,KIH); 
        break;
      case 2:
        Lcd_String(l7,0,"KDH:");  
       Lcd_Numh(l11,0,KDH); 
        break;
      case 3:
        Lcd_String(l7,0,"BACK <<"); 
        break;
      default:
        break;
      }                                                               //end lcd display--2 switch
    }
    else{
      Lcd_String(l7,0,">>     ");     
    }                                                         //end lcd display--2
    break;
  case 4:                                                     //4
    Lcd_String(0,0,"Flash_Write");   
    break;
  case 5:
    Lcd_String(0,0,"ATTDCC");  
    break;
  default:
    break;
  }                                                   //end lcd switch  
}

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
uchar urat_temp;
#pragma vector=USCI_A1_VECTOR
__interrupt void uart1(void)
{  
  if(UCA1IFG&UCRXIFG)
  {
    urat_temp = UCA1RXBUF;
    Led1_01();
//    Uart1_Send_Char(urat_temp);
    Bell(1);
    UCA1IFG &= ~UCRXIFG;
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

//p1中断
#pragma vector = PORT1_VECTOR 
__interrupt void Port1(void)
{  
  if(P1IFG&key1){
    Mark |= KEY1;
    P1IFG &=~key1;
  }
  if(P1IFG&key2){
    Mark |= KEY2;
    P1IFG &=~key2;
  }
  if(P1IFG&key3){
    Mark |= KEY3;
    P1IFG &=~key3;
  }
  if(P1IFG&key4){
    Mark |= KEY4;
    P1IFG &=~key4;
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
  case 14:                                  // Vector 14:  ADC12IFG4
    adc_tmp[0] = ADC12MEM0;                 // Move results, IFG is cleared
    adc_tmp[1] = ADC12MEM1;                 // Move results, IFG is cleared
    adc_tmp[2] = ADC12MEM2;                 // Move results, IFG is cleared
    adc_tmp[3] = ADC12MEM3;                 // Move results, IFG is cleared
    adc_tmp[4] = ADC12MEM4;
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

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
  Bell_IE();
}

        /*  if(!key3in){
            Mark &=~(KEY3);
            indi++;
            if(indi>6) indi = 0;            
          }
          else if(ad_tmp1<1024){//0 1024 2048 3072 4096          
            indi++;
            if(indi>6) indi = 0; 
          }
          else if(ad_tmp1>3027){
            indi--;
            if(indi<0) indi = 6; 
          }
///////////////////////          
          if(!key2in){
            Mark &=~(KEY2);
            adj[indi][0]++;           
          }
          else if(!key1in){
            Mark &=~(KEY1);
            adj[indi][0]--;           
          }
          else if(ad_tmp2>3027){
            adj[indi][0]++; 
          }
          else if(ad_tmp2<1024){
            adj[indi][0]--; 
          }
//////////////////
          Lcd_Numh(l8,0,p1);
          Lcd_Numh(l8,1,i1);
          Lcd_Numh(l8,2,d1);
          Lcd_Numh(l12,0,p2);
          Lcd_Numh(l12,1,i2);
          Lcd_Numh(l12,2,d2);      
          Lcd_Char(l7,0,' ');
          Lcd_Char(l7,1,' ');
          Lcd_Char(l7,2,' ');
          Lcd_Char(l11,0,' ');
          Lcd_Char(l11,1,' ');
          Lcd_Char(l11,2,' ');
          Lcd_Char(l13,3,' ');       
          switch(indi){
          case 0:            
            Lcd_Char(l13,3,'>');
            switch((uchar)i2%4)
            {
            case 0:
              Lcd_Char(l14,3,'E');
              break;
            case 1:
              Lcd_Char(l14,3,'P');
              break;
            case 2:
              Lcd_Char(l14,3,'W');
              break;
            case 3:
              Lcd_Char(l14,3,'S');
              break;
            default:
              break;
            } 
            break;
          case 1:            
            Lcd_Char(l7,0,'>');
            break;
          case 2:           
            Lcd_Char(l7,1,'>');
            break;
          case 3:
            Lcd_Char(l7,2,'>');
            break;
          case 4:
            Lcd_Char(l11,0,'>');
            break;
          case 5:
            Lcd_Char(l11,1,'>');
            break;           
          case 6:
            Lcd_Char(l11,2,'>');
            break;
          default:
            break;
          }
          
          if((Mark&KEY4)&&(!key4in))
          {
            Mark &=~(KEY4);
            switch(indi){ 
            case 0:
              switch((uchar)i2%4)
              {
              case 0:
                if((point4<50)&&((point3<100)||(point3>-100))
                  &&((point2<100)||(point2>-100))&&((point1<100)||(point1>-100)))
                {
                  txbuf[0] = 3;
                  Mark &=~(FSTA);
                } 
                break;
              case 1:
                txbuf[0] = 7;
                break;
              case 2:
                txbuf[0] = 2;
                break;
              case 3:
                txbuf[0] = 5;
                point3 = 0;
                break;
              default:
                break;
              }              
              break;           
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
              PID_TXBUF();
              break;            
            default:
              break;
            }
          }*/

