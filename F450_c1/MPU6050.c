
#include "MPU6050.h"



//初始化MPU6050
//**************************************
uint InitMPU6050()
{
#if defined __UCB_I2C_H
  UCB_I2CInit();
#endif
  if(Single_WriteI2C(SlaveAddr,PWR_MGMT_1, 0x00))	//解除休眠状态
    return 1;
  delay_ms(200);
  if(Single_WriteI2C(SlaveAddr,PWR_MGMT_1, 0x01))	//时钟
    return 1;
  delay_ms(50);
  if(Single_WriteI2C(SlaveAddr,CONFIGmpu, 0x03))        //42hz滤波
    return 1;
  delay_ms(50);
  if(Single_WriteI2C(SlaveAddr,GYRO_CONFIG, 0x18))      //2000'
    return 1;
  delay_ms(50);
  if(Single_WriteI2C(SlaveAddr,ACCEL_CONFIG, 0x00))     //2g
    return 1;
  return 0;
}

//读取一个数据
/******************************************************************************/
int MPU6050_GetData(uchar REG_Address)
{
  int dat;
  dat=Single_ReadI2C(SlaveAddr,REG_Address);
  dat <<= 8;
  dat |= Single_ReadI2C(SlaveAddr,REG_Address+1);
  return dat;   //合成数据
}

//数据更新
/******************************************************************************/
void MPU6050_Updata(MPU6050_BUF mpudat)
{
  uchar dat[14],i;
  Multiple_readI2C(SlaveAddr,dat,ACCEL_XOUT_H,14);
  for(i=0;i<7;i++)
  {
    mpudat[i] = dat[2*i];
    mpudat[i] <<= 8;
    mpudat[i] |= dat[2*i+1];
  }
//  mpudat[3] = (mpudat[3]+ 13200)/ 280;
}




/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test()
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP. == 0x7
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    }
    else
    {
        return 1;
    }
}




static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};



//dmp初始化
/******************************************************************************/
uint mpu605dmp_init()
{
//  uint result = 0;
#if defined __UCB_I2C_H
  UCB_I2CInit();
#endif
  if(mpu_init()){
//    result |= 1;
    return 1;
  }
  if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
//    result |= 2 
    return 1;
  if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    return 1;
//  result |= 4;
  if(mpu_set_sample_rate(DEFAULT_MPU_HZ))
    return 1;
//  result |= 8;
  if(dmp_load_motion_driver_firmware())
    return 1;
//    result |= 0x10;
  if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
    return 1;
//  result |= 0x20;
  if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
                  DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
    return 1;
//  result |= 0x40;
  if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))
    return 1;
//  result |= 0x80;
  if(run_self_test())
    return 1;
//  result |= 0x100;
  if(mpu_set_dmp_state(1))
    return 1;
//  result |= 0x200;
  return 0;
//  return result;
}


































