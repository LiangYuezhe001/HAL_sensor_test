#include "main.h"


#define HSCDTD_REG_SELFTEST_RESP        0x0C //
#define HSCDTD_REG_MORE_INFO            0x0D
#define HSCDTD_REG_MORE_INFO_ALPS       0x0E
#define HSCDTD_REG_WIA                  0x0F
#define HSCDTD_REG_XOUT_L               0x10
#define HSCDTD_REG_XOUT_H               0x11
#define HSCDTD_REG_YOUT_L               0x12
#define HSCDTD_REG_YOUT_H               0x13
#define HSCDTD_REG_ZOUT_L               0x14
#define HSCDTD_REG_ZOUT_H               0x15

#define HSCDTD_REG_STATUS               0x18
#define HSCDTD_REG_FIFO_P_STATUS        0x19
#define HSCDTD_REG_CTRL1                0x1B
#define HSCDTD_REG_CTRL2                0x1C
#define HSCDTD_REG_CTRL3                0x1D
#define HSCDTD_REG_CTRL4                0x1E

#define HSCDTD_REG_OFFSET_X_L           0x20
#define HSCDTD_REG_OFFSET_X_H           0x21
#define HSCDTD_REG_OFFSET_Y_L           0x22
#define HSCDTD_REG_OFFSET_Y_H           0x23
#define HSCDTD_REG_OFFSET_Z_L           0x24
#define HSCDTD_REG_OFFSET_Z_H           0x25
#define HSCDTD_REG_INTR_THR_L           0x26
#define HSCDTD_REG_INTR_THR_H           0x27

#define HSCDTD_REG_TEMP                 0x31


u8 MAG_Init(void); 								//初始化MAG6050
u8 MAG_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC连续写
u8 MAG_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC连续读 
u8 MAG_Write_Byte(u8 reg,u8 data);				//IIC写一个字节
u8 MAG_Read_Byte(u8 reg);						//IIC读一个字节

u8 MAG_Set_Gyro_Fsr(u8 fsr);
u8 MAG_Set_Accel_Fsr(u8 fsr);
u8 MAG_Set_LPF(u16 lpf);
u8 MAG_Set_Rate(u16 rate);
u8 MAG_Set_Fifo(u8 sens);
u8 Get_Gyro(float* gyro);
u8 Get_Acc(float* acc);
