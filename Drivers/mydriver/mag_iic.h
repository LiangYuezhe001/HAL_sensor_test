#include "main.h"
	   		   
//IO方向设置
#define MAG_SDA_IN()   {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=8<<20;}
#define MAG_SDA_OUT()  {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=3<<20;}

//IO操作函数	 
#define MAG_IIC_SCL     PCout(0) //SCL
#define MAG_IIC_SDA     PCout(13) //SDA	
#define MAG_READ_SDA   	PCin(13) //SDA	

void MAG_IIC_Delay(void);				//MAG IIC延时函数
void MAG_IIC_Init(void);                //初始化IIC的IO口				 
void MAG_IIC_Start(void);				//发送IIC开始信号
void MAG_IIC_Stop(void);	  			//发送IIC停止信号
void MAG_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MAG_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MAG_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MAG_IIC_Ack(void);					//IIC发送ACK信号
void MAG_IIC_NAck(void);				//IIC不发送ACK信号
u8 MAG_Read_Byte(u8 reg);
u8 MAG_Write_Byte(u8 reg, u8 data);
