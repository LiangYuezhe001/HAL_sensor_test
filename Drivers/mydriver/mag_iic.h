#include "main.h"
	   		   
//IO��������
#define MAG_SDA_IN()   {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=8<<20;}
#define MAG_SDA_OUT()  {GPIOC->CRH&=0XFF0FFFFF;GPIOC->CRH|=3<<20;}

//IO��������	 
#define MAG_IIC_SCL     PCout(0) //SCL
#define MAG_IIC_SDA     PCout(13) //SDA	
#define MAG_READ_SDA   	PCin(13) //SDA	

void MAG_IIC_Delay(void);				//MAG IIC��ʱ����
void MAG_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MAG_IIC_Start(void);				//����IIC��ʼ�ź�
void MAG_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MAG_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 MAG_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MAG_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MAG_IIC_Ack(void);					//IIC����ACK�ź�
void MAG_IIC_NAck(void);				//IIC������ACK�ź�
u8 MAG_Read_Byte(u8 reg);
u8 MAG_Write_Byte(u8 reg, u8 data);
