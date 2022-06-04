#ifndef __MPUIIC_H
#define __MPUIIC_H
//#include "sys.h"
#include "main.h"
	   		   
//IO��������
#define MPU_SDA_IN()   {GPIOB->CRH&=0XFFF0FFFF;GPIOB->CRH|=8<<16;}
#define MPU_SDA_OUT()  {GPIOB->CRH&=0XFFF0FFFF;GPIOB->CRH|=3<<16;}

//IO��������	 
#define MPU_IIC_SCL     PBout(14) //SCL
#define MPU_IIC_SDA     PBout(12) //SDA	
#define MPU_READ_SDA   	PBin(12) //SDA	


//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�
void iicsearch(void);

u8 i2cdevReadByte(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data);
u8 i2cdevReadBit(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data);
u8 i2cdevWriteByte(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t data);
u8 i2cdevWrite(u8 dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);
u8 i2cdevRead(u8 dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);
u8 i2cdevWriteBit(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data);
u8 i2cdevWriteBits(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















