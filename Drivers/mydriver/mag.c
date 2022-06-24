#include "MAG_IIC.h"
#include "sys.h"
//#include "delay.h"
#include "usart.h"
#include <math.h>
#include "tim.h"
#include "mag.h"

u8 MAG_Init(void)
{
	HSCDTD_CTRL3_t reg;
	reg.SRST = 1;

	///////reset mag sensor//////////////
	MAG_Write_Byte(HSCDTD_REG_CTRL3, reg); //?? reg3
	delay_ms(5);
	reg = MAG_Read_Byte(HSCDTD_REG_CTRL3);
	if (reg.SRST == 1)
	{

		return 1;
	}

	
}
//����MAG6050�����Ǵ����������̷�Χ
// fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MAG_Set_Gyro_Fsr(u8 fsr)
{
	return MAG_Write_Byte(MAG_GYRO_CFG_REG, fsr << 3); //���������������̷�Χ
}
//����MAG6050���ٶȴ����������̷�Χ
// fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MAG_Set_Accel_Fsr(u8 fsr)
{
	return MAG_Write_Byte(MAG_ACCEL_CFG_REG, fsr << 3); //���ü��ٶȴ����������̷�Χ
}
//����MAG6050�����ֵ�ͨ�˲���
// lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MAG_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MAG_Write_Byte(MAG_CFG_REG, data); //�������ֵ�ͨ�˲���
}
//����MAG6050�Ĳ�����(�ٶ�Fs=1KHz)
// rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MAG_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MAG_Write_Byte(MAG_SAMPLE_RATE_REG, data); //�������ֵ�ͨ�˲���
	return MAG_Set_LPF(rate / 2);					  //�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MAG_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	MAG_Read_Len(MAG_ADDR, MAG_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
	;
}
//�õ�������ֵ(ԭʼֵ)
// gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MAG_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = MAG_Read_Len(MAG_ADDR, MAG_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
	;
}
u8 Get_Gyro(float *gyro)
{
	u8 buf[6], res;
	short raw_gyro[3];
	res = MAG_Read_Len(MAG_ADDR, MAG_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		raw_gyro[0] = ((u16)buf[0] << 8) | buf[1];
		raw_gyro[1] = ((u16)buf[2] << 8) | buf[3];
		raw_gyro[2] = ((u16)buf[4] << 8) | buf[5];
	}
	gyro[0] = (float)raw_gyro[0] / 32767 * 250;
	gyro[1] = (float)raw_gyro[1] / 32767 * 250;
	gyro[2] = (float)raw_gyro[2] / 32767 * 250;
	return res;
	;
}
//�õ����ٶ�ֵ(ԭʼֵ)
// gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MAG_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = MAG_Read_Len(MAG_ADDR, MAG_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
	;
}

u8 Get_Acc(float *acc)
{
	u8 buf[6], res;
	short raw_acc[3];
	res = MAG_Read_Len(MAG_ADDR, MAG_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		raw_acc[0] = ((u16)buf[0] << 8) | buf[1];
		raw_acc[1] = ((u16)buf[2] << 8) | buf[3];
		raw_acc[2] = ((u16)buf[4] << 8) | buf[5];
	}
	acc[0] = (float)raw_acc[0] / 32767 * 2;
	acc[1] = (float)raw_acc[1] / 32767 * 2;
	acc[2] = (float)raw_acc[2] / 32767 * 2;
	return res;
	;
}

// IIC����д
// addr:������ַ
// reg:�Ĵ�����ַ
// len:д�볤��
// buf:������
//����ֵ:0,����
//     ����,�������
u8 MAG_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (MAG_IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		MAG_IIC_Stop();
		return 1;
	}
	MAG_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	MAG_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		MAG_IIC_Send_Byte(buf[i]); //��������
		if (MAG_IIC_Wait_Ack())	   //�ȴ�ACK
		{
			MAG_IIC_Stop();
			return 1;
		}
	}
	MAG_IIC_Stop();
	return 0;
}
// IIC������
// addr:������ַ
// reg:Ҫ��ȡ�ļĴ�����ַ
// len:Ҫ��ȡ�ĳ���
// buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//     ����,�������
u8 MAG_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (MAG_IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		MAG_IIC_Stop();
		return 1;
	}
	MAG_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	MAG_IIC_Wait_Ack();		//�ȴ�Ӧ��
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
	MAG_IIC_Wait_Ack();					//�ȴ�Ӧ��
	while (len)
	{
		if (len == 1)
			*buf = MAG_IIC_Read_Byte(0); //������,����nACK
		else
			*buf = MAG_IIC_Read_Byte(1); //������,����ACK
		len--;
		buf++;
	}
	MAG_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}
// IICдһ���ֽ�
// reg:�Ĵ�����ַ
// data:����
//����ֵ:0,����
//     ����,�������
u8 MAG_Write_Byte(u8 reg, u8 data)
{
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((MAG_ADDR << 1) | 0); //����������ַ+д����
	if (MAG_IIC_Wait_Ack())					//�ȴ�Ӧ��
	{
		MAG_IIC_Stop();
		return 1;
	}
	MAG_IIC_Send_Byte(reg);	 //д�Ĵ�����ַ
	MAG_IIC_Wait_Ack();		 //�ȴ�Ӧ��
	MAG_IIC_Send_Byte(data); //��������
	if (MAG_IIC_Wait_Ack())	 //�ȴ�ACK
	{
		MAG_IIC_Stop();
		return 1;
	}
	MAG_IIC_Stop();
	return 0;
}
// IIC��һ���ֽ�
// reg:�Ĵ�����ַ
//����ֵ:����������
u8 MAG_Read_Byte(u8 reg)
{
	u8 res;
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((MAG_ADDR << 1) | 0); //����������ַ+д����
	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
	MAG_IIC_Send_Byte(reg);					//д�Ĵ�����ַ
	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
	MAG_IIC_Start();
	MAG_IIC_Send_Byte((MAG_ADDR << 1) | 1); //����������ַ+������
	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
	res = MAG_IIC_Read_Byte(0);				//��ȡ����,����nACK
	MAG_IIC_Stop();							//����һ��ֹͣ����
	return res;
}
