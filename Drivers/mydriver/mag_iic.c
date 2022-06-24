#include "mag_iic.h"
#include "tim.h"


void MAG_IIC_Delay(void)
{
	delay_us(3);
}

void MAG_IIC_Start(void)
{
	MAG_SDA_OUT(); // sda�����
	MAG_IIC_SDA = 1;
	MAG_IIC_SCL = 1;
	MAG_IIC_Delay();
	MAG_IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
	MAG_IIC_Delay();
	MAG_IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void MAG_IIC_Stop(void)
{
	MAG_SDA_OUT(); // sda�����
	MAG_IIC_SCL = 0;
	MAG_IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
	MAG_IIC_Delay();
	MAG_IIC_SCL = 1;
	MAG_IIC_SDA = 1; //����I2C���߽����ź�
	MAG_IIC_Delay();
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MAG_IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	MAG_SDA_IN(); // SDA����Ϊ����
	MAG_IIC_SDA = 1;
	MAG_IIC_Delay();
	MAG_IIC_SCL = 1;
	MAG_IIC_Delay();
	while (MAG_READ_SDA)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			MAG_IIC_Stop();
			return 1;
		}
	}
	MAG_IIC_SCL = 0; //ʱ�����0
	return 0;
}
//����ACKӦ��
void MAG_IIC_Ack(void)
{
	MAG_IIC_SCL = 0;
	MAG_SDA_OUT();
	MAG_IIC_SDA = 0;
	MAG_IIC_Delay();
	MAG_IIC_SCL = 1;
	MAG_IIC_Delay();
	MAG_IIC_SCL = 0;
}
//������ACKӦ��
void MAG_IIC_NAck(void)
{
	MAG_IIC_SCL = 0;
	MAG_SDA_OUT();
	MAG_IIC_SDA = 1;
	MAG_IIC_Delay();
	MAG_IIC_SCL = 1;
	MAG_IIC_Delay();
	MAG_IIC_SCL = 0;
}
// IIC����һ���ֽ�
//���شӻ�����Ӧ��
// 1����Ӧ��
// 0����Ӧ��
void MAG_IIC_Send_Byte(u8 txd)
{
	u8 t;
	MAG_SDA_OUT();
	MAG_IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	for (t = 0; t < 8; t++)
	{
		MAG_IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		MAG_IIC_SCL = 1;
		MAG_IIC_Delay();
		MAG_IIC_SCL = 0;
		MAG_IIC_Delay();
	}
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 MAG_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	MAG_SDA_IN(); // SDA����Ϊ����
	for (i = 0; i < 8; i++)
	{
		MAG_IIC_SCL = 0;
		MAG_IIC_Delay();
		MAG_IIC_SCL = 1;
		receive <<= 1;
		if (MAG_READ_SDA)
			receive++;
		MAG_IIC_Delay();
	}
	if (!ack)
		MAG_IIC_NAck(); //����nACK
	else
		MAG_IIC_Ack(); //����ACK
	return receive;
}

//u8 i2cdevReadByte(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data)
//{
//	u8 res;
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //����������ַ+д����
//	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
//	MAG_IIC_Send_Byte(memAddress);					//д�Ĵ�����ַ
//	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 1); //����������ַ+������
//	MAG_IIC_Wait_Ack();						//�ȴ�Ӧ��
//	*data = MAG_IIC_Read_Byte(0);				//��ȡ����,����nACK
//	MAG_IIC_Stop();							//����һ��ֹͣ����
//	return 1;
//}

//u8 i2cdevReadBit(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data)
//{
//	uint8_t byte;
//	u8 status;

//	status = i2cdevRead(dev, devAddress, memAddress, 1, &byte);
//	*data = byte & (1 << bitNum);

//	return status;
//}

//u8 i2cdevWriteByte(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t data)
//{
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //����������ַ+д����
//	if (MAG_IIC_Wait_Ack())					//�ȴ�Ӧ��
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress);	 //д�Ĵ�����ַ
//	MAG_IIC_Wait_Ack();		 //�ȴ�Ӧ��
//	MAG_IIC_Send_Byte(data); //��������
//	if (MAG_IIC_Wait_Ack())	 //�ȴ�ACK
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Stop();
//	return 0;
//}

//u8 i2cdevWrite(u8 dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
//{
//	u8 i;
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //����������ַ+д����
//	if (MAG_IIC_Wait_Ack())				//�ȴ�Ӧ��
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress); //д�Ĵ�����ַ
//	MAG_IIC_Wait_Ack();		//�ȴ�Ӧ��
//	for (i = 0; i < len; i++)
//	{
//		MAG_IIC_Send_Byte(data[i]); //��������
//		if (MAG_IIC_Wait_Ack())	   //�ȴ�ACK
//		{
//			MAG_IIC_Stop();
//			return 1;
//		}
//	}
//	MAG_IIC_Stop();
//	return 0;
//}


//u8 i2cdevRead(u8 dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
//{
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //����������ַ+д����
//	if (MAG_IIC_Wait_Ack())				//�ȴ�Ӧ��
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress); //д�Ĵ�����ַ
//	MAG_IIC_Wait_Ack();		//�ȴ�Ӧ��
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 1); //����������ַ+������
//	MAG_IIC_Wait_Ack();					//�ȴ�Ӧ��
//	while (len)
//	{
//		if (len == 1)
//			*data = MAG_IIC_Read_Byte(0); //������,����nACK
//		else
//			*data = MAG_IIC_Read_Byte(1); //������,����ACK
//		len--;
//		data++;
//	}
//	MAG_IIC_Stop(); //����һ��ֹͣ����
//	return 0;
//}

//u8 i2cdevWriteBit(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data)
//{
//	uint8_t byte;
//	i2cdevReadByte( dev,  devAddress,  memAddress,  &byte);
//	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
//	return i2cdevWriteByte(dev, devAddress, memAddress, byte);
//}

//u8 i2cdevWriteBits(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data)
//{
//	u8 status;
//	uint8_t byte;

//	if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == 1)
//	{
//		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//		data <<= (bitStart - length + 1); // shift data into correct position
//		data &= mask; // zero all non-important bits in data
//		byte &= ~(mask); // zero all important bits in existing byte
//		byte |= data; // combine data with existing byte
//		status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
//	}

//	return status;
//}


 