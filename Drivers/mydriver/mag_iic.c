#include "mag_iic.h"
#include "tim.h"


void MAG_IIC_Delay(void)
{
	delay_us(3);
}

void MAG_IIC_Start(void)
{
	MAG_SDA_OUT(); // sda线输出
	MAG_IIC_SDA = 1;
	MAG_IIC_SCL = 1;
	MAG_IIC_Delay();
	MAG_IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
	MAG_IIC_Delay();
	MAG_IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void MAG_IIC_Stop(void)
{
	MAG_SDA_OUT(); // sda线输出
	MAG_IIC_SCL = 0;
	MAG_IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
	MAG_IIC_Delay();
	MAG_IIC_SCL = 1;
	MAG_IIC_SDA = 1; //发送I2C总线结束信号
	MAG_IIC_Delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MAG_IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	MAG_SDA_IN(); // SDA设置为输入
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
	MAG_IIC_SCL = 0; //时钟输出0
	return 0;
}
//产生ACK应答
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
//不产生ACK应答
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
// IIC发送一个字节
//返回从机有无应答
// 1，有应答
// 0，无应答
void MAG_IIC_Send_Byte(u8 txd)
{
	u8 t;
	MAG_SDA_OUT();
	MAG_IIC_SCL = 0; //拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 MAG_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	MAG_SDA_IN(); // SDA设置为输入
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
		MAG_IIC_NAck(); //发送nACK
	else
		MAG_IIC_Ack(); //发送ACK
	return receive;
}

//u8 i2cdevReadByte(u8 dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data)
//{
//	u8 res;
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //发送器件地址+写命令
//	MAG_IIC_Wait_Ack();						//等待应答
//	MAG_IIC_Send_Byte(memAddress);					//写寄存器地址
//	MAG_IIC_Wait_Ack();						//等待应答
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 1); //发送器件地址+读命令
//	MAG_IIC_Wait_Ack();						//等待应答
//	*data = MAG_IIC_Read_Byte(0);				//读取数据,发送nACK
//	MAG_IIC_Stop();							//产生一个停止条件
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
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //发送器件地址+写命令
//	if (MAG_IIC_Wait_Ack())					//等待应答
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress);	 //写寄存器地址
//	MAG_IIC_Wait_Ack();		 //等待应答
//	MAG_IIC_Send_Byte(data); //发送数据
//	if (MAG_IIC_Wait_Ack())	 //等待ACK
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
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //发送器件地址+写命令
//	if (MAG_IIC_Wait_Ack())				//等待应答
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress); //写寄存器地址
//	MAG_IIC_Wait_Ack();		//等待应答
//	for (i = 0; i < len; i++)
//	{
//		MAG_IIC_Send_Byte(data[i]); //发送数据
//		if (MAG_IIC_Wait_Ack())	   //等待ACK
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
//	MAG_IIC_Send_Byte((devAddress << 1) | 0); //发送器件地址+写命令
//	if (MAG_IIC_Wait_Ack())				//等待应答
//	{
//		MAG_IIC_Stop();
//		return 1;
//	}
//	MAG_IIC_Send_Byte(memAddress); //写寄存器地址
//	MAG_IIC_Wait_Ack();		//等待应答
//	MAG_IIC_Start();
//	MAG_IIC_Send_Byte((devAddress << 1) | 1); //发送器件地址+读命令
//	MAG_IIC_Wait_Ack();					//等待应答
//	while (len)
//	{
//		if (len == 1)
//			*data = MAG_IIC_Read_Byte(0); //读数据,发送nACK
//		else
//			*data = MAG_IIC_Read_Byte(1); //读数据,发送ACK
//		len--;
//		data++;
//	}
//	MAG_IIC_Stop(); //产生一个停止条件
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


 