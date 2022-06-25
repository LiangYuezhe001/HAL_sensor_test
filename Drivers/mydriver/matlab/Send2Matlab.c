#include "main.h"
#include "usart.h"
#include "time.h"
#include "math.h"
#include "Kalman_Filter.h"
#include "math.h"
#include "ANO_DT.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "vl53lxx.h"
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
u8 data_matlab[27];

void Send_Data(u8 *dataToSend , u8 length)
{
	Usart_Send(data_matlab, length);	
}

//void Send_Status(double a, double b, double c)
//{
//	u8 _cnt=0;
//	vs16 _temp;
//	//data_matlab[_cnt++]=0xaa;
//	void* p = &data_matlab[_cnt];
//	memcpy(p,&a,8);
//	data_matlab[_cnt++]=13;
//	data_matlab[_cnt++]=10;
//	_cnt+=8;
//	p = &data_matlab[_cnt];
//	memcpy(p,&b,8 );
//	data_matlab[_cnt++]=13;
//	data_matlab[_cnt++]=10;
//	_cnt+=8;
//	p = &data_matlab[_cnt];
//	memcpy(p,&c,8 );	
//data_matlab[_cnt++]=13;
//	data_matlab[_cnt++]=10;
//	
//	Send_Data(data_matlab, _cnt);
//	\r\n
//}

void Send_Status(double a, double b, double c)
{
	printf("250\r\n");
	printf("%lf\r\n",a);
	printf("%lf\r\n",b);
	printf("%lf\r\n",c);
}
