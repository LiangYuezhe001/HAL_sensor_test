#include "spi.h"
#include "gpio.h"
#include "pwm3901.h"
#include <string.h>
#include "usart.h"
#include "sys.h"
#include "tim.h"
#include "ANO_DT.h"
#include "main.h"
#include "mpu6050.h"
#include "ak8963.h"
#include "bmp280.h"
#include "mpu9250.h"
#define NCS_PIN PAout(3)

u8 getandsend(void)
{   motionBurst_t currentMotion;
    float dx,odx=0,ddx,sumx=0,osumx=0;
    float dy,ody=0,ddy,sumy=0,osumy=0;
    int i=0,j=0;
    readMotion(&currentMotion);
    dx=currentMotion.deltaX;
    dy=currentMotion.deltaY;
    if(dx>=100)dx=00;
    if(dx<=-100)dx=00;
    if(dy>=100)dy=00;
    if(dy<=-100)dy=00;
    if(j==0)
    {
        if(i<500)
        {
            sumx+=dx;
            sumy+=dy;
            i++;
        }
        else
        {
            j=1;
            ddx=sumx/500;
            ddy=sumy/500;

        }
    }
    else {

        dx-=ddx;
        dy-=ddy;
        dx=0.1*odx+0.9*dx;
        odx=dx;
        dy=0.1*ody+0.9*dy;
        ody=dy;

        sumx+=dx;
        sumx=osumx*0.1+sumx*0.9;
        osumx=sumx;

        sumy+=dy;
        sumy=osumy*0.1+sumy*0.9;
        osumy=sumy;

        ANO_DT_Send_Senser(sumx,sumy,0,0,0,0,0,0,0,0);
    }
    return 1;
}

u8 GetOpFlow(int16_t* dx,int16_t* dy,u8* squal)
{
	motionBurst_t currentMotion;
	readMotion(&currentMotion);
	*squal=currentMotion.squal;
    *dx=currentMotion.deltaX;
    *dy=currentMotion.deltaY;
	 if(*dx>=100||*dx<=-100)*dx=00;
    if(*dy>=100||*dy<=-100)*dy=00;
    
	return 0;
}


void registerWrite(uint8_t reg, uint8_t value)
{

    reg |= 0x80u;
    NCS_PIN = 0;
    delay_us(5);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    delay_us(5);
    HAL_SPI_Transmit(&hspi1, &value, 1, 1000);
    delay_us(5);
    NCS_PIN = 1;
    delay_us(5);
}

void readMotion(motionBurst_t *motion)
{   u8 add = 0x16;
    u8 buff[11];
    u16 i;
    NCS_PIN = 0; //使能器件
    delay_us(5);
    HAL_SPI_Transmit(&hspi1, &add, 1, 1000);
    delay_us(5);
    HAL_SPI_TransmitReceive(&hspi1, buff, buff, 12, 100000);
    // for(i=0; i<12; i++)
    // {
    //     buff[i]=SPI1_ReadWriteByte(0XFF);   //循环读数
    // }
    NCS_PIN = 1; //取消片选

    memcpy(motion, buff, sizeof(motionBurst_t));
    uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
    realShutter |= (motion->shutter & 0x0ff) << 8;
    motion->shutter = realShutter;
}

static uint8_t registerRead(uint8_t reg)
{
    uint8_t data = 0;
    reg &= ~0x80u;
    NCS_PIN = 0;
    delay_us(5);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    delay_us(5);
    HAL_SPI_TransmitReceive(&hspi1, &data, &data, 1, 100000);
    // data=SPI1_ReadWriteByte(reg);
    delay_us(5);
    NCS_PIN = 1;
    delay_us(5);
    return data;
}

static void InitRegisters(void)
{
//    u8 data, data2;
//    registerWrite(0x7F, 0x00);
//    registerWrite(0x55, 0x01);
//    registerWrite(0x50, 0x07);
//    while (1)
//    {
//        registerWrite(0x43, 0x10);
//        data = registerRead(0x47);
//        if (data != 0x08)
//            break;
//    }
//    data = registerRead(0x47);
//    data &= 0x80;
//    if (data == 0x80)
//        registerWrite(0x48, 0x04);
//    else
//        registerWrite(0x48, 0x02);

//    registerWrite(0x7F, 0x00);
//    registerWrite(0x51, 0x7B);
//    registerWrite(0x50, 0x00);
//    registerWrite(0x55, 0x00);
//    registerWrite(0x7F, 0x0E);

//    data = registerRead(0x47);
//    if (data == 0x80)
//    {
//        data = registerRead(0x70);
//        if (data > 28)
//            data += 11;
//        else
//            data += 14;
//        data2 = registerRead(0x71);
//        data2 = (data2 * 45) / 100;
//        registerWrite(0x7F, 0x00);
//        registerWrite(0x61, 0xAD);
//        registerWrite(0x51, 0x70);
//        registerWrite(0x7F, 0x0E);
//        registerWrite(0x70, data);
//        registerWrite(0x71, data2);
//    }

    registerWrite(0x7F, 0x00);
    registerWrite(0x61, 0xAD);
    registerWrite(0x7F, 0x03);
    registerWrite(0x40, 0x00);
    registerWrite(0x7F, 0x05);
    registerWrite(0x41, 0xB3);
    registerWrite(0x43, 0xF1);
    registerWrite(0x45, 0x14);
    registerWrite(0x5B, 0x32);
    registerWrite(0x5F, 0x34);
    registerWrite(0x7B, 0x08);
    registerWrite(0x7F, 0x06);
    registerWrite(0x44, 0x1B);
    registerWrite(0x40, 0xBF);
    registerWrite(0x4E, 0x3F);
    registerWrite(0x7F, 0x08);
    registerWrite(0x65, 0x20);
    registerWrite(0x6A, 0x18);
    registerWrite(0x7F, 0x09);
    registerWrite(0x4F, 0xAF);
    registerWrite(0x5F, 0x40);
    registerWrite(0x48, 0x80);
    registerWrite(0x49, 0x80);
    registerWrite(0x57, 0x77);
    registerWrite(0x60, 0x78);
    registerWrite(0x61, 0x78);
    registerWrite(0x62, 0x08);
    registerWrite(0x63, 0x50);
    registerWrite(0x7F, 0x0A);
    registerWrite(0x45, 0x60);
    registerWrite(0x7F, 0x00);
    registerWrite(0x4D, 0x11);
    registerWrite(0x55, 0x80);
    registerWrite(0x74, 0x1F);
    registerWrite(0x75, 0x1F);
    registerWrite(0x4A, 0x78);
    registerWrite(0x4B, 0x78);
    registerWrite(0x44, 0x08);
    registerWrite(0x45, 0x50);
    registerWrite(0x64, 0xFF);
    registerWrite(0x65, 0x1F);
    registerWrite(0x7F, 0x14);
    registerWrite(0x65, 0x67);
    registerWrite(0x66, 0x08);
    registerWrite(0x63, 0x70);
    registerWrite(0x7F, 0x15);
    registerWrite(0x48, 0x48);
    registerWrite(0x7F, 0x07);
    registerWrite(0x41, 0x0D);
    registerWrite(0x43, 0x14);
    registerWrite(0x4B, 0x0E);
    registerWrite(0x45, 0x0F);
    registerWrite(0x44, 0x42);
    registerWrite(0x4C, 0x80);
    registerWrite(0x7F, 0x10);
    registerWrite(0x5B, 0x02);
    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x41);
    registerWrite(0x70, 0x00);
    HAL_Delay(15);
    registerWrite(0x32, 0x44);
    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x40);
    registerWrite(0x7F, 0x06);
    registerWrite(0x62, 0xF0);
    registerWrite(0x63, 0x00);
    registerWrite(0x7F, 0x0D);
    registerWrite(0x48, 0xC0);
    registerWrite(0x6F, 0xD5);
    registerWrite(0x7F, 0x00);
    registerWrite(0x5B, 0xA0);
    registerWrite(0x4E, 0xA8);
    registerWrite(0x5A, 0x50);
    registerWrite(0x40, 0x80);
}

u8 opticalFlowInit(void)
{
    u8 chipId = 0;
    u8 invChipId = 0;
    HAL_Delay(1);
		PAout(2) =1;//get those off 
    NCS_PIN = 1;
    HAL_Delay(2);
    NCS_PIN = 0;
    HAL_Delay(2);
    NCS_PIN = 1;
    HAL_Delay(2);
    chipId = registerRead(0x00);
    invChipId = registerRead(0x5f);
    if (chipId != 0x49 || invChipId != 0xB6)
        return 0;
		printf("3901ready\r\n");
    registerWrite(0x3a, 0x5a);
    HAL_Delay(50);
    registerRead(0x2);
    registerRead(0x3);
    registerRead(0x4);
    registerRead(0x5);
    registerRead(0x6);
    HAL_Delay(1);
    InitRegisters();
    HAL_Delay(1);
    return 1;
}

