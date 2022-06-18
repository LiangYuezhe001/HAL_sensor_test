//############################################################
// Created on: 2019年5月18日
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//STM32电机控制开发板
//匠心科技
//网址: https://shop298362997.taobao.com/
//电机控制答疑邮箱：JXKJ_2007@163.com
//############################################################
#include "stm32f10x.h"


#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    
				   
#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_6)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(GPIOC,GPIO_Pin_6)


#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_0)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_0)


#define OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_1)//RES
#define OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_1)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_10)//DC
#define OLED_DC_Set() GPIO_SetBits(GPIOC,GPIO_Pin_10)


#define OLED_CS_Clr()  GPIO_ResetBits(GPIOC,GPIO_Pin_3)//CS
#define OLED_CS_Set()  GPIO_SetBits(GPIOC,GPIO_Pin_3)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 mod);
void OLED_ShowString(u8 x,u8 y,char *chr,u8 mod);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N);
