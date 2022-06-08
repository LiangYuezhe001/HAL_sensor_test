#include "Kalman_Filter.h"
#include "math.h"
#include "ANO_DT.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "vl53lxx.h"

#define RESOLUTION			(0.2131946f)/*1m??? 1????????,??cm*/
#define DEG2RAD		0.017453293f	/* ??????? ??/180 */
#define RAD2DEG		57.29578f		/* ??????? 180/?? */

extern float Compound_G,DeltaYaw,R_Yaw,Pitch_Kalman,Roll_Kalman,Yaw_Kalman,yaw_bias;
float acc[3],Body_Bias_Pitch,Body_Bias_Roll,opitch,oroll;
float height;

void get_height(void)
{
	u16 laser_range= LaserGetHeight();
	if(laser_range!=600)
		{
	height=(float)(laser_range*cosf(Abs(Pitch_Kalman-Body_Bias_Pitch)*DEG2RAD)*cosf(Abs(Roll_Kalman-Body_Bias_Roll)*DEG2RAD));
	
	}
	
}

void yaw_calibration(int a)
	{float sum_yaw=0,gyro[3];
		
		for(int i =0;i<=a;i++)
		{
			Get_Gyro(gyro);
			sum_yaw+=gyro[2];
			HAL_Delay(5);
		}
		yaw_bias=sum_yaw/a;
}
	
void body_calibration(void)
{
	HAL_Delay(1000);
	Body_Bias_Pitch=Pitch_Kalman;
	Body_Bias_Roll=Roll_Kalman;
}

void Angle_Update(void)
{	static int i;
  float gyro[3],pressure,temperature,asl;
	
	Get_Gyro(gyro);
	Get_Acc(acc);

	//acc calibration
	acc[0]=(acc[0]+0.020)/0.99;
	acc[1]=(acc[1]+0.01)/1.01;
	acc[2]=(acc[2]+0.021)/1.00;
	
	//Compound g
	Compound_G=acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2];
	
  KalmanCalculation(gyro,acc);   
	
	if(Abs(DeltaYaw)<0.5&&Compound_G<1.2)
	{
		i++;
		if(i>=300)
		{i=0;
			yaw_calibration(50);
			R_Yaw=1;
		}
		
		else{
			i=0;
		R_Yaw=0.4;}
	}
	
	get_height();
ANO_DT_Send_Status(Roll_Kalman, Pitch_Kalman, height, 0, 0, 0);
}


void calibration(void)
{
	yaw_calibration(200);
	body_calibration();
}


