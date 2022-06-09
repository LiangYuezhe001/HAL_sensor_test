#include "Kalman_Filter.h"
#include "math.h"
#include "ANO_DT.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "vl53lxx.h"
#include "send2matlab.h"
#include "pwm3901.h"
#define RESOLUTION			(0.2131946f)/*1m??? 1????????,??cm*/
#define DEG2RAD		0.017453293f	/* ??????? ??/180 */
#define RAD2DEG		57.29578f		/* ??????? 180/?? */

extern float Compound_G,DeltaYaw,R_Yaw,Pitch_Kalman,Roll_Kalman,Yaw_Kalman,yaw_bias;
float acc[3],Body_Bias_Pitch,Body_Bias_Roll,opitch,oroll;
float height;
float PosSum_x=0,PosSum_y=0;
float pixSum_x,pixSum_y;
float pixValidLast_x,pixValidLast_y;
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
		PosSum_x=0,PosSum_y=0;
		pixSum_x=0,pixSum_y=0;
		pixValidLast_x=0,pixValidLast_y=0;
}
	
void body_calibration(void)
{
	HAL_Delay(1000);
	Body_Bias_Pitch=Pitch_Kalman;
	Body_Bias_Roll=Roll_Kalman;
}

void Angle_Update(void)
{	static int i,j,k=1;
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
	
//	if(Abs(DeltaYaw)<0.5&&Compound_G<1.2)
//	{
//		i++;
//		if(i>=300)
//		{i=0;
//			yaw_calibration(50);
//			R_Yaw=1;
//		}
//		
//		else{
//			i=0;
//		R_Yaw=0.4;}
//	}
	
	get_height();
//ANO_DT_Send_Status(Roll_Kalman, Pitch_Kalman, height, 0, 0, 0);	
}

void getOpFlowData(void)
{
	static u8 cnt = 0;
	int16_t dy,dx;
	float pixComp_x,pixComp_y;
	float pixValid_x,pixValid_y;
	
	
	float deltaPos_x,deltaPos_y;
	
	Angle_Update();
	get_height();
	GetOpFlow(&dx,&dy);
	pixSum_x+=dx;
	pixSum_y+=dy;
	height/=100;
	if(height<4.0f)	/*4m???,????*/
	{
		cnt= 0;

		float coeff = RESOLUTION * height;
		float tanRoll = tanf((Roll_Kalman-Body_Bias_Roll) * DEG2RAD);
		float tanPitch = tanf((Pitch_Kalman-Body_Bias_Pitch) * DEG2RAD);
		
		pixComp_x = 800.f * tanPitch;	
		pixComp_y = 480.f * tanRoll;
		pixValid_x = (pixSum_x + pixComp_x);	/*??????*/
		//pixValid_y = (pixSum_y - pixComp_y);		
		pixValid_y =pixSum_x;
		if(height < 0.02f)	/*????????5cm*/
		{
			coeff = 0.0f;
		}
		
		deltaPos_x = coeff * (pixValid_x - pixValidLast_x);	/*2????????,??cm*/
		deltaPos_y = coeff * (pixValid_y - pixValidLast_y);
		pixValidLast_x=pixValid_x;
		pixValidLast_y=pixValid_y;
//		opFlow.deltaVel[X] = opFlow.deltaPos[X] / dt;	/*?? cm/s*/
//		opFlow.deltaVel[Y] = opFlow.deltaPos[Y] / dt;

		PosSum_x += deltaPos_x;	/*???? cm*/
		PosSum_y += deltaPos_y;	/*???? cm*/
	ANO_DT_Send_Status(PosSum_x, Pitch_Kalman, PosSum_y, 0, 0, 0);
	}
	
	
}

void calibration(void)
{
	yaw_calibration(200);
	body_calibration();
}


