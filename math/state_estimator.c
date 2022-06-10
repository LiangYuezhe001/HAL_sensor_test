#include "Kalman_Filter.h"
#include "maths.h"
#include "ANO_DT.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "vl53lxx.h"
#include "send2matlab.h"
#include "pwm3901.h"
#define RESOLUTION			(0.2131946f)/*1m??? 1????????,??cm*/
#define DEG2RAD		0.017453293f	/* ??????? ??/180 */
#define RAD2DEG		57.29578f		/* ??????? 180/?? */

extern float DeltaYaw,R_Yaw,Pitch_Kalman,Roll_Kalman,Yaw_Kalman,yaw_bias,dt;
float acc[3],Body_Bias_Pitch,Body_Bias_Roll,opitch,oroll;
float height;
float PosSum_x=0,PosSum_y=0;
float pixSum_x,pixSum_y;
float pixValidLast_x,pixValidLast_y;
float cosPitch,cosRoll,cosYaw;
float Compound_G,Basic_G;

float acc_bias_x,acc_bias_y,acc_bias_z;
float acc_scal_x,acc_scal_y,acc_scal_z;


fp_angles_t body_angle;
t_fp_vector_def acc_vector;
t_fp_vector_def speed_vector;
t_fp_vector_def earth_bias_vector;
t_fp_vector_def pos_vector;
u8 squal;


//get height
void get_height(void)
{
	u16 laser_range= LaserGetHeight();
	if(laser_range!=600)
		{
	height=(float)(laser_range*Abs(cosPitch-Body_Bias_Pitch)*Abs(cosRoll-Body_Bias_Roll));
	
	}
	
}

//
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
	Body_Bias_Pitch=Pitch_Kalman*DEG2RAD;
	Body_Bias_Roll=Roll_Kalman*DEG2RAD;
}

void Angle_Update(void)
{	static int i,j,k=1;
	
  float gyro[3],pressure,temperature,asl;

	
	Get_Gyro(gyro);
	Get_Acc(acc);

	//acc calibration
	acc_vector.Y=acc[0]=(acc[0]+acc_bias_y)/acc_scal_y;
	acc_vector.X=acc[1]=(acc[1]+acc_bias_x)/acc_scal_x;
	acc_vector.Z=acc[2]=(acc[2]+acc_bias_z)/acc_scal_z;
	
	//Compound g
	Compound_G=acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2];
	
	//kalman
  KalmanCalculation(gyro,acc); 

	body_angle.angles.pitch=Pitch_Kalman*DEG2RAD;
	body_angle.angles.yaw=Yaw_Kalman*DEG2RAD;
	body_angle.angles.roll=Roll_Kalman*DEG2RAD;
	
		//COS
	cosPitch=cos_approx(body_angle.angles.pitch);
	cosRoll=cos_approx(body_angle.angles.roll);
	cosYaw=cos_approx(body_angle.angles.yaw);
	
	
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
	GetOpFlow(&dx,&dy,&squal);
	pixSum_x+=dx;
	pixSum_y+=dy;
	height/=100;
	if(height<4.0f)	/*4m???,????*/
	{
		cnt= 0;
		
		float coeff = RESOLUTION * height;
		float tanRoll = tanf((Roll_Kalman-Body_Bias_Roll) * DEG2RAD);
		float tanPitch = tanf((Pitch_Kalman-Body_Bias_Pitch) * DEG2RAD);
		
		pixComp_x = 490.f * tanPitch-6*DeltaYaw;	
		pixComp_y = 490.f * tanRoll+6*DeltaYaw;
		pixValid_x = (pixSum_x + pixComp_x);	/*??????*/
		pixValid_y = (pixSum_y - pixComp_y);
		
		//pixValid_y =pixSum_x;
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
		rotateV(&acc_vector, &body_angle);
		speed_vector.X+=(acc_vector.X);
		speed_vector.Y+=(acc_vector.Y);
		speed_vector.Z+=(acc_vector.Z-earth_bias_vector.Z);
		pos_vector.Z+=(speed_vector.Z)*dt;
	ANO_DT_Send_Status(speed_vector.X, pos_vector.Z, speed_vector.Z, 0, 0, 0);
	}
	
	
}


void calibration(void)
{
	yaw_calibration(200);
	body_calibration();
	getOpFlowData();
	earth_bias_vector.Z = acc_vector.Z;
	acc_bias_x=0.01;acc_bias_y=0.020;acc_bias_z=0.021;
	acc_scal_x=1.01;acc_scal_y=0.99;acc_scal_z=1;
}

void Accfrombody2earth()
{
	rotateV(&acc_vector, &body_angle);
	
}

