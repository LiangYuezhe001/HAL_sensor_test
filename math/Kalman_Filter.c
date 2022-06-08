#include "Kalman_Filter.h"
#include "math.h"
#include "ANO_DT.h"
#include "Ellipse_Fitting.h"
#include "mpu6050.h"
#include "bmp280.h"
extern float pitch, roll;
float Q_ROLL=0.001;
float Q_PITCH=0.001;
float Q_Yaw=0.001;
float R_ROLL=0.6;
float R_PITCH=0.6;
float R_Yaw=0.4;
float dt=0.005;
float yaw_dt=0.01;
float oyaw,fyaw,dyaw,yaw_bias;
float DeltaPitch;
float DeltaRoll;
float DeltaYaw;
float Compound_G;
/* ˵���� */
/* 1.������y��ָ����������y��Ľ���Roll�ǣ���x��Ľ���Pitch�� */

float Pitch_Kalman,Roll_Kalman,Yaw_Kalman;                     //�������˲���������ĽǶ�


float Roll_z,Pitch_z,Yaw_z;                               //���ٶȼƼ���ĽǶ�ֵ
float Roll_hat_pri=0,Pitch_hat_pri=0,Yaw_hat_pri=0;               // �Ƕ��������
float Roll_hat_pos=0,Pitch_hat_pos=0,Yaw_hat_pos=0;               // �ǶȺ������
float P_Roll_hat_pri=0,P_Pitch_hat_pri=0,P_Yaw_hat_pri=0;           // �����������
float P_Roll_hat_pos=0,P_Pitch_hat_pos=0,P_Yaw_hat_pos=0;           // ����������
float KalmanGain_Pitch,KalmanGain_Roll,KalmanGain_Yaw;             // ����������



void KalmanCalculation(float*gyro ,float* acc_real)
{        
    /*-----------------------------------------�������˲�--------------------------------------------------*/
	
    // ����˵����A=1��B=dt��������=���ٶȣ��۲�ֱֵ�Ӳ�ýǶ�H=1��
		Pitch_z = (atan(acc_real[1]/(sqrt(acc_real[0]*acc_real[0]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		Roll_z = 	(atan(acc_real[0]/(sqrt(acc_real[1]*acc_real[1]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		Yaw_z += (gyro[2]-yaw_bias)*yaw_dt;
	
		// DYNAMIC R/Q
		R_PITCH=0.6*(ABs(DeltaPitch)*10+0.5);
		R_ROLL=0.6*(ABs(DeltaRoll)*10+0.5);
		if(Abs(Compound_G)>=1.2)
		{
			R_PITCH=0.6*(ABs(DeltaPitch)*100+0.5);
			R_ROLL=0.6*(ABs(DeltaRoll)*100+0.5);
			
		}
	
		//filter of yaw
		fyaw = 0.9*oyaw+0.1*Yaw_z;
		dyaw=fyaw-oyaw;
		oyaw=fyaw;
	
    // Ԥ�⣺���νǶ�������� = ת�ƾ��� * �ϴνǶȺ������ + ������
    Pitch_hat_pri = 1 * Pitch_hat_pos + gyro[0]*dt;
    Roll_hat_pri = 1 * Roll_hat_pos +  gyro[1]*dt;
		Yaw_hat_pri = 1 * Yaw_hat_pos + dyaw*yaw_dt ;
		
    // Ԥ�⣺���η���������� = ת�ƾ��� * �ϴη��������� * ת�ƾ���ת�� + Q
    P_Pitch_hat_pri = P_Pitch_hat_pos + Q_PITCH;
    P_Roll_hat_pri = P_Roll_hat_pos + Q_ROLL;
		P_Yaw_hat_pri = P_Yaw_hat_pos + Q_Yaw;
		
    // ���£����㿨��������
    KalmanGain_Pitch = (P_Pitch_hat_pri)/(P_Pitch_hat_pri+R_PITCH);
    KalmanGain_Roll = (P_Roll_hat_pri)/(P_Roll_hat_pri+R_ROLL);
		KalmanGain_Yaw = (P_Yaw_hat_pri)/(P_Yaw_hat_pri+R_Yaw);
		
    // ���£�����������
    Pitch_hat_pos = Pitch_hat_pri + KalmanGain_Pitch*(Pitch_z-Pitch_hat_pri);
    Roll_hat_pos = Roll_hat_pri + KalmanGain_Roll*(Roll_z-Roll_hat_pri);
		Yaw_hat_pos = Yaw_hat_pri + KalmanGain_Yaw*(Yaw_z-Yaw_hat_pri);

    // ���£������µķ���������
    P_Pitch_hat_pos = (1-KalmanGain_Pitch)*P_Pitch_hat_pri;
    P_Roll_hat_pos = (1-KalmanGain_Roll)*P_Roll_hat_pri;
		P_Yaw_hat_pos = (1-KalmanGain_Yaw)*P_Yaw_hat_pri;


    DeltaPitch = (-Pitch_hat_pos)-Pitch_Kalman;                         // Ϊ�˷���PID�����е�΢������
   DeltaYaw= (-Yaw_hat_pos)-Yaw_Kalman;
	 Pitch_Kalman = -Pitch_hat_pos;
    Roll_Kalman = Roll_hat_pos;
		Yaw_Kalman =Yaw_hat_pos;
		

}



void Angle_Update(void)
{	static int i;
  float acc[3],gyro[3],pressure,temperature,asl;
	
	Get_Gyro(gyro);
	Get_Acc(acc);
	
	//acc calibration
	acc[0]=(acc[0]+0.020)/0.99;
	acc[1]=(acc[1]+0.01)/1.01;
	acc[2]=(acc[2]+0.021)/1.00;
	
	//g
	Compound_G=acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2];
	
  KalmanCalculation(gyro,acc);   
	
	if(Abs(DeltaYaw)<0.5&&Compound_G<1.2)
	{
		i++;
		if(i>=200)
		{i=0;
			yawcalibration(50);
			R_Yaw=1;
		}
		
		else{
			i=0;
		R_Yaw=0.4;}
	}
	
	ANO_DT_Send_Status(Pitch_Kalman, Roll_Kalman, Yaw_Kalman, 0, 0, 0);
}



void yawcalibration(int a)
	{float sum_yaw=0,gyro[3];
		
		for(int i =0;i<=a;i++)
		{
			Get_Gyro(gyro);
			sum_yaw+=gyro[2];
			HAL_Delay(5);
			
		}
		yaw_bias=sum_yaw/a;
}