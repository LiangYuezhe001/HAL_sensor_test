#include "Kalman_Filter.h"
#include "math.h"
#include "ANO_DT.h"
#include "Ellipse_Fitting.h"
#include "mpu6050.h"
extern float pitch, roll;
float Q_ROLL=0.001;
float Q_PITCH=0.001;
float R_ROLL;
float R_PITCH=0.4;
float dt=0.005;
float DeltaPitch;
float DeltaRoll;
/* ˵���� */
/* 1.������y��ָ����������y��Ľ���Roll�ǣ���x��Ľ���Pitch�� */

float Pitch_Kalman,Roll_Kalman;                     //�������˲���������ĽǶ�

//extern double  acc_real[3];	            //���ٶ�ԭʼ����
//extern signed short  gyrox,gyroy,gyroz;             //������ԭʼ����
//float pitch, roll, yaw, gyro[3];                //���ٶȻ���ֵ
//float gyrox_real,gyroy_real,gyroz_real;             //���ٶȻ���ֵ

float Roll_z,Pitch_z,oo;                               //���ٶȼƼ���ĽǶ�ֵ
float Roll_hat_pri=0,Pitch_hat_pri=0;               // �Ƕ��������
float Roll_hat_pos=0,Pitch_hat_pos=0;               // �ǶȺ������
float P_Roll_hat_pri=0,P_Pitch_hat_pri=0;           // �����������
float P_Roll_hat_pos=0,P_Pitch_hat_pos=0;           // ����������
float KalmanGain_Pitch,KalmanGain_Roll;             // ����������


float DeltaPitch=0;                                 // �Ƕȱ仯 ����PID����

//float integ_angle_x=0;integ_angle_y=0;integ_angle_z=0;  //�ǶȻ���ֵ


//void GetAndWashData(void)
//{
//    // ��ȡԭʼ����
//    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);
//    

//    /*-----------------------------------------------����Ԥ��----------------------------------------------*/
//    // ��ԭʼ���ݽ��е�λת��,�����ʵ�ļ��ٶȺͽ��ٶȣ��������ǵĳ�ʼ��Ϊ+-2000dps�����ٶȼ�+-2g��
//    aacx_real = aacx/16384.0;
//    aacy_real = aacy/16384.0;
//    aacz_real = aacz/16384.0;
//    gyrox_real = gyrox/16.4;
//    gyroy_real = gyroy/16.4;
//    gyroz_real = gyroz/16.4;

//    // ���ٶȼƼ���(�۲�ֵ)
//    Pitch_z = (atan(aacy_real/(sqrt(aacx_real*aacx_real+aacz_real*aacz_real)))) * 180 / 3.1415;   // ����y����ˮƽ��н�
//    Roll_z = (atan(aacx_real/(sqrt(aacy_real*aacy_real+aacz_real*aacz_real)))) * 180 / 3.1415;    // ����x����ˮƽ��н�
//}

void KalmanCalculation(float*gyro ,float* acc_real)
{        
    /*-----------------------------------------�������˲�--------------------------------------------------*/
    // ����˵����A=1��B=dt��������=���ٶȣ��۲�ֱֵ�Ӳ�ýǶ�H=1��
		Pitch_z = (atan(acc_real[1]/(sqrt(acc_real[0]*acc_real[0]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		Roll_z = 	(atan(acc_real[0]/(sqrt(acc_real[1]*acc_real[1]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		R_PITCH=0.4*(ABs(DeltaPitch)*10+0.5);
		Q_ROLL=0.4*(ABs(DeltaRoll)*2+1);
    // Ԥ�⣺���νǶ�������� = ת�ƾ��� * �ϴνǶȺ������ + ������
    Pitch_hat_pri = 1 * Pitch_hat_pos + gyro[0]*dt;
    Roll_hat_pri = 1 * Roll_hat_pos +  gyro[1]*dt;
		oo+=gyro[0]*dt;
    // Ԥ�⣺���η���������� = ת�ƾ��� * �ϴη��������� * ת�ƾ���ת�� + Q
    P_Pitch_hat_pri = P_Pitch_hat_pos + Q_PITCH;
    P_Roll_hat_pri = P_Roll_hat_pos + Q_ROLL;

    // ���£����㿨��������
    KalmanGain_Pitch = (P_Pitch_hat_pri)/(P_Pitch_hat_pri+R_PITCH);
    KalmanGain_Roll = (P_Roll_hat_pri)/(P_Roll_hat_pri+R_ROLL);

    // ���£�����������
    Pitch_hat_pos = Pitch_hat_pri + KalmanGain_Pitch*(Pitch_z-Pitch_hat_pri);
    Roll_hat_pos = Roll_hat_pri + KalmanGain_Roll*(Roll_z-Roll_hat_pri);

    // ���£������µķ���������
    P_Pitch_hat_pos = (1-KalmanGain_Pitch)*P_Pitch_hat_pri;
    P_Roll_hat_pos = (1-KalmanGain_Roll)*P_Roll_hat_pri;


    DeltaPitch = (-Pitch_hat_pos)-Pitch_Kalman;                         // Ϊ�˷���PID�����е�΢������
    Pitch_Kalman = -Pitch_hat_pos;
    Roll_Kalman = Roll_hat_pos;
		
		ANO_DT_Send_Senser(Pitch_z,0,-Pitch_Kalman,0,0,0,0,0,0,0);

}



void Output(void)
{
   float acc[3],gyro[3];
	Get_Gyro(gyro);
	Get_Acc(acc);
   KalmanCalculation(gyro,acc);   
}