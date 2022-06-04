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
/* 说明： */
/* 1.陀螺仪y轴指向正方向，绕y轴的角是Roll角，绕x轴的角是Pitch角 */

float Pitch_Kalman,Roll_Kalman;                     //卡尔曼滤波最终输出的角度

//extern double  acc_real[3];	            //加速度原始数据
//extern signed short  gyrox,gyroy,gyroz;             //陀螺仪原始数据
//float pitch, roll, yaw, gyro[3];                //加速度换算值
//float gyrox_real,gyroy_real,gyroz_real;             //角速度换算值

float Roll_z,Pitch_z,oo;                               //加速度计计算的角度值
float Roll_hat_pri=0,Pitch_hat_pri=0;               // 角度先验估计
float Roll_hat_pos=0,Pitch_hat_pos=0;               // 角度后验估计
float P_Roll_hat_pri=0,P_Pitch_hat_pri=0;           // 方差先验估计
float P_Roll_hat_pos=0,P_Pitch_hat_pos=0;           // 方差后验估计
float KalmanGain_Pitch,KalmanGain_Roll;             // 卡尔曼增益


float DeltaPitch=0;                                 // 角度变化 用于PID计算

//float integ_angle_x=0;integ_angle_y=0;integ_angle_z=0;  //角度积分值


//void GetAndWashData(void)
//{
//    // 获取原始数据
//    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);
//    

//    /*-----------------------------------------------数据预备----------------------------------------------*/
//    // 对原始数据进行单位转换,输出真实的加速度和角速度（对陀螺仪的初始化为+-2000dps，加速度计+-2g）
//    aacx_real = aacx/16384.0;
//    aacy_real = aacy/16384.0;
//    aacz_real = aacz/16384.0;
//    gyrox_real = gyrox/16.4;
//    gyroy_real = gyroy/16.4;
//    gyroz_real = gyroz/16.4;

//    // 加速度计计算(观测值)
//    Pitch_z = (atan(aacy_real/(sqrt(aacx_real*aacx_real+aacz_real*aacz_real)))) * 180 / 3.1415;   // 计算y轴与水平面夹角
//    Roll_z = (atan(aacx_real/(sqrt(aacy_real*aacy_real+aacz_real*aacz_real)))) * 180 / 3.1415;    // 计算x轴与水平面夹角
//}

void KalmanCalculation(float*gyro ,float* acc_real)
{        
    /*-----------------------------------------卡尔曼滤波--------------------------------------------------*/
    // 参数说明：A=1，B=dt，控制量=角速度，观测值直接测得角度H=1，
		Pitch_z = (atan(acc_real[1]/(sqrt(acc_real[0]*acc_real[0]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		Roll_z = 	(atan(acc_real[0]/(sqrt(acc_real[1]*acc_real[1]+acc_real[2]*acc_real[2])))) * 180 / 3.1415;
		R_PITCH=0.4*(ABs(DeltaPitch)*10+0.5);
		Q_ROLL=0.4*(ABs(DeltaRoll)*2+1);
    // 预测：本次角度先验估计 = 转移矩阵 * 上次角度后验估计 + 控制量
    Pitch_hat_pri = 1 * Pitch_hat_pos + gyro[0]*dt;
    Roll_hat_pri = 1 * Roll_hat_pos +  gyro[1]*dt;
		oo+=gyro[0]*dt;
    // 预测：本次方差先验估计 = 转移矩阵 * 上次方差后验估计 * 转移矩阵转置 + Q
    P_Pitch_hat_pri = P_Pitch_hat_pos + Q_PITCH;
    P_Roll_hat_pri = P_Roll_hat_pos + Q_ROLL;

    // 更新：计算卡尔曼增益
    KalmanGain_Pitch = (P_Pitch_hat_pri)/(P_Pitch_hat_pri+R_PITCH);
    KalmanGain_Roll = (P_Roll_hat_pri)/(P_Roll_hat_pri+R_ROLL);

    // 更新：计算后验估计
    Pitch_hat_pos = Pitch_hat_pri + KalmanGain_Pitch*(Pitch_z-Pitch_hat_pri);
    Roll_hat_pos = Roll_hat_pri + KalmanGain_Roll*(Roll_z-Roll_hat_pri);

    // 更新：计算新的方差后验估计
    P_Pitch_hat_pos = (1-KalmanGain_Pitch)*P_Pitch_hat_pri;
    P_Roll_hat_pos = (1-KalmanGain_Roll)*P_Roll_hat_pri;


    DeltaPitch = (-Pitch_hat_pos)-Pitch_Kalman;                         // 为了方便PID而进行的微分运算
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