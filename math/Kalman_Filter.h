#ifndef KALMAN
#define KALMAN

#include <stdio.h>
#include "Ellipse_Fitting.h"
void TestKalman(void);
void KalmanCalculation(float*gyro ,float* acc_real);
void Output(void);
//void yawcalibration(int a);
#endif