#include "sys.h"
void ResetMatrix(void);
void CalcData_Input(double x, double y, double z);
void CalcData_Input_average();
u8 Matrix_GaussElimination(void);
void Matrix_Solve(double* solve, double* acc_bias, double* acc_scal);
void Matrix_RowSimplify(void);
double Abs(double a);
float ABs(float a);
void GetItDone(double* acc_bias, double* acc_scal);