
#include "stdio.h"
#include "string.h"
#include "math.h"

#define MATRIX_SIZE 6
#define u8 unsigned char
double m_matrix[MATRIX_SIZE][MATRIX_SIZE + 1];
double m_result[MATRIX_SIZE];
int N = 0;

float ABs(float a)
{
	return a < 0 ? -a : a;
}

double Abs(double a)
{
	return a < 0 ? -a : a;
}

void ResetMatrix(void)
{
	for (u8 row = 0; row < MATRIX_SIZE; row++)
	{
		for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
			m_matrix[row][column] = 0.0f;
	}
}

void CalcData_Input(double x, double y, double z)
{
	double 	X[4],Y[4],Z[4];
	int i,j;
	N++;
	X[1]=x;X[2]=x*x;X[3]=x*x*x;
	Y[1]=y;Y[2]=y*y;Y[3]=y*y*y;
	Z[1]=z;Z[2]=z*z;Z[3]=z*z*z;
	
	m_matrix[0][0]+=Y[2]*Y[2];	m_matrix[0][1]+=Y[2]*Z[2];	m_matrix[0][2]+=X[1]*Y[2];	m_matrix[0][3]+=Y[1]*Y[2];	m_matrix[0][4]+=Y[2]*Z[1];	m_matrix[0][5]+=Y[2];	m_matrix[0][6]-=X[2]*Y[2];
															m_matrix[1][1]+=Z[2]*Z[2];	m_matrix[1][2]+=X[1]*Z[2];	m_matrix[1][3]+=Y[1]*Z[2];	m_matrix[1][4]+=Z[2]*Z[1];	m_matrix[1][5]+=Z[2];	m_matrix[1][6]-=X[2]*Z[2];
																													m_matrix[2][2]+=X[2];				m_matrix[2][3]+=X[1]*Y[1];	m_matrix[2][4]+=X[1]*Z[1];	m_matrix[2][5]+=X[1];	m_matrix[2][6]-=X[2]*X[1];
																																											m_matrix[3][3]+=Y[2];				m_matrix[3][4]+=Y[1]*Z[1];	m_matrix[3][5]+=Y[1];	m_matrix[3][6]-=X[2]*Y[1];
																																																									m_matrix[4][4]+=Z[2];				m_matrix[4][5]+=Z[1];	m_matrix[4][6]-=X[2]*Z[1];
																																																																							m_matrix[5][5]+=1;		m_matrix[5][6]-=X[2];
	
	for(i=0;i<5;i++)
	{
		for(j=i+1;j<6;j++)
		{
			m_matrix[j][i]=m_matrix[i][j];
		}
	}
//double V[MATRIX_SIZE + 1];
//	N++;
//	V[0] = y*y;
//	V[1] = z*z;
//	V[2] = x;
//	V[3] = y;
//	V[4] = z;
//	V[5] = 1.0;
//	V[6] = -x*x;
//	//??????,?????
//	for (u8 row = 0; row < MATRIX_SIZE; row++)
//	{
//		for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
//		{
//			m_matrix[row][column] += V[row] * V[column];
//		}
//	}
}

void CalcData_Input_average()
{
	for (u8 row = 0; row < MATRIX_SIZE; row++)
	for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
		m_matrix[row][column] /= N;
	
}


void Row2_swop_Row1(int row1, int row2)
{
	double tmp = 0;
	for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
	{
		tmp = m_matrix[row1][column];
		m_matrix[row1][column] = m_matrix[row2][column];
		m_matrix[row2][column] = tmp;
	}
}


void k_muiltply_Row(double k, int row)
{
	for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
		m_matrix[row][column] *= k;
}


void Row2_add_kRow1(double k, int row1, int row2)
{
	for (u8 column = 0; column < MATRIX_SIZE + 1; column++)
		m_matrix[row2][column] += k*m_matrix[row1][column];
}



void MoveBiggestElement_to_Top(int k)
{
	int row = 0, column = 0;

	for (row = k + 1; row < MATRIX_SIZE; row++)
	{
		if (Abs(m_matrix[k][k]) < Abs(m_matrix[row][k]))
		{
			Row2_swop_Row1(k, row);
		}
	}
}


u8 Matrix_GaussElimination(void)
{
	double k = 0;
	for (u8 cnt = 0; cnt < MATRIX_SIZE; cnt++)//???k????,?????k???????k???????0
	{
		MoveBiggestElement_to_Top(cnt);
		if (m_matrix[cnt][cnt] == 0)
			return(1);
		for (u8 row = cnt + 1; row < MATRIX_SIZE; row++)
		{
			k = -m_matrix[row][cnt] / m_matrix[cnt][cnt];
			Row2_add_kRow1(k, cnt, row);
		}
		
	}
	return 0;
}

void Matrix_RowSimplify(void)
{
	double k = 0;
	for (u8 row = 0; row < MATRIX_SIZE; row++)
	{
		k = 1 / m_matrix[row][row];
		k_muiltply_Row(k, row);
	}
	
}


void Matrix_Solve(double* solve, double* acc_bias, double* acc_scal)
{
	for (short row = MATRIX_SIZE - 1; row >= 0; row--)
	{
		solve[row] = m_matrix[row][MATRIX_SIZE];
		for (u8 column = MATRIX_SIZE - 1; column >= row + 1; column--)
			solve[row] -= m_matrix[row][column] * solve[column];
	}

		acc_bias[0] = -solve[2] / 2;
		acc_bias[1] = -solve[3] / (2 * solve[0]);
		acc_bias[2] = -solve[4] / (2 * solve[1]);
		acc_scal[0] = sqrt(acc_bias[0]*acc_bias[0] + solve[0]*acc_bias[1]*acc_bias[1] + solve[1]*acc_bias[2]*acc_bias[2] - solve[5]);
		acc_scal[1] = acc_scal[0] / sqrt(solve[0]);
		acc_scal[2] = acc_scal[0] / sqrt(solve[1]);
}

void GetItDone(double* acc_bias, double* acc_scal)
{
			double solve[6];
//		Get_Acc(acc);
//		CalcData_Input((double)acc[0], (double)acc[1], (double)acc[2]);
//		i++;
//		if(i>1000)
//		{GetItDone(acc_bias, acc_scal);
//			break;}
	CalcData_Input_average();
	Matrix_GaussElimination();
	Matrix_RowSimplify();
  Matrix_Solve(solve,acc_bias,acc_scal);
}


