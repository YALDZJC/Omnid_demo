#pragma once
#include "RM_stdxxx.h"

struct RM_double_t
{
  double a[10];
  RM_double_t(double a0 = 0,double a1 = 0,double a2 = 0,double a3 = 0,double a4 = 0,double a5 = 0,double a6 = 0,double a7 = 0,double a8 = 0,double a9 = 0)
  {
    a[0] = a0;
    a[1] = a1;
    a[2] = a2;
    a[3] = a3;
    a[4] = a4;
    a[5] = a5;
    a[6] = a6;
    a[7] = a7;
    a[8] = a8;
    a[9] = a9;
  }
};

//一阶低通滤波
typedef struct 
{	
	double a,as[10];
	double x,xs[10];
	double y,ys[10],ysy;
	double last_y,last_ys[10];
  //一阶低通滤波器更新数据
  inline double FOLPFUpData(double x,double a)
  {
    this->a = a;
	  this->x = x;
	  this->y = this->a * this->x + (1 - this->a) * this->last_y;
	  this->last_y = this->y;
	  return this->y;
  }
  //Nx个一阶低通滤波器更新数据
  inline double NxFOLPFUpData(double x,RM_double_t a,int size)
  {
    this->as[0] = a.a[0];
    this->xs[0] = x;
    this->ys[0] = this->as[0] * this->xs[0] + (1 - this->as[0]) * this->last_ys[0];
    this->last_ys[0] = this->ys[0];
    for(int i = 1;i < size;i++)
    {
      this->as[i] = a.a[0];
      this->xs[i] = this->ys[i - 1];
      this->ys[i] = this->as[i] * this->xs[i] + (1 - this->as[i]) * this->last_ys[i];
      this->last_ys[i] = this->ys[i];
    }
    this->ysy = this->ys[size - 1];
    return this->ysy;
  }
}FirstOrderLowPassFilter_t;

//平均滤波
typedef struct 
{	
	double x[100];
	short int counter;
	double y;
  //平均滤波器更新数据
  inline double MFUpData(double x,double a)
  {
    this->counter++;
    if(this->counter > a)
    {
      memcpy(&this->x[0],&this->x[1],sizeof(double) * a);
      this->counter--;
    }
    this->x[this->counter - 1] = x;
    this->y = 0;
    for(short int i = 0;i < this->counter;i++)this->y += this->x[i];
    if(this->y != 0)this->y = this->y / this->counter;
    return  this->y / this->counter;
  }
}MedianFilter_t;

//矩阵转置
void MatrixTrans(const double A[][2], double B[][2])
{
	memcpy(B, A, sizeof(double) * 4);
	B[0][1] = A[1][0];
	B[1][0] = A[0][1];
}

//矩阵加减
void MatrixPlus(const double A[][2], const double B[][2], double C[][2], char SumOrSub)
{
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			if (SumOrSub == 0)C[i][j] = A[i][j] - B[i][j];
			if (SumOrSub == 1)C[i][j] = A[i][j] + B[i][j];
		}
	}
}

//矩阵相乘
void MatrixMult(const double A[][2], const double B[][2], double C[][2])
{
	C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
	C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
	C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
	C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}

//逆矩阵
void MatrixRevers(const double A[][2], double B[][2])
{
	double temp = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
	if (temp == 0)return;
	B[0][0] = A[1][1];
	B[1][1] = A[0][0];
	B[0][1] = -A[0][1];
	B[1][0] = -A[1][0];
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			B[i][j] /= temp;
}

//备份矩阵
double kem_temp[10][2][2] = { 0 };
//卡尔曼滤波器
typedef struct KalmanFilter
{
	double Q[2][2];//预测误差方差矩阵,（过程误差）
	double R[2][2];//测量误差矩阵，（测量误差）
	double A[2][2];//状态矩阵
	double H[2][2];//测量矩阵
	double Pk[2][2];//测量方差矩阵
	double Pp[2][2];//预测误差矩阵
	double Kk[2][2];//卡尔曼增益矩阵
	double Cin[2][2];//输入矩阵
	double Cout[2][2];//输出矩阵
	double y,y2;//输出
  unsigned int tager_h;//目标步长
  unsigned int now_h;//当前步长，用于收敛pp,pk,kk
	char is_close_h;//是否打开步长
  //卡尔曼滤波
  inline double UpData(double input,double input2 = 0)
  {
		if(is_close_h == 0)
		{			
    //准备参数和测量
    this->Cin[1][0] = this->Cin[0][0];//上一次
		}
		else
		{
		//准备参数和测量
    this->Cin[1][0] = input2;//上一次
		}
		
    this->Cin[0][0] = input;//这一次
    MatrixMult(this->A, this->Cout, kem_temp[0]);//kem_temp[0] = A * x
    memcpy(this->Cout, kem_temp[0], sizeof(double) * 4);
		
    if(this->now_h < this->tager_h || is_close_h == 1)
    {
      //计算Pp
      MatrixMult(this->A, this->Pk, kem_temp[0]);//kem_temp[0] = A * Pk
      MatrixTrans(this->A, kem_temp[1]);//kem_temp[1] = A'
      MatrixMult(kem_temp[0], kem_temp[1], kem_temp[2]);//kem_temp[3] = kem_temp[0] * kem_temp[1]
      MatrixPlus(kem_temp[2], this->Q, kem_temp[3], 1);//kem_temp[3] = kem_temp[2] + Q
      memcpy(this->Pp, kem_temp[3], sizeof(double) * 4);
    }

    if(this->now_h < this->tager_h || is_close_h == 1)
    {
      //计算Kk
      MatrixMult(this->H, this->Pp, kem_temp[0]);//kem_temp[0] = H * Pp
      MatrixTrans(this->H, kem_temp[1]);//kem_temp[1] = H'
      MatrixMult(kem_temp[0], kem_temp[1], kem_temp[2]);//kem_temp[2] = kem_temp[0] * kem_temp[1]
      MatrixPlus(kem_temp[2], this->R, kem_temp[3], 1);//kem_temp[3] = kem_temp[2] + R
      MatrixRevers(kem_temp[3], kem_temp[4]);//kem_temp[4] = kem_temp[3]的逆矩阵
      MatrixMult(this->Pp, kem_temp[1], kem_temp[5]);//kem_temp[5] = Pp * kem_temp[1]
      MatrixMult(kem_temp[5], kem_temp[4], kem_temp[6]);//kem_temp[6] = kem_temp[5] * kem_temp[4]
      memcpy(this->Kk, kem_temp[6], sizeof(double) * 4);
    }

    //计算融合值
    MatrixPlus(this->Cin, this->Cout, kem_temp[0], 0);//kem_temp[0] = y - x
    MatrixMult(this->Kk, kem_temp[0], kem_temp[1]);//kem_temp[1] = Kk * kem_temp[0]
    MatrixPlus(this->Cout, kem_temp[1], kem_temp[2], 1);//kem_temp[2] = y + kem_temp[1]
    memcpy(this->Cout, kem_temp[2], sizeof(double) * 4);

    if(this->now_h < this->tager_h || is_close_h == 1)
    {
      //计算P_k
      MatrixMult(this->Kk, this->Pp, kem_temp[0]);//kem_temp[0] = Kk * Pp
      MatrixPlus(this->Pp, kem_temp[0], kem_temp[1], 0);//kem_temp[1] = Pp - kem_temp[0]
      memcpy(this->Pk, kem_temp[1], sizeof(double) * 4);
    }
		
		if(this->now_h < this->tager_h )
    {
			this->now_h++;
		}
		
		this->y = this->Cout[0][0];//输出
		this->y2 = this->Cout[1][0];//输出

    return this->Cout[0][0];
  }
  //设置卡尔曼中的矩阵
  inline void SetKalman(double Kalmanx[][2], double x1, double x2, double x3, double x4)
  {
    Kalmanx[0][0] = x1;
    Kalmanx[0][1] = x2;
    Kalmanx[1][0] = x3;
    Kalmanx[1][1] = x4;
  }
  //卡尔曼滤波基本初始化
  inline void KalmanFilterInit(unsigned int tager_h = 200)
  {
    SetKalman(this->A,1,1,0,1);
    SetKalman(this->H,1,0,0,1);
    SetKalman(this->R,1,0,0,1);//预测误差方差矩阵,测量噪声
    SetKalman(this->Q,1,0,0,1);//测量误差矩阵,过程噪声
    SetKalman(this->Pk,10,0,0,10);
    this->tager_h = tager_h;
  }
	//设置测量误差矩阵
	inline void SetMatrix_Q(double x)
	{
		this->SetKalman(this->Q,x,0,0,x);
	}
	//设置测误差方差矩阵
	inline void SetMatrix_R(double x)
	{
		this->SetKalman(this->R,x,0,0,x);
	}
	KalmanFilter(double q,double r,unsigned int tager_h = 200)
	{
		this->KalmanFilterInit(tager_h);
    this->SetMatrix_Q(q);
    this->SetMatrix_R(r);
	}
  KalmanFilter()
	{
		this->KalmanFilterInit();
	}
}KalmanFilter_t;

