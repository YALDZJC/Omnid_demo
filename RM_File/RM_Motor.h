#pragma once
#include "RM_StaticTime.h"
#include "RM_stdxxx.h"
#include "RM_Pid.h"
#include "RM_Filter.h"

//发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)

//获取设置id
#define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
#define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
#define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

//获取设置stdid
#define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

//电机发送数据
typedef struct 
{
	uint8_t Data[8];
}Motor_send_data_t;

//电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Motor_Data_Type
{
	Motor_Data_Angle = 0x00,
	Motor_Data_Speed = 0x01,
	Motor_Data_Torque = 0x02,
	Motor_Data_Temperature = 0x03,
	Motor_Data_Stop = 0x04,
};

//提供一个默认数据变量
Motor_send_data_t msd_6020;
Motor_send_data_t msd_3508_2006;

//设置发送数据
void setMSD(Motor_send_data_t* msd,int16_t data,int id)
{
	msd->Data[(id - 1) * 2] = data >> 8;
  msd->Data[(id - 1) * 2 + 1] = data << 8 >> 8;	
}

//获取下标
#define GET_Motor_ID_IDX_BIND_(_motor_,id) (_motor_.id_idx_bind.idxs[id])
#define _Motor_ID_IDX_BIND_SIZE_ 5
//电机绑定
struct _Motor_ID_IDX_BIND_
{
	uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];
	_Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size)
	{
		for (uint8_t i = 0; i < _Motor_ID_IDX_BIND_SIZE_; i++)//标记
		{
			this->idxs[i] = 0xff;
		}
		for (uint8_t i = 0; i < size; i++)//绑定
		{
			this->idxs[ids[i]] = i;
		}
	}
};
class RM_Motor;
//获取对应的下标
int GET_Motor_ID_ADDRESS_BIND_(RM_Motor* _motor_,int address);

typedef struct 
{
  int16_t address;//地址
	int16_t Data[4];//数据
	int16_t LastData[4];//历史数据
	int32_t AddData;//累加数据
	int16_t InitData;//初始化数据
	bool  InitFlag;//初始化标记
	bool  DirFlag;//死亡标记
	RM_StaticTime dirTime;//运行时间
	RM_PID posPid;//位置pid
	RM_PID speedPid;//速度pid
	RM_PID stopPid;//停止pid
	RM_PID torquePid;//转矩pid
}Motor_t;	//电机

class RM_Motor
{
public:
	int16_t init_address;//首地址
  Motor_t* motorData;
  uint8_t MotorSize;
	_Motor_ID_IDX_BIND_ id_idx_bind;
	//初始化
  RM_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs);
	//数据解析
  void Parse(RM_FDorCAN_RxHeaderTypeDef  RxHeader,uint8_t RxHeaderData[]);
	//获取速度
	int GetMotorDataSpeed(int16_t address);
	//获取位置
	int GetMotorDataPos(int16_t address);
	//获取转矩
	int GetMotorDataTorque(int16_t address);
	//获取类型
	Motor_t* GetMotor(int16_t address);
	//获取速度pid
	RM_PID* GetMotorSpeedPid(int address);
	//获取位置pid
	RM_PID* GetMotorPosPid(int address);
	//获取停止pid
	RM_PID* GetMotorStopPid(int address);
	//获取停止pid
	RM_PID* GetMotorTorquePid(int address);
	//断链
  uint8_t ISDir();
	//获取电机pid
	RM_PID* GetMotorPid(int address,uint8_t type);
	//更新速度pid(默认位置式)
	double UpSpeedPid(Kpid_t kpid, double expectations, int address, double maxspeed, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double) = &RM_PID::GetPidPos);
	//更新位置pid(默认位置式)
	double UpPosPid(Kpid_t kpid, double expectations, int address, double maxspeed, double maxpos, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double) = &RM_PID::GetPidPos);
	//更新转矩(默认位置式，附带一个卡尔曼滤波)
	double UpTorquePid(Kpid_t kpid, double expectations, int address, double maxspeed, KalmanFilter_t* KalmanFilter, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double) = &RM_PID::GetPidPos);
};

RM_Motor::RM_Motor(int16_t address, uint8_t MotorSize, Motor_t* MotorAddress,uint8_t* idxs)
	:id_idx_bind(_Motor_ID_IDX_BIND_(idxs,MotorSize))
{
	this->motorData = MotorAddress;
	this->init_address = address;
	for (uint8_t i = 0; i < MotorSize; i++)
  {		
    this->motorData[i].LastData[0] = -1;
		this->motorData[i].address = address + idxs[i];		
  }
  this->MotorSize = MotorSize;
}

inline void RM_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef  RxHeader, uint8_t RxHeaderData[])
{
  if(!(FDorCAN_ID(RxHeader) >= 0x200 && FDorCAN_ID(RxHeader) <= 0x208) || this->MotorSize == 0)return;	
	int idx = GET_Motor_ID_ADDRESS_BIND_((this),FDorCAN_ID(RxHeader));
	if(idx == -1)return;//如果超越数组大小，或者不存在id
	//数据解析
	this->motorData[idx].Data[Motor_Data_Angle] = (int16_t)(RxHeaderData[0]) << 8 | RxHeaderData[1];
	//转子速度
	this->motorData[idx].Data[Motor_Data_Speed] = (int16_t)(RxHeaderData[2]) << 8 | RxHeaderData[3];
	this->motorData[idx].Data[Motor_Data_Torque] = (int16_t)(RxHeaderData[4]) << 8 | RxHeaderData[5];
  //温度
  this->motorData[idx].Data[Motor_Data_Temperature] = (int16_t)(RxHeaderData[6]);
  //数据累加
	if(this->motorData[idx].LastData[Motor_Data_Angle] != this->motorData[idx].Data[Motor_Data_Angle] && 
     this->motorData[idx].LastData[Motor_Data_Angle] != -1)
	{
			int lastData = this->motorData[idx].LastData[Motor_Data_Angle];
			int Data = this->motorData[idx].Data[Motor_Data_Angle];
			if(Data - lastData < -4000)//正转
				this->motorData[idx].AddData += (8191 - lastData + Data);
			else if(Data - lastData > 4000)//反转
				this->motorData[idx].AddData += -(8191 - Data + lastData);
			else 
				this->motorData[idx].AddData += (Data - lastData);
	}
	//数据上一次更新
	//数据解析
	this->motorData[idx].LastData[Motor_Data_Angle] = this->motorData[idx].Data[Motor_Data_Angle];
	//转子速度
	this->motorData[idx].LastData[Motor_Data_Speed] = this->motorData[idx].Data[Motor_Data_Speed];
	this->motorData[idx].LastData[Motor_Data_Torque] = this->motorData[idx].Data[Motor_Data_Torque];
	//初始化数据
	if(this->motorData[idx].InitFlag == 0)
	{
		this->motorData[idx].InitData = this->motorData[idx].Data[Motor_Data_Angle];
		this->motorData[idx].InitFlag = 1;
	}
  //更新时间
  this->motorData[idx].dirTime.UpLastTime();
}

inline int RM_Motor::GetMotorDataSpeed(int16_t address)
{
	return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Speed];
}

inline int RM_Motor::GetMotorDataPos(int16_t address)
{
  return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Angle];
}

inline int RM_Motor::GetMotorDataTorque(int16_t address)
{
  return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].Data[Motor_Data_Torque];
}

inline Motor_t *RM_Motor::GetMotor(int16_t address)
{
  return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)];
}

inline RM_PID *RM_Motor::GetMotorSpeedPid(int address)
{
  return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].speedPid;
}

inline RM_PID *RM_Motor::GetMotorPosPid(int address)
{
  return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].posPid;
}

inline RM_PID *RM_Motor::GetMotorStopPid(int address)
{
  return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].stopPid;
}

inline RM_PID *RM_Motor::GetMotorTorquePid(int address)
{
	return &this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),address)].torquePid;
}

inline uint8_t RM_Motor::ISDir()
{
  for (int i = 0; i < this->MotorSize; i++)
  {
    if(this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),this->motorData[i].address)].dirTime.ISDir(10))
      return i + 1;
  }
  return 0;
//	return this->motorData[GET_Motor_ID_ADDRESS_BIND_((this),this->motorData->address)].dirTime.ISDir(10));
}

inline RM_PID *RM_Motor::GetMotorPid(int address, uint8_t type)
{
	if (type == Motor_Data_Angle)
	{
		return this->GetMotorPosPid(address);
	}
	else if (type == Motor_Data_Speed)
	{
		return this->GetMotorSpeedPid(address);
	}
	else if(type == Motor_Data_Torque)
	{
		return this->GetMotorTorquePid(address);
	}
	else if (type == Motor_Data_Stop)
	{
		return this->GetMotorStopPid(address);
	}
  return NULL;
}

inline double RM_Motor::UpSpeedPid(Kpid_t kpid, double expectations, int address, double maxspeed, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double))
{
  return (GetMotorPid(address,Motor_Data_Speed)->*RM_PIDx)(kpid,expectations,this->GetMotorDataSpeed(address),maxspeed);
}

inline double RM_Motor::UpPosPid(Kpid_t kpid, double expectations, int address, double maxspeed, double maxpos, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double))
{
	double tempcin = fmod(expectations,maxpos);
	double x1 = this->GetMotorDataPos(address);
	if(tempcin < 0)
		x1 -= maxpos;
		//过0处理
	if(tempcin - x1 < -maxpos/2)
		tempcin += maxpos;			
	if(tempcin - x1 > maxpos/2)
		tempcin -= maxpos;
	return (GetMotorPid(address,Motor_Data_Angle)->*RM_PIDx)(kpid,tempcin,x1,maxspeed);
}

inline double RM_Motor::UpTorquePid(Kpid_t kpid, double expectations, int address, double maxspeed, KalmanFilter_t* KalmanFilter, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double))
{
	KalmanFilter->UpData(this->GetMotorDataTorque(address));
	return (GetMotorPid(address,Motor_Data_Torque)->*RM_PIDx)(kpid,expectations,KalmanFilter->y,maxspeed);
}

//获取对应的下标
int GET_Motor_ID_ADDRESS_BIND_(RM_Motor* _motor_,int address)
{
	int idx = address - (_motor_->init_address);
	if(idx < 0)return -1;
	if(idx >= _Motor_ID_IDX_BIND_SIZE_)return -1;
	if(_motor_->id_idx_bind.idxs[idx] == 0xff)return -1;
	return _motor_->id_idx_bind.idxs[idx];
}

//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 2
#define _Motor6020_SIZE 2

Motor_t _Motor2006_[_Motor2006_SIZE] = { 0 };uint8_t _Motor2006_ID_[_Motor2006_SIZE] = { 1 };
Motor_t _Motor3508_[_Motor3508_SIZE] = { 0 };uint8_t _Motor3508_ID_[_Motor3508_SIZE] = { 3,2 };
Motor_t _Motor6020_[_Motor6020_SIZE] = { 0 };uint8_t _Motor6020_ID_[_Motor6020_SIZE] = { 1,2 };
RM_Motor Motor2006(0x200,_Motor2006_SIZE,_Motor2006_,_Motor2006_ID_);
RM_Motor Motor3508(0x200,_Motor3508_SIZE,_Motor3508_,_Motor3508_ID_);
RM_Motor Motor6020(0x204,_Motor6020_SIZE,_Motor6020_,_Motor6020_ID_);

//求圆最小旋转数据
double SolvingCircleMinData(double expectations, double feedback, double maxData, double feedbackData)
{
		expectations = fmod(expectations , maxData);
		double error = expectations - feedback;
		if(error < -maxData/2)feedbackData += maxData;			
		if(error > maxData/2)feedbackData -= maxData;
		return feedbackData;
}


double isminmode[4] = { 0 };
//设置位置环最小处理
double MinPosHelm(RM_PID *Rm_Pid, Kpid_t kpid, double expectations, double feedback, double maxspeed, double maxpos, int isminmodeidx)
{
		//x1当前位置
		//x2当前位置的对半位置
		//x3反馈位置
		double x1 = 0,x2 = 0,x3 = 0;
		double tempcin = fmod(expectations,maxpos);
		x1 = tempcin;
		x2 = tempcin + 8191 / 2;
		x3 = feedback;
		if(tempcin < 0)
			x3 -= maxpos;
		//过0处理
		if(x1 - x3 < -maxpos/2)
			x1 += maxpos;
		
		if(x1 - x3 > maxpos/2)
			x1-= maxpos;
		
		if(x2 - x3 < -maxpos/2)
			x2 += maxpos;
		
		if(x2 - x3 > maxpos/2)
			x2 -= maxpos;
		
		//两个最小角度
		int minangle1 = 0,minangle2 = 0;
		minangle1 =  fabs(x1 - x3);
		minangle2 =  fabs(x2 - x3);
		//角度比较，看选择哪个角度
		if(minangle1 <= minangle2)
		{
			tempcin = x1;
			isminmode[isminmodeidx] = 1;
		}
		else
		{
			tempcin = x2;
			isminmode[isminmodeidx] = 2;
		}
		return Rm_Pid->GetPidPos(kpid,tempcin,x3,maxspeed);
}

//设置速度环最小处理
double SpeedHelm(RM_PID *Rm_Pid, Kpid_t kpid, double speed, double feedback, double maxspeed, int isminmodeidx)
{
		double tempcin = speed;
		//判断轮子位置是否速度放转
		if(isminmode[isminmodeidx] == 2)
			tempcin = -tempcin;
		return Rm_Pid->GetPidPos(kpid,tempcin,feedback,maxspeed);
}

//更新任意速度pid
double UpASpeedPid(RM_PID *Rm_Pid, Kpid_t kpid, double expectations, double feedback, double maxspeed, double (RM_PID::*RM_PIDx)(Kpid_t, double, double, double))
{
		return (Rm_Pid->*RM_PIDx)(kpid,expectations,feedback,maxspeed);
}

//更新任意位置pid
double UpAPosPid(RM_PID *Rm_Pid, Kpid_t kpid,double expectations,double feedback,double maxspeed,double maxpos,double (RM_PID::*RM_PIDx)(Kpid_t,double,double,double))
{
	double tempcin = expectations;
	if(maxpos != 0)
	{
		tempcin = fmod(expectations,maxpos);
		double x1 = feedback;
		if(tempcin < 0)
			x1 -= maxpos;
		//过0处理
		if(tempcin - feedback < -maxpos/2)
			tempcin += maxpos;			
		if(tempcin - feedback > maxpos/2)
			tempcin -= maxpos;
	}
	return (Rm_Pid->*RM_PIDx)(kpid,tempcin,feedback,maxspeed);
}

//过零处理
float Zero_crossing_processing(float expectations, float feedback, float maxspeed )
{
	if(expectations - feedback > maxspeed * 0.5)expectations -= maxspeed;
	else if(expectations - feedback < -maxspeed * 0.5)expectations += maxspeed;
	return expectations;
}
