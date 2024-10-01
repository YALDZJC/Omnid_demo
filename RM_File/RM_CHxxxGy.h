#pragma once
#include "RM_stm32fxxx_hal.h"
#include "RM_StaticTime.h"
typedef struct 
{	
	float x;
	float y;
	float z;
	float z_360;//360度z
	float last_z_360;//上一次角度
	float Add_z;//累计角度
	bool InitFlag;//初始化标记
	float InitData;//初始化角度
  //加速度
	int16_t AcSx;
	int16_t AcSy;
	int16_t AcSz;
	//角速度
	float ASx;
	float ASy;
	float ASz;
}GyAngle_t;

class RM_CHxxxGy
{
public:
  GyAngle_t gy;//陀螺仪
  RM_StaticTime dirTime;//死亡时间
  bool dir;//死亡标志
  void Parse(CAN_RxHeaderTypeDef RxHeader,uint8_t RxHeaderData[]);//解析
  bool GetDir();//断连
};

inline void RM_CHxxxGy::Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
  if(!(RxHeader.StdId == 0x188) & !(RxHeader.StdId == 0x288) & !(RxHeader.StdId == 0x388))return;
  if(RxHeader.StdId == 0x188)
	{
		this->gy.AcSx = ((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0]));
		this->gy.AcSy = ((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2]));
		this->gy.AcSz = ((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4]));
	}
	if(RxHeader.StdId == 0x288)
	{
		this->gy.ASx = (float)((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0])) * 0.01;
		this->gy.ASy = (float)((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2])) * 0.01;
		this->gy.ASz = (float)((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4])) * 0.01;
	}
	if(RxHeader.StdId == 0x388)
	{
		this->gy.x = (float)((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0])) * 0.01;
		this->gy.y = (float)((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2])) * 0.01;
		this->gy.z = (float)((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4])) * 0.01;
		this->gy.z_360 = this->gy.z;
		if(this->gy.z_360 < 0)this->gy.z_360 += 360;
		//数据累加
		if(this->gy.last_z_360 != this->gy.z_360 && 
			 this->gy.last_z_360 != -1)
		{
				float lastData = this->gy.last_z_360;
				float Data = this->gy.z_360;
				if(Data - lastData < -(360*0.5))//正转
					this->gy.Add_z += (360 - lastData + Data);
				else if(Data - lastData > (360*0.5))//反转
					this->gy.Add_z += -(360 - Data + lastData);
				else 
					this->gy.Add_z += (Data - lastData);
		}
		//数据上一次更新
		//数据解析
		this->gy.last_z_360 = this->gy.z_360;
		//初始化数据
		if(this->gy.InitFlag == 0)
		{
			this->gy.InitData = this->gy.z_360;
			this->gy.InitFlag = 1;
		}
	}
  dirTime.UpLastTime();
}

inline bool RM_CHxxxGy::GetDir()
{
  this->dir = this->dirTime.ISDir(20);
  return this->dir;
}
