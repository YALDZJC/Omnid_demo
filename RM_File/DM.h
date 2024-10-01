#pragma once

#include "RM_StaticTime.h"//静态定时器
#include <stdint.h>

//根据上位机调节
#define DM_Pitch_CAN_ID 0x10
#define DM_Pitch_Master_ID 0x10

#define DM_YAW_CAN_ID 0x20
#define DM_YAW_Master_ID 0x20
//速度模式
#define DM_V_ID DM_YAW_CAN_ID + 0x200

//位置速度模式
#define DM_PV_ID DM_CAN_ID + 0x100
#define P_MIN (-3.1415926)
#define P_MAX (3.1415926)  

#define V_MIN (-30)
#define V_MAX (30)

#define KP_MIN (0.0)
#define KP_MAX (500.0)

#define KD_MIN (0.0)
#define KD_MAX (5.0)

#define T_MIN (-10)
#define T_MAX (10)

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min; 
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
/// Converts a float to an unsigned int, given range and number of bits///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x-offset)*((float)((1<<bits)-1))/span);
}

class DM
{
public:
  struct Data_t
  {
    uint8_t ID;
    uint8_t ERR;//故障
    float position;//位置
    float velocity;//速度
    float torque;//转矩
    uint8_t T_MOS;//MOS的平均温度
    uint8_t T_Rotor;//示电机内部线圈的平均温度
    int p_int;
    int v_int;
    int t_int;
  }Data;
  uint8_t send_data[8];
	RM_StaticTime dir_time;
	bool dir;
  DM(/* args */);
  //初始化
  void init();
  //解析
  void parse(CAN_RxHeaderTypeDef RxHeader, const uint8_t* RxData);
  //设置电机数据，力矩控制
  void ctrl_motor(float _pos, float _vel,float _KP, float _KD, float _torq);
  //位置速度
  void ctrl_motor2(float _pos, float _vel);
  //速度
  void ctrl_motor3(float _vel);
  //位置限幅
  float Pitch_angle_Limit(float IN,float max,float min);
  //开电机
  void on();
  //关电机
  void off();
  //清除电机错误
  void clear_err();
	//断链
	bool is_dir(int time);
};

DM::DM(/* args */)
{
}

void DM::init()
{
  this->on();  
}

void DM::parse(CAN_RxHeaderTypeDef RxHeader, const uint8_t* RxData)
{
	switch(RxHeader.StdId)
	{
    case DM_Pitch_CAN_ID:
        this->Data.ID = RxData[0] & 0xF0;
        this->Data.ERR = RxData[0] & 0xF;
        this->Data.p_int = (RxData[1]<<8)|RxData[2];
        this->Data.v_int = (RxData[3]<<4)|(RxData[4]>>4);
        this->Data.t_int = ((RxData[4]&0xF)<<8)|RxData[5];
        this->Data.T_MOS = RxData[6];
        this->Data.T_Rotor = RxData[7];

        this->Data.position = uint_to_float(this->Data.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
        this->Data.velocity = uint_to_float(this->Data.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
        this->Data.torque = uint_to_float(this->Data.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
      
        this->dir_time.UpLastTime();

     case DM_YAW_CAN_ID:
        this->Data.ID = RxData[0] & 0xF0;
        this->Data.ERR = RxData[0] & 0xF;
        this->Data.p_int = (RxData[1]<<8)|RxData[2];
        this->Data.v_int = (RxData[3]<<4)|(RxData[4]>>4);
        this->Data.t_int = ((RxData[4]&0xF)<<8)|RxData[5];
        this->Data.T_MOS = RxData[6];
        this->Data.T_Rotor = RxData[7];

        this->Data.position = uint_to_float(this->Data.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
        this->Data.velocity = uint_to_float(this->Data.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
        this->Data.torque = uint_to_float(this->Data.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
        
        this->dir_time.UpLastTime();
	}
}

void DM::ctrl_motor(float _pos, float _vel,float _KP, float _KD, float _torq)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
  kp_tmp  = float_to_uint(_KP,  KP_MIN,KP_MAX,12);
  kd_tmp  = float_to_uint(_KD,  KD_MIN,KD_MAX,12);
  tor_tmp = float_to_uint(_torq,T_MIN, T_MAX, 12);

  this->send_data[0] = (pos_tmp >> 8);
  this->send_data[1] = (pos_tmp);
  this->send_data[2] = (vel_tmp >> 4);
  this->send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
  this->send_data[4] = kp_tmp;
  this->send_data[5] = (kd_tmp >> 4);
  this->send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
  this->send_data[7] = tor_tmp;  
}

void DM::ctrl_motor2(float _pos, float _vel)
{
  uint8_t *pbuf,*vbuf;
  pbuf = (uint8_t*)&_pos;
  vbuf = (uint8_t*)&_vel;
  this->send_data[0] = *pbuf;
  this->send_data[1] = *(pbuf+1);
  this->send_data[2] = *(pbuf+2);
  this->send_data[3] = *(pbuf+3);
  this->send_data[4] = *vbuf;
  this->send_data[5] = *(vbuf+1);
  this->send_data[6] = *(vbuf+2);
  this->send_data[7] = *(vbuf+3);
}

void DM::ctrl_motor3(float _vel)
{
  uint8_t *vbuf;
  vbuf = (uint8_t*)&_vel;
  this->send_data[0] = *vbuf;
  this->send_data[1] = *(vbuf+1);
  this->send_data[2] = *(vbuf+2);
  this->send_data[3] = *(vbuf+3);
}

float DM::Pitch_angle_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	
	return OUT;
}

inline void DM::on()
{
  *(uint64_t*)(&this->send_data[0]) = 0xFCFFFFFFFFFFFFFF;
}

inline void DM::off()
{
  *(uint64_t*)(&this->send_data[0]) = 0xFDFFFFFFFFFFFFFF;
}

inline void DM::clear_err()
{
  *(uint64_t*)(&this->send_data[0]) = 0xFBFFFFFFFFFFFFFF;
}

inline bool DM::is_dir(int time)
{
	this->dir = this->dir_time.ISDir(time);
	return this->dir;
}