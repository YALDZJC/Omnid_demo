#pragma once

#include "RM_stdxxx.h"

#define MF9025_StdId(x) (0x140+x)

enum MF9025CommandAddress
{
  enum_readPid = 0x30,//读取pid
  enum_writePidtoRAM = 0x31,//写入pid到ram
  enum_writePidtoROM = 0x32,//写入pid到rom
  enum_readSpeed_ramp = 0x33,//读取加速度
  enum_writeSpeed_ramp = 0x34,//写入加速度
  enum_readEncoder = 0x90,//读取编码器
  enum_writeEncodertoROM = 0x91,//写入编码器值到rom,只允许写入零偏
  enum_readMotorAngle = 0x92,//读取电机多圈角度
  enum_readCircleAngle = 0x94,//读取电机单圈角度
  enum_readMotrState1 = 0x9A,//读取电机状态1和错误标志
  enum_clearMotrError = 0x9B,//清除电机错误标志
  enum_readMotrState2 = 0x9C,//读取电机状态2
  enum_readMotrState3 = 0x9D,//读取电机状态3
  enum_off = 0x80,//电机关闭
  enum_stop = 0x81,//电机停止
  enum_up = 0x88,//电机启动
  enum_Torque_Control = 0xA1,//转矩闭环控制
  enum_Speed_Control = 0xA2,//速度闭环控制
  enum_Multi_Loop_Angle_Control1 = 0xA3,//多圈位置闭环控制1
  enum_Multi_Loop_Angle_Control2 = 0xA4,//多圈位置闭环控制2
  enum_Single_Loop_Angle_Control1 = 0xA5,//单圈位置闭环控制1
  enum_Single_Loop_Angle_Control2 = 0xA6,//单圈位置闭环控制2
  enum_Increment_Angle_Control1 = 0xA7,//增量位置闭环控制1
  enum_Increment_Angle_Control2 = 0xA8,//增量位置闭环控制2
};

struct Array1x2TempU8//1*2的uint8_t类型的数组
{
    uint8_t data1, data2;
    Array1x2TempU8(const uint8_t& data1, const uint8_t& data2) :data1(data1), data2(data2) {}
};

class MF9025
{
private:
	/*数据类型*/
  struct Data_t
  {
    /*PID*/
    struct
    {
      uint8_t kp,ki;
      void set(const uint8_t& kp,const uint8_t& ki)
      {
        this->kp = kp;
        this->ki = ki;
      }
    }pid_angle,pid_speed,pid_current;
    /*加速度*/
    int32_t speed_ramp;
    /*编码器*/
    struct 
    {
      uint16_t encoder;//编码器位置
      uint16_t encoderRam;//编码器原始位置
      uint16_t encoderOffset;//编码器零偏
    }encoder_data;
    /*多圈角度*/
    float motor_angle;
    /*单圈角度*/
    float circle_angle;
		float circle_angle_parse;//自己计算得来
    /*电机状态表*/
    struct 
    {
      int8_t temperature;//电机温度
      /*状态1*/
      struct 
      {
        uint16_t voltage;//电压 * 0.1
        uint8_t errorState;//错误标准
      }motor_state1;
      /*状态2*/
      struct 
      {
        int16_t iq;//电机转矩电流，-2048~2048，对应-33~33A
        int16_t speed;//电机转速
        /*编码器的值与encoder_data共享同一个变量*/
      }motor_state2;
      /*状态3*/
      struct 
      {
        int16_t iA,iB,iC;//A,B,C相电流
      }motor_state3;
    }motor_state_data;
  }Data;
	//清除Tx
  void clearTxData();
public:
	uint8_t TxData[8],RxData[8];
  MF9025(/* args */);
  //解析
  void parse(const uint8_t* RxData);
  //读取pid
  void readPid();
  //写入pid到ram
  void writePidtoRAM(Array1x2TempU8 angle = Array1x2TempU8(0,0),Array1x2TempU8 speed = Array1x2TempU8(0,0),Array1x2TempU8 current = Array1x2TempU8(0,0));
  //写入pid到rom
  void writePidtoROM(Array1x2TempU8 angle = Array1x2TempU8(0,0),Array1x2TempU8 speed = Array1x2TempU8(0,0),Array1x2TempU8 current = Array1x2TempU8(0,0));
  //读取加速度
  void readSpeed_ramp();
  //写入加速度
  void writeSpeed_ramp(int32_t speed_ramp);
  //读取编码器命令
  void readEncoder();
  //写入编码器指令，只允许零偏写入
  void writeEncodertoROM(uint16_t encoderOffset);
  //读取电机多圈角度
  void readMotorAngle();
  //读取电机单圈角度
  void readCircleAngle();
  //读取电机状态1
  void readMotorState1();
  //清除电机错误标准
  void clearMotorError();
  //读取电机状态2
  void readMotorState2();
  //读取电机状态3
  void readMotorState3();
  //电机关闭
  void off();
  //电机停止
  void stop();
  //电机启动
  void up();
  //转矩闭环控制
  void Torque_Control(int16_t iqControl);
  //速度闭环控制
  void Speed_Control(float speedControl);
  //多圈位置闭环控制1
  void Multi_Loop_Angle_Control1(float angleControl);
  //多圈位置闭环控制2
  void Multi_Loop_Angle_Control2(float angleControl,uint16_t maxSpeed = 500);
  //单圈位置闭环控制1
  void Single_Loop_Angle_Control1(float angleControl,uint8_t spinDirection = 0x00);
  //单圈位置闭环控制2
  void Single_Loop_Angle_Control2(float angleControl,uint16_t maxSpeed = 500,uint8_t spinDirection = 0x00);
  //增量位置闭环控制1
  void Increment_Angle_Control1(float angleControl);
  //增量位置闭环控制2
  void Increment_Angle_Control2(float angleControl,uint16_t maxSpeed = 500);
	//获取Data
	MF9025::Data_t& getData();
};

MF9025::MF9025(/* args */)
{
  
}

inline void MF9025::parse(const uint8_t *RxData)
{
  memcpy(this->RxData,RxData,sizeof(this->RxData));
  switch (RxData[0])//id
  {
  case enum_readPid:
  case enum_writePidtoRAM:
  case enum_writePidtoROM:
    this->Data.pid_angle.set(RxData[2],RxData[3]);
    this->Data.pid_speed.set(RxData[4],RxData[5]);
    this->Data.pid_current.set(RxData[6],RxData[7]);
    break;
  case enum_readSpeed_ramp:
  case enum_writeSpeed_ramp:
    this->Data.speed_ramp = (RxData[7] << 24) | (RxData[6] << 16) | (RxData[5] << 8) | RxData[4];
		break;
  case enum_readEncoder:
  case enum_writeEncodertoROM:
    this->Data.encoder_data.encoder = (RxData[3] << 8) | RxData[2];
    this->Data.encoder_data.encoderRam = (RxData[5] << 8) | RxData[4];
    this->Data.encoder_data.encoderOffset = (RxData[7] << 8) | RxData[6];
		break;
  case enum_readMotorAngle:
    this->Data.motor_angle = (float)((RxData[7] << 48) | (RxData[6] << 40) | (RxData[5] << 32) | (RxData[4] << 24) | (RxData[3] << 16) | (RxData[2] << 8) | RxData[1]) * 0.01f;
		break;
  case enum_readCircleAngle:
    this->Data.circle_angle = (float)((RxData[7] << 24) | (RxData[6] << 16) | (RxData[5] << 8) | RxData[4]) * 0.01f;
		break;
  case enum_readMotrState1:
  case enum_clearMotrError:
    this->Data.motor_state_data.temperature = RxData[1];
    this->Data.motor_state_data.motor_state1.voltage = (RxData[4] << 8) | RxData[3];
    this->Data.motor_state_data.motor_state1.errorState = RxData[7];
    break;
  case enum_readMotrState2:
  case enum_Torque_Control:
  case enum_Speed_Control:
  case enum_Multi_Loop_Angle_Control1:
  case enum_Multi_Loop_Angle_Control2:
  case enum_Single_Loop_Angle_Control1:
  case enum_Single_Loop_Angle_Control2:
  case enum_Increment_Angle_Control1:
  case enum_Increment_Angle_Control2:
    this->Data.motor_state_data.temperature = RxData[1];
    this->Data.motor_state_data.motor_state2.iq = (RxData[3] << 8) | RxData[2];
    this->Data.motor_state_data.motor_state2.speed = (RxData[5] << 8) | RxData[4];
    this->Data.encoder_data.encoder = (RxData[7] << 8) | RxData[6];
    break;
  case enum_readMotrState3:
    this->Data.motor_state_data.temperature = RxData[1];
    this->Data.motor_state_data.motor_state3.iA = (RxData[3] << 8) | RxData[2];
    this->Data.motor_state_data.motor_state3.iB = (RxData[5] << 8) | RxData[4];
    this->Data.motor_state_data.motor_state3.iC = (RxData[7] << 8) | RxData[6];
    break;
  default:
    break;
  }
  this->Data.circle_angle_parse = (double)360.0 / 65535.0 * this->Data.encoder_data.encoder;
}

inline void MF9025::clearTxData()
{
	memset(this->TxData,0,sizeof(this->TxData));
}

inline void MF9025::readPid()
{
  this->clearTxData();
  this->TxData[0] = enum_readPid;
}

inline void MF9025::writePidtoRAM(Array1x2TempU8 angle,Array1x2TempU8 speed,Array1x2TempU8 current)
{
  this->TxData[0] = enum_writePidtoRAM;
  this->TxData[1] = 0x00;
  this->TxData[2] = angle.data1;//位置环kp
  this->TxData[3] = angle.data2;//位置环ki
  this->TxData[4] = speed.data1;//速度环kp
  this->TxData[5] = speed.data2;//速度环ki
  this->TxData[6] = current.data1;//电流环kp
  this->TxData[7] = current.data2;//电流环ki
}

inline void MF9025::writePidtoROM(Array1x2TempU8 angle,Array1x2TempU8 speed,Array1x2TempU8 current)
{
  this->TxData[0] = enum_writePidtoROM;
  this->TxData[1] = 0x00;
  this->TxData[2] = angle.data1;//位置环kp
  this->TxData[3] = angle.data2;//位置环ki
  this->TxData[4] = speed.data1;//速度环kp
  this->TxData[5] = speed.data2;//速度环ki
  this->TxData[6] = current.data1;//电流环kp
  this->TxData[7] = current.data2;//电流环ki
}

inline void MF9025::readSpeed_ramp()
{
  this->clearTxData();
  this->TxData[0] = enum_readSpeed_ramp;
}

inline void MF9025::writeSpeed_ramp(int32_t speed_ramp)
{
  this->clearTxData();
  this->TxData[0] = enum_writeSpeed_ramp;
  this->TxData[4] = *((uint8_t*)(&speed_ramp));//加速度
  this->TxData[5] = *((uint8_t*)(&speed_ramp) + 1);
  this->TxData[6] = *((uint8_t*)(&speed_ramp) + 2);
  this->TxData[7] = *((uint8_t*)(&speed_ramp) + 3);
}

inline void MF9025::readEncoder()
{
  this->clearTxData();
  this->TxData[0] = enum_readEncoder;
}

inline void MF9025::writeEncodertoROM(uint16_t encoderOffset)
{
  this->clearTxData();
  this->TxData[0] = enum_writeEncodertoROM;
  this->TxData[6] = *((uint8_t*)(&encoderOffset));//零偏
  this->TxData[7] = *((uint8_t*)(&encoderOffset) + 1);
}

inline void MF9025::readMotorAngle()
{
  this->clearTxData();
  this->TxData[0] = enum_readMotorAngle;
}

inline void MF9025::readCircleAngle()
{
  this->clearTxData();
  this->TxData[0] = enum_readCircleAngle;
}

inline void MF9025::readMotorState1()
{
  this->clearTxData();
  this->TxData[0] = enum_readMotrState1;
}

inline void MF9025::clearMotorError()
{
  this->clearTxData();
  this->TxData[0] = enum_clearMotrError;
}

inline void MF9025::readMotorState2()
{
  this->clearTxData();
  this->TxData[0] = enum_readMotrState2;
}

inline void MF9025::readMotorState3()
{
  this->clearTxData();
  this->TxData[0] = enum_readMotrState3;
}

inline void MF9025::off()
{
  this->clearTxData();
  this->TxData[0] = enum_off;
}

inline void MF9025::stop()
{
  this->clearTxData();
  this->TxData[0] = enum_stop;
}

inline void MF9025::up()
{
  this->clearTxData();
  this->TxData[0] = enum_up;
}

inline void MF9025::Torque_Control(int16_t iqControl)
{
  this->clearTxData();
  this->TxData[0] = enum_Torque_Control;
  this->TxData[4] = *((uint8_t*)(&iqControl));
  this->TxData[5] = *((uint8_t*)(&iqControl) + 1);
}

inline void MF9025::Speed_Control(float speedControl)
{
  int32_t speedControl_temp = speedControl * 100.0f;
  this->clearTxData();
  this->TxData[0] = enum_Speed_Control;
  this->TxData[4] = *((uint8_t*)(&speedControl_temp));
  this->TxData[5] = *((uint8_t*)(&speedControl_temp) + 1);
  this->TxData[6] = *((uint8_t*)(&speedControl_temp) + 2);
  this->TxData[7] = *((uint8_t*)(&speedControl_temp) + 3);
}

inline void MF9025::Multi_Loop_Angle_Control1(float angleControl)
{
  int32_t angleControl_temp = angleControl * 100.0f;
  this->clearTxData();
  this->TxData[0] = enum_Multi_Loop_Angle_Control1;
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
  this->TxData[6] = *((uint8_t*)(&angleControl_temp) + 2);
  this->TxData[7] = *((uint8_t*)(&angleControl_temp) + 3);
}

inline void MF9025::Multi_Loop_Angle_Control2(float angleControl, uint16_t maxSpeed)
{
  this->clearTxData();
	if(maxSpeed == 0){this->stop();return;}
	int32_t angleControl_temp = angleControl * 100.0f;
  this->TxData[0] = enum_Multi_Loop_Angle_Control2;
  this->TxData[2] = *((uint8_t*)(&maxSpeed));
  this->TxData[3] = *((uint8_t*)(&maxSpeed) + 1);
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
  this->TxData[6] = *((uint8_t*)(&angleControl_temp) + 2);
  this->TxData[7] = *((uint8_t*)(&angleControl_temp) + 3);
}

inline void MF9025::Single_Loop_Angle_Control1(float angleControl, uint8_t spinDirection)
{
  uint16_t angleControl_temp = angleControl * 100.0f;
  this->clearTxData();
  this->TxData[0] = enum_Single_Loop_Angle_Control1;
  this->TxData[1] = spinDirection;
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
}

inline void MF9025::Single_Loop_Angle_Control2(float angleControl, uint16_t maxSpeed, uint8_t spinDirection)
{
  this->clearTxData();
	if(maxSpeed == 0){this->stop();return;}
  uint16_t angleControl_temp = angleControl * 100.0f;
  this->TxData[0] = enum_Single_Loop_Angle_Control2;
  this->TxData[1] = spinDirection;
  this->TxData[2] = *((uint8_t*)(&maxSpeed));
  this->TxData[3] = *((uint8_t*)(&maxSpeed) + 1);
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
}

inline void MF9025::Increment_Angle_Control1(float angleControl)
{
  int32_t angleControl_temp = angleControl * 100.0f;
  this->clearTxData();
  this->TxData[0] = enum_Increment_Angle_Control1;
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
  this->TxData[6] = *((uint8_t*)(&angleControl_temp) + 2);
  this->TxData[7] = *((uint8_t*)(&angleControl_temp) + 3);
}

inline void MF9025::Increment_Angle_Control2(float angleControl, uint16_t maxSpeed)
{
	if(maxSpeed == 0){this->stop();return;}
  this->clearTxData();
  int32_t angleControl_temp = angleControl * 100.0f;
  this->TxData[0] = enum_Increment_Angle_Control2;
  this->TxData[2] = *((uint8_t*)(&maxSpeed));
  this->TxData[3] = *((uint8_t*)(&maxSpeed) + 1);
  this->TxData[4] = *((uint8_t*)(&angleControl_temp));
  this->TxData[5] = *((uint8_t*)(&angleControl_temp) + 1);
  this->TxData[6] = *((uint8_t*)(&angleControl_temp) + 2);
  this->TxData[7] = *((uint8_t*)(&angleControl_temp) + 3);
}

inline MF9025::Data_t& MF9025::getData()
{
	return this->Data;
}
