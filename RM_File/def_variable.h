#pragma once

#include "RM_StaticTime.h"//静态定时器
#include "RM_Clicker.h"//遥控器
#include "ch_gyro_232.h"//陀螺仪
#include "RM_PID.h"
#include "ladrc.h"
#include "RM_Motor.h"//RM电机
#include "RM_Can.h"
#include "RM_stm32fxxx_hal.h"
#include "stm32f4xx_hal_can.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "RM_CHxxxGy.h"
#include "RM_Servos.h"
#include "DM.h"

#define pi (3.14159026f)

/*
														操作控制策划														

x代表无效
y代表有效
数字代表值

左手                                     右手
ch3:前后			ch2:左右			s1开关132			ch1:前后			ch0:左右			s2开关132
y             x            1             x             y            1             底盘跟随云台（1）
x             x            1             y             y            3             侧身（2）
x							x						 1						 x						 x					  2							小陀螺（3）

y             x            3             y             y            1             遥控器,底盘跟随云台（1）
y             x            3             y             y            3             遥控器,云台正方向（2）
x             x            3             x             x            2             遥控器,小陀螺（3）

x							x						 2						 x						 x						1							待定（7）
x							x						 2						 x						 x						3							待定（8）
x							x						 2						 x						 x						2							急停（9）
*/

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define GY_CAN 1//设置陀螺仪的can
#define GY_232 2//设置陀螺仪的232
#define GY_GET_SIG GY_CAN//设置can信号的陀螺仪

#define PITCH_ENCODER 1//pitch选择用编码器控制
#define PITCH_GY 2//pitch选择用陀螺仪控制
#define PITCH__GET_SIG PITCH_ENCODER//设置can信号的陀螺仪

#define MODE1 (RM_Clicker::RC_Ctl.rc.s1 == 1)//遥控器模式
#define MODE3 (RM_Clicker::RC_Ctl.rc.s1 == 3)//遥控器模式
#define MODE2 (RM_Clicker::RC_Ctl.rc.s1 == 2)//遥控器模式

#define CONTROL_SIG 0
#if CONTROL_SIG == 0	
	#define Clicker_chassis_follow_gimbal (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 1)//（1）底盘跟随云台
	#define Clicker_chassis_follow_gimbal_90 (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）底盘跟随云台90
	#define Clicker_chassis_gyro (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）小陀螺
	#define On_Friction_Dial_Warehouse_doors (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 1)//打开摩擦轮+仓门+拨盘
	#define Keyboard_Mouse (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）
	#define Combinatoria_Movement (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）组合运动模式
	#define Emergency_Stop (MODE2 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（9）急停模式
#elif CONTROL_SIG == 1
	#define stop_mode (Gimbal_to_Chassis_Data.stop)//（9）
#endif

#define r2g_180(x) ((float)x / 22.75 + 180.0f)

float _yaw_init_encoder_angle_ = (r2g_180(2.81839752)); //编码器绝对值初始化（第一次初始化的参数）
float yaw_init_encoder_angle = (_yaw_init_encoder_angle_); //编码器绝对值初始化
MedianFilter_t yaw_init_encoder_angle_MF;

#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
		#define piath_init_encoder_angle 7280 //编码器绝对值初始化
		#define piath_init_gy_angle -0.8 * 22.7527777//pitch初始角度转编码器的值
		#define PITCH_On_limit    (528)//(5800.0 - piath_init_encoder_angle) //上极限 测量编码器-初始化角度piath_init_encoder_angle
		#define PITCH_Lower_limit (-528)//(4800.0 - piath_init_encoder_angle) //下极限 测量编码器-初始化角度piath_init_encoder_angle
#elif PITCH__GET_SIG == PITCH_GY
		#define piath_init_encoder_angle 0- //陀螺仪值初始化
		#define PITCH_On_limit    (-30.0 - piath_init_encoder_angle) //上极限 测量编码器-初始化角度piath_init_encoder_angle
		#define PITCH_Lower_limit (22.0 - piath_init_encoder_angle) //下极限 测量编码器-初始化角度piath_init_encoder_angle
#endif

#define MOUSE_X_K -0.0005f//鼠标x移动系数
#define MOUSE_Y_K -0.00025f//鼠标y移动系数

/***************************变量声明*********************************/

//串口打印
#define Send_Usart_Data_Huart huart1
//云台到底盘
#define Send_Gimbal_to_Chassis_Huart huart6
#define Send_Gimbal_to_Chassis_Huart_LEN 12

//通信挂掉标记
bool dir = false;

/***************************底盘*********************************/

RM_StaticTime Total_tasks_staticTime;//控制时间x	

//陀螺仪获取端
#if GY_GET_SIG == GY_CAN
	RM_CHxxxGy chGy_chassis;//底盘陀螺仪
#elif GY_GET_SIG == GY_232
	ch_gyro_232_t chGy_chassis = { 0 };//底盘陀螺仪
#endif

/***************************云台到底盘数据*********************************/
#define YAW_MOTOR_ID 0x206
#define PITCH_MOTOR_ID 0x205

//摩擦轮
#define Friction_MOTOR_L_ID 0x203
#define Friction_MOTOR_R_ID 0x202
#define Friction_MAX_Speed 6500

//拨盘
#define Dial_MOTOR_R_ID 0x201
#define Dial_MAX_Speed 3000

//yaw轴大喵电机
//DM dm_pitch;
//bool dm_is_open;//大喵启动标志位

//用编码器的值做限位
#define MAX_PITCH_GY 15
#define MIN_PITCH_GY -15

//当前车体运行状态选择
enum now_chassis_mode_enum
{
	chassis_follow_gimbal = 0,
	chassis_follow_gimbal_90 = 1,
	chassis_gyro = 2
};
uint8_t now_chassis_mode = 0;

//组合测试小陀螺
int16_t test_gy_v = 0;
bool is_on_test = 0;
RM_Key Test_key;

//是否运行
bool is_run = 0;
//速度反向
bool is_v_reverse = 0;

//是否开启摩擦轮
bool is_on_MCL = false;
//是否开启仓门
bool is_on_CM = false;
//是否开启拨盘
bool is_on_BP = false;

//模式切换
RM_Key Mode1_key;
RM_Key Mode2_key;

//前馈数据
//RM_FeedForward yaw_ff(0.5,20,2000,5);
//RM_FeedForward pitch_ff(0.3,100000,30000,30000);
//pitch
float pitch_target_angle_gy = 0;//pitch期望角度,陀螺仪

float pitch_target_angle_Encoder = 0;//pitch期望角度,陀螺仪
Kpid_t kpid_pitch_angle(0, 0, 0);
Kpid_t kpid_pitch_speed(0, 0, 0);
RM_PID pid_pitch_angle;
RM_PID pid_pitch_speed;
TD_quadratic td_pitch_Encoder_speed;

//yaw
float yaw_target_angle_gy = 0;//yaw期望角度,陀螺仪
TD_quadratic td_yaw_target_angle_gy(200);//微分器滤波的角度
TD_quadratic td_yaw_target_angle_gasy;//微分器滤波的角速度
LADRC_quadratic ladrc_yaw_gy;//yaw
TD_quadratic td_yaw_gy(15);//目标跟踪
Kpid_t kpid_yaw_gy(0.3,500,-0.02,0,0,0.1,2000);
RM_PID pid_yaw_gy;

//弹舱相关
int32_t shoot_time;
int32_t shoot_time_ms;
//弹仓PID
int64_t Dial_angle;
Kpid_t kpid_angle_dial(0.35, 0, 0.5);
Kpid_t kpid_speed_dial(2, 0, 9);
RM_PID pid_speed_dial;
RM_PID pid_angle_dial;

float demo_tar;
RM_PID pid_DEMO;
Kpid_t pid_DEMO_Init(0, 0, 0);
UDE ude_pitch;

float tar_pitch = 75;
float tar_pitch_Demo;
int32_t data_ms;

float max_yaw_t = 5;//五牛米
float yaw_speed_kp = 0.05;
float last_yaw_cin,now_yaw_cin,e_yaw_cin;//记录输入
float load_t = 0.3;//负载扭矩

float YAW_Init_Angle;//初始化角度
float yaw_target_add_angle;//增加的角度
float yaw_target_angle;//期望角度
float yaw_target_angle_temp;//期望角度temp
float yaw_encoder_angle_e = 0;//当前编码器与初始位置的误差
float temp_yaw_init_encoder_angle = 0;
float yaw_now_angle = 0;
float Friction_Speed;//摩擦轮期望速度
float Dial_Speed;//拨盘期望速度
uint8_t send_motor_ms;//用于3508与6020错开发送

//键鼠前进后退控制率
TD_quadratic td_move(10);
TD_quadratic td_move1(10);//左右

#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
		LADRC_quadratic ladrc_pitch(TD_quadratic(50),6,0,50,30000);//ptich
		float pitch_offsets_gy = 0;//补偿角度
		bool is_open_pitch_offsets_gy;//是否开启补偿
		TD_quadratic td_gy_y(400);//y轴td滤波		
#elif PITCH__GET_SIG == PITCH_GY
		LADRC_quadratic ladrc_pitch;//ptich
		RM_PID pitch_pid;
		Kpid_t pitch_kpid(-1200,-4000,-500);
#endif

float ladrc_friction_b = 500;
float ladrc_friction_z1 = 100;
float ladrc_friction_z2 = 5000;
float ladrc_friction_z3 = 1000000;
float ladrc_friction_max_u = 16384;
float ladrc_friction_kp = 800;
float ladrc_friction_kd = 1000;

LADRC_quadratic ladrc_friction_l(TD_quadratic(10),500,0,120,16384,0.001);//左摩擦轮

LADRC_quadratic ladrc_friction_r(TD_quadratic(10),500,0,120,16384,0.001);//右摩擦轮

LADRC_quadratic ladrc_dial(TD_quadratic(10),800,0,80,16384,0.001);//拨盘

RM_PID yaw_pid;
TD_quadratic td_yaw_cin(10);//对输入滤波
#if GY_GET_SIG == GY_CAN
		Kpid_t yaw_kpid(2000,3000,2000);
		TD_quadratic td_yaw_i(800);//对积分求导
		float yaw_i_kd = -0.05;//积分的导数的系数kd
		LADRC_quadratic ladrc_yaw_angle(TD_quadratic(20),0.3,0,0,30000,0.001);
		float disturbance_test = 0,t,hz = 0,rx = 7;//扰动测试
		float ff_add_angle_kp = 20000;//给予前馈补偿
		float ff_Clicker_add_angle_kp = 20000;//给予前馈补偿
		float ff_Mouse_add_angle_kp = 1000;//给予前馈补偿
		RM_FeedForward ff_add_angle;//前馈
		RM_FeedForward ff_Clicker_add_angle;//前馈
		RM_FeedForward ff_Mouse_add_angle;//前馈
#elif GY_GET_SIG == GY_232
		Kpid_t yaw_kpid;
		TD_quadratic td_yaw_i(800);//对积分求导
		float yaw_i_kd = -0.07;//积分的导数的系数kd
#endif

float yaw_send_data = 0;//融合所有数据之后要发送的数据

RM_Key Ctrl_key;

RM_Key Q_Key;//q键触发
uint8_t Q_Key_size;//q键触发次数,单数小陀螺，双数不小陀螺

RM_Key F_Key;//f键触发
uint8_t F_Key_size;//f键触发次数,单数侧身，双数不侧身

RM_Key B_Key;//B键触发，开摩擦轮

RM_Key G_Key;//G键触发，关摩擦轮

RM_Key Shift_Key;//Shift键触发
uint8_t Shift_Key_size;//Shift键触发次数,跟随并且加速

RM_Key E_Key;//E键触发+底盘转速
RM_Key R_Key;//R键触发-底盘转速
int8_t ER_Key_size;//ER计算器

RM_Key C_Key;//C键触发开关仓门舵机

RM_Key V_Key;//V键触发开关仓门舵机

RM_Servos servos;//仓门舵机
float servos_angle;//舵机角度
float servos_angle_on = 0;//初始舵机角度，开
float servos_angle_off = 80;//舵机固定角度，关

/***************************云台到底盘数据*********************************/
struct Chassis_to_Gimbal_Data_t
{
	uint8_t head;//帧头
	
	float chGy_chassis_data_x;//底盘角度
	
	bool stop;//停止

	uint8_t now_mode;//当前模式
	int8_t is_v_reverse;//速度反向
	
	uint16_t magnet_size_last;
	uint16_t magnet_size;
	
	uint8_t pData[50];
	bool dir;
	RM_StaticTime dir_time;
};
//数据
Chassis_to_Gimbal_Data_t Chassis_to_Gimbal_Data; 
//获取云台到底盘数据ore
void Get_Chassis_to_Gimbal_ore(UART_HandleTypeDef* huart);//解决ore问题