#pragma once

#include "def_variable.h"
/***************************函数声明*********************************/
//发送串口数据
void Send_Usart_Data(char* format,...);//发送

//主跑初始化
void Total_tasks_Init();

//主跑函数
void Total_tasks_Run();

//发送6020数据
void Send_6020_CAN();

//发送3508/2006数据
void Send_3508_2006_CAN();

//发送大喵电机
void Send_DM_CAN();

//急停模式控制所有pid
void control_pid_0_speed();

//清空数据数据
void Clear_ALL_Data();

//云台发送数据到底盘
void Send_Gimbal_to_Chassis();

//获取云台到底盘数据初始化
void Get_Chassis_to_Gimbal_Init();

//获取云台到底盘数据
void Get_Chassis_to_Gimbal(UART_HandleTypeDef* huart);

//获取云台到底盘数据ore
void Get_Chassis_to_Gimbal_ore(UART_HandleTypeDef* huart);//解决ore问题

//发送6020数据
void Send_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_6020,msd_6020.Data,CAN_TX_MAILBOX1);
}
//发送3508/2006数据
void Send_3508_2006_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_3508,msd_3508_2006.Data,CAN_TX_MAILBOX1);
}
//发送大喵电机
void Send_DM_CAN()
{
//	RM_FDorCAN_Send(&hcan1,0x20,dm_pitch.send_data,CAN_TX_MAILBOX0);//发送
}

//急停模式控制所有pid
void control_pid_0_speed()
{
}

//清空数据数据
void Clear_ALL_Data()
{

}

//主跑初始化
void Total_tasks_Init()
{
	ude_pitch.ude.separate_break = 400;
	
	RM_FDorCAN_Init();//can配置初始化
	
	rmClicker.Init();//遥控器串口配置初始化
	
//	servos.Init(&htim2,TIM_CHANNEL_1);//舵机初始化
	
	Get_Chassis_to_Gimbal_Init();//上下板通信
	
	#if GY_GET_SIG == GY_CAN
		
	#elif GY_GET_SIG == GY_232
		ch_gyro_232_Init(&ch_gyro_232_uart_chassis,&chGy_chassis);//地盘陀螺仪初始化
	#endif
	
	HAL_Delay(10);
	
	//设置云台初始角度
	#if GY_GET_SIG == GY_CAN

	#elif GY_GET_SIG == GY_232

	#endif
	
	#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号

	#elif PITCH__GET_SIG == PITCH_GY
		
	#endif
	
	//yaw
	pid_yaw_gy.SetMixI(3,0,5);
	pid_yaw_gy.pid.td_e.r = 2;
	pid_yaw_gy.pid.td_c_c_d.r = 300;
	yaw_target_angle_gy = (int)chGy_chassis.gy.Add_z;
	
	//pitch
	// pid_pitch_gy.SetMixI(10000,0,5);
	// pid_pitch_gy.pid.td_c_c_d.r = 300;
	// ladrc_pitch_gy.set_is_separate_wc(10000,300);
	// ladrc_pitch_gy.set_is_separate_w0(100,500,1200000);
	
	//拨盘设置
	ladrc_dial.set_is_separate_wc(800,300);
	ladrc_dial.set_is_separate_w0(100,5000,500000);
	
	//左摩擦轮设置
	ladrc_friction_l.set_is_separate_wc(800,200);
	ladrc_friction_l.set_is_separate_w0(100,5000,1000000);
	
	//右摩擦轮设置
	ladrc_friction_r.set_is_separate_wc(800,200);
	ladrc_friction_r.set_is_separate_w0(100,5000,1000000);
	
	//记录上一次时间
	uint64_t time_adrc = HAL_GetTick();
	//adrc收敛期
	while(1)
	{
		//pitch收敛

		//拨盘收敛
		ladrc_dial.up_data(0,Motor3508.GetMotor(Dial_MOTOR_R_ID)->Data[Motor_Data_Speed],2);
		
		//左摩擦轮收敛
		ladrc_friction_l.up_data(0,Motor3508.GetMotor(Friction_MOTOR_L_ID)->Data[Motor_Data_Speed],2);
		
		//左摩擦轮收敛
		ladrc_friction_r.up_data(0,Motor3508.GetMotor(Friction_MOTOR_R_ID)->Data[Motor_Data_Speed],2);		

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}


uint8_t send_str2[64];
//主跑函数
void Total_tasks_Run()
{	
//	ch_gyro_232_ore(&ch_gyro_232_uart_chassis,&chGy_chassis);//处理陀螺仪ore问题
//	servos.SetAngle(servos_angle);//舵机
	Get_Chassis_to_Gimbal_ore(&Send_Gimbal_to_Chassis_Huart);
	if(Total_tasks_staticTime.ISOne(2))//控制地盘can发送频率
	{

	}
}

//can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader;	//can接收数据
uint8_t RxHeaderData[8] = { 0 };
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//接受信息 
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxHeaderData);
	if(hcan == &hcan1)
	{
		//6020电机解析
		Motor6020.Parse(RxHeader,RxHeaderData);
		
		//3508电机解析
		Motor3508.Parse(RxHeader,RxHeaderData);
		
		//2006电机解析
		Motor2006.Parse(RxHeader,RxHeaderData);
	}
	if(hcan == &hcan2)
	{
		//CH110陀螺仪
		chGy_chassis.Parse(RxHeader, RxHeaderData);
	}
}

//UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	Get_Chassis_to_Gimbal(huart);
	#if GY_GET_SIG == GY_CAN
		
	#elif GY_GET_SIG == GY_232
		ch_gyro_232_parse(huart,&ch_gyro_232_uart_chassis,&chGy_chassis);
		td_pitch_target_angle_gy.td_quadratic(chGy_chassis.data.y);
//		td_pitch_target_angle_gasy.td_quadratic(chGy_chassis.data.Asy);
	#endif
}

//UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//遥控器解析
}

//云台发送数据到底盘
uint8_t format[100] = { 0xAA };
void Send_Gimbal_to_Chassis()
{
	/*
	帧头 0xAA 8位	
	前进后退符号位 1位 0正 1负
	前进后退 峰值660,压缩到110 倍速6 7位
	左移右移符号位 1位 0正 1负
	左移右移 峰值660,压缩到110 倍速6 7位
	底盘跟随云台误差 32位
	暂停模式 1位 + 底盘跟随云台模式 1位
	*/
//	for(int i = 1;i < Send_Gimbal_to_Chassis_Huart_LEN;i++)format[i] = 0;//复位
//	if((Clicker_chassis_follow_gimbal | Clicker_chassis_follow_gimbal_90 | Clicker_chassis_gyro | On_Friction_Dial_Warehouse_doors | Combinatoria_Movement) == true)//遥控器模式
//	{
//		if(Clicker_chassis_gyro == true)
//		{
//			format[1] = ((RC_LY) < 0 ? 1 : 0) << 7 | (abs((RC_LY) / 6));
//			format[2] = ((-RC_LX) < 0 ? 1 : 0) << 7 | (abs((RC_LX) / 6));
//		}
//		else
//		{
//			if(Clicker_chassis_follow_gimbal | On_Friction_Dial_Warehouse_doors | Combinatoria_Movement)
//			{
//				format[1] = ((RC_LY) < 0 ? 1 : 0) << 7 | (abs((RC_LY) / 6));
//			}
//			else if(Clicker_chassis_follow_gimbal_90)
//			{
//				format[1] = ((-RC_LX) < 0 ? 1 : 0) << 7 | (abs((RC_LX) / 6));
//			}		
//		}
//		/*在组合运动里面6和4等于1,5代表着是小陀螺测试*/
//		format[5] = ((0) << 7/*保留给停止位*/ | 
//								 (Clicker_chassis_follow_gimbal | On_Friction_Dial_Warehouse_doors | Combinatoria_Movement) << 6/*底盘跟随云台*/|
//								 (Clicker_chassis_follow_gimbal_90 | is_on_test) << 5/*侧身*/|
//								 (Clicker_chassis_gyro | Combinatoria_Movement) << 4/*小陀螺*/|
//								 (is_v_reverse) << 3/*速度方向*/|
//								 (1) << 2/*冲刺模式*/|
//								 (0) << 1/*ctrl按键*/);//停止模式		
//		
//		*((int16_t*)&format[6]) = test_gy_v;		
//	}
//	else if(Keyboard_Mouse == true)//键鼠模式
//	{
//		if(now_chassis_mode == 2)//小陀螺
//		{
//			if((KEY_PRESSED_OFFSET_W | KEY_PRESSED_OFFSET_S) == true)
//			{
//				format[1] = ((td_move.x1 > 0 ? 0 : 1) << 7 | (int)(fabsf(td_move.x1) / 6.0));
//			}
//			if((KEY_PRESSED_OFFSET_D | KEY_PRESSED_OFFSET_A) == true)
//			{
//				format[2] = ((td_move1.x1 > 0 ? 0 : 1) << 7 | (int)(fabsf(td_move1.x1) / 6.0));
//			}
//		}
//		else
//		{
//			if((KEY_PRESSED_OFFSET_W | KEY_PRESSED_OFFSET_S) == true && ((now_chassis_mode == 0) || KEY_PRESSED_OFFSET_CTRL == true))
//			{
//				format[1] = ((td_move.x1 > 0 ? 0 : 1) << 7 | (int)(fabsf(td_move.x1) / 6.0));
//			}			
//			if((KEY_PRESSED_OFFSET_D | KEY_PRESSED_OFFSET_A) == true && ((now_chassis_mode == 1) || KEY_PRESSED_OFFSET_CTRL == true))
//			{
//				format[1] = ((td_move.x1 > 0 ? 0 : 1) << 7 | (int)(fabsf(td_move.x1) / 6.0));
//			}
//		}
//		format[5] = ((0) << 7/*保留给停止位*/ | 
//								 (now_chassis_mode == 0 ? 1 : 0) << 6/*底盘跟随云台*/|
//								 (now_chassis_mode == 1 ? 1 : 0) << 5/*侧身*/|
//								 (now_chassis_mode == 2 ? 1 : 0) << 4/*小陀螺*/|
//								 (is_v_reverse) << 3/*速度方向*/|
//								 (Shift_Key_size) << 2/*冲刺模式*/|
//								 (KEY_PRESSED_OFFSET_CTRL == true) << 1/*ctrl按键*/);//停止模式	
//		*((int16_t*)&format[6]) = 0;
//	}
//	
//	*((int16_t*)&format[3]) = (int16_t)(yaw_encoder_angle_e * 20.0);
//	
//	format[8] = ((is_on_MCL) << 7/*摩擦轮*/ | 
//							 (is_on_CM) << 6/*仓门*/ |
//							 (is_on_BP) << 5/*拨盘*/|
//							 (abs(RC_SW) - 600 > 0 ? 1 : 0) << 4//拨轮断电
//							 );
//	
//	format[5] |= ((Emergency_Stop == true) | dir) << 7 | (is_v_reverse) << 3;//停止位
//	
//	HAL_UART_Transmit_DMA(&Send_Gimbal_to_Chassis_Huart, format, Send_Gimbal_to_Chassis_Huart_LEN);
}

//获取云台到底盘数据初始化
void Get_Chassis_to_Gimbal_Init()
{
//	HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Chassis_to_Gimbal_Data.head,1);//先使用中断接收到帧头
}
//获取云台到底盘数据ore
void Get_Chassis_to_Gimbal_ore(UART_HandleTypeDef* huart)//解决ore问题
{
//	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_ORE ) != RESET )
//	{
//		__HAL_UART_CLEAR_OREFLAG( huart );
//		Get_Chassis_to_Gimbal_Init();//获取云台到底盘数据初始化
//	}
}
bool sid = 0;
int kiosdfsdf = 0;
//获取云台到底盘数据
void Get_Chassis_to_Gimbal(UART_HandleTypeDef* huart)
{
//	if(huart == &Send_Gimbal_to_Chassis_Huart)
//	{		
//		//获取帧头
//		if(Chassis_to_Gimbal_Data.head == 0xBB)
//		{
//			Chassis_to_Gimbal_Data.chGy_chassis_data_x = *((float*)&Chassis_to_Gimbal_Data.pData[0]);
//			
//			Chassis_to_Gimbal_Data.magnet_size_last = Chassis_to_Gimbal_Data.magnet_size;
//			Chassis_to_Gimbal_Data.magnet_size = *((uint16_t*)&Chassis_to_Gimbal_Data.pData[4]);//磁铁定位云台
//			
//			if(Chassis_to_Gimbal_Data.magnet_size != Chassis_to_Gimbal_Data.magnet_size_last && Chassis_to_Gimbal_Data.magnet_size >= 5)
//			{
//				yaw_init_encoder_angle = r2g_180(Motor6020.GetMotorDataPos(0x205)) + Fign_Fun(Motor6020.GetMotorDataPos(0x205)) * 5.0 + 73.5314793910;
////				yaw_init_encoder_angle += _yaw_init_encoder_angle_ - yaw_init_encoder_angle;
//			}
//			
//			yaw_init_encoder_angle_MF.MFUpData(yaw_init_encoder_angle,10);
//			
//			Chassis_to_Gimbal_Data.head = Chassis_to_Gimbal_Data.pData[Send_Gimbal_to_Chassis_Huart_LEN - 1];
//							
//			HAL_UART_Receive_DMA(&Send_Gimbal_to_Chassis_Huart,Chassis_to_Gimbal_Data.pData,Send_Gimbal_to_Chassis_Huart_LEN);//开启dma接收
//		}
//		else
//		{
//			HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Chassis_to_Gimbal_Data.head,1);//先使用中断接收到帧头
//		}
//		Chassis_to_Gimbal_Data.dir_time.UpLastTime();//串口更新时间
//	}	
}

float fitting_out(float x)
{
	float a1 = 2909.6;
	float b1 = 0.00073493;
	float c1 = 6.6582;

	return a1*sin(b1*x+c1);
}



float sin_out;
//组合运动
float Combinatoria_Movement_Fun(bool* is_on_test,bool RisingEdge)
{
	static float t = 0,f1 = 0,f2 = 0,f3 = 0,a = 0,b = 0,c = 0;
	if(t >= 20)*is_on_test = false;
	if(*is_on_test == false || RisingEdge == true)t = 0;
	f1 = sinf(0.1 * 2 * pi * t);
	f2 = sinf(0.5 * 2 * pi * t);
	c = Fign_Fun(15.0 - t) * 0.5;
	a = c + 0.5;
	b = -(c - 0.5);
	f3 = (a * f1 + b * f2) * (*is_on_test);
	t += 0.001;
	return f3;
}

float xxsdff = 0,xxsdff1 = 0;
float x0[5];

float ts = 0;
float angelse = 0,angelse1 = 0,sintt,sintts,abcd,abcd1, sin_dc;bool isdf = 0;

float zy = 0;

float out, b = 18, D_out, Kd, k;
int16_t idx = 0;
int16_t data_angle[400];
int16_t data_T[400];
int8_t data_star = 0;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)//回调函数
{
	if(htim == &htim6)
	{
		static bool mode1 = false,mode2 = false;
		mode1 = mode2 = is_run = false;
		is_on_MCL = is_on_BP = false;
		//遥控器
		if((Clicker_chassis_follow_gimbal | Clicker_chassis_follow_gimbal_90 | Clicker_chassis_gyro | On_Friction_Dial_Warehouse_doors | Combinatoria_Movement) == true)
		{
			mode1 = true;
			is_run = true;
			
			if(RC_RY > 15)//移动云台pitch
			{
				pitch_target_angle_gy += 0.02;
			}
			else if(RC_RY < -15)
			{
				pitch_target_angle_gy -= 0.02;
			}
			if(RC_RX > 15)//移动云台yaw
			{
				yaw_target_angle_gy -= 0.2;
			}
			else if(RC_RX < -15)
			{
				yaw_target_angle_gy += 0.2;
			}	
			if(On_Friction_Dial_Warehouse_doors == true)
			{
				int32_t shoot_time_tick = HAL_GetTick();
				
				if(RC_SW > 0 && shoot_time_tick - shoot_time_ms >= shoot_time)
				{
					Dial_angle -= 32764;
					shoot_time_ms = shoot_time_tick;
					is_on_test = 1;		
				}

				Friction_Speed = Friction_MAX_Speed;//设置摩擦轮速度
			}
			else if(Combinatoria_Movement == true)//组合测试
			{
//				Test_key.UpKey(RC_SW <= -600 ? 1 : 0);
//				if(Test_key.RisingEdge == true)is_on_test = 1;
				
//				test_gy_v = Combinatoria_Movement_Fun(&is_on_test,Test_key.RisingEdge) * 1000;		

//				Friction_Speed = Friction_MAX_Speed * (RC_SW > 0 ? 1 : 0);//设置摩擦轮速度
//				Dial_Speed = (float)abs(RC_SW) * (Dial_MAX_Speed / 660.0) * (RC_SW > 0 ? 1 : 0);
				
				test_gy_v = is_on_test = 0;
				if(RC_SW > 0)
				{
					test_gy_v = 1000;
					is_on_test = 1;					
				}
				else if(RC_SW < 0)
				{
					test_gy_v = -1000;
					is_on_test = 1;
				}
				
			}
			else
			{
				is_on_test = Friction_Speed = Dial_Speed = test_gy_v = 0;
				Combinatoria_Movement_Fun(0,0);
			}
		}
		//键鼠
		else if(Keyboard_Mouse == true)
		{
			mode2 = true;
			is_run = true;
			//移动云台pitch
			if(MOUSE_X != 0)//移动云台yaw
			{
				yaw_target_angle_gy += MOUSE_X * MOUSE_X_K;
			}
			//移动云台yaw
			if(MOUSE_Y != 0)//移动云台pitch
			{
				pitch_target_angle_gy += MOUSE_Y * MOUSE_Y_K;
			}
			
			//控制移动速度
			float move_v = 0,move_v1 = 0;
			if(now_chassis_mode == chassis_follow_gimbal)
			{
				if(KEY_PRESSED_OFFSET_W == true)move_v = 660;
				else if(KEY_PRESSED_OFFSET_S == true)move_v = -660;
				else if(KEY_PRESSED_OFFSET_W == false || KEY_PRESSED_OFFSET_S == false) move_v = 0;

				if(KEY_PRESSED_OFFSET_A == true)move_v1 = 660;
				else if(KEY_PRESSED_OFFSET_D == true)move_v1 = -660;
				else if(KEY_PRESSED_OFFSET_A == false || KEY_PRESSED_OFFSET_D == false) move_v1 = 0;

			}
				
			td_move.td_quadratic(move_v);
			td_move1.td_quadratic(move_v1);			
			
			//ctrl键
			Ctrl_key.UpKey(KEY_PRESSED_OFFSET_CTRL);
			
			//e键正身
			E_Key.UpKey(KEY_PRESSED_OFFSET_E);//e键更新
			
			//f键侧身
			F_Key.UpKey(KEY_PRESSED_OFFSET_F);//f键更新
			F_Key_size += F_Key.RisingEdge;//f键更新次数
			F_Key_size %= 2;//f键更新次数取余	
			
			//q键小陀螺
			Q_Key.UpKey(KEY_PRESSED_OFFSET_Q);//q键更新
			Q_Key_size += Q_Key.RisingEdge;//q键更新次数
			Q_Key_size %= 2;//q键更新次数取余	
			
			//c键开仓门
			C_Key.UpKey(KEY_PRESSED_OFFSET_C);//c键开仓门
			//v键开仓门
			V_Key.UpKey(KEY_PRESSED_OFFSET_V);//v键关仓门
			if(C_Key.RisingEdge == true)
			{
				is_on_CM = true;
			}
			else if(V_Key.RisingEdge == true)
			{
				is_on_CM = false;
			}
			
			//设置当前状态
			if(E_Key.RisingEdge == true)Q_Key_size = F_Key_size = 0;//清空小陀螺状态
			else if(F_Key.RisingEdge == true)Q_Key_size = 0;//清空小陀螺状态
			else if(Q_Key.RisingEdge == true)F_Key_size = 0;//清空侧身状态
			if(F_Key_size == 1)now_chassis_mode = chassis_follow_gimbal_90;//设置状态
			else if(Q_Key_size == 1)now_chassis_mode = chassis_gyro;//设置状态
			else now_chassis_mode = chassis_follow_gimbal;//设置状态
			
			//shift键冲刺
			Shift_Key.UpKey(KEY_PRESSED_OFFSET_SHIFT);//shift键更新
			Shift_Key_size = Shift_Key.NowKey;//shift键更新次数			
			
			B_Key.UpKey(KEY_PRESSED_OFFSET_B);//b键更新
			
			G_Key.UpKey(KEY_PRESSED_OFFSET_G);//g键更新
			
			if(B_Key.FallingEdge == true)//b键下降沿
			{
				Friction_Speed = 0;
			}
						
			if(G_Key.FallingEdge == true)//g键下降沿
			{
				Friction_Speed = Friction_MAX_Speed;//设置摩擦轮速度
			}
			
			if(MOUSE_LDOWN == true)//长按鼠标左键开拨盘，否则关闭
			{
				Dial_Speed = Dial_MAX_Speed;
			}
			else
			{
				Dial_Speed = 0;
			}
		}
		else//非运行状态
		{
			Combinatoria_Movement_Fun(0,0);
			now_chassis_mode = Friction_Speed = Dial_Speed = test_gy_v = is_on_test = 0;//复位摩擦轮和拨盘速度...
		}
		Mode1_key.UpKey(mode1);
		Mode2_key.UpKey(mode2);
		if((Mode1_key.FallingEdge | Mode2_key.FallingEdge) == true)
		{
			is_on_MCL = is_on_CM = is_on_BP = false;
			Combinatoria_Movement_Fun(0,0);
			now_chassis_mode = Friction_Speed = Dial_Speed = test_gy_v = is_on_test = 0;//复位摩擦轮和拨盘速度...
		}
		
		//设置是否开启某些功能
		if(Friction_Speed != 0)
		{
			//(当前 - 期度)差值 < 期望的百分之n
			if((fabs((fabs(ladrc_friction_l.z1) + fabs(ladrc_friction_r.z1)) * 0.5) - fabs(Friction_Speed)) < Friction_Speed * 0.4)is_on_MCL = true;
		}
		if(Dial_Speed != 0)
		{
			//(当前 - 期度)差值 < 期望的百分之n
			if((fabs(fabs(ladrc_dial.z1) - fabs(Dial_Speed)) < Dial_Speed * 0.5))is_on_BP = true;
		}
		if((On_Friction_Dial_Warehouse_doors | Combinatoria_Movement) == true)
		{
			is_on_CM = true;
		}
		
		//修改舵机角度
		if(is_on_CM == true)
		{
			servos_angle = servos_angle_on;
		}
		else
		{
			servos_angle = servos_angle_off;
		}
		
		//yaw
		#if GY_GET_SIG == GY_CAN

		#elif GY_GET_SIG == GY_232

		#endif

		#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
				
		#elif PITCH__GET_SIG == PITCH_GY
				
		#endif
		
		//底盘与云台夹角计算		
		is_v_reverse = false;
		temp_yaw_init_encoder_angle = yaw_init_encoder_angle_MF.y;
//		yaw_now_angle = (dm_pitch.Data.position < 0 ? (dm_pitch.Data.position + 3.141593) : dm_pitch.Data.position) * 57.295779;//6.283185
		if((Clicker_chassis_follow_gimbal | On_Friction_Dial_Warehouse_doors | Combinatoria_Movement) == true || 
			 (Keyboard_Mouse == true && now_chassis_mode == chassis_follow_gimbal) || Clicker_chassis_gyro == true || (now_chassis_mode == chassis_gyro))
		{
			//yaw_now_angle = (dm_pitch.Data.position < 0 ? (dm_pitch.Data.position + 3.141593) : dm_pitch.Data.position) * 57.295779;//6.283185
			yaw_now_angle = (Motor6020.GetMotorDataPos(0x205) < 0 ? (Motor6020.GetMotorDataPos(0x205) + 8191) : Motor6020.GetMotorDataPos(0x205)) / 22.75;//6.283185
			if(fabs(Zero_crossing_processing(temp_yaw_init_encoder_angle + 0,
																	yaw_now_angle,360) - yaw_now_angle) <=
				 fabs(Zero_crossing_processing(temp_yaw_init_encoder_angle + 180,
																	yaw_now_angle,360) - yaw_now_angle))
				{
					temp_yaw_init_encoder_angle += 0;
					is_v_reverse = false;
				}
			else 
			{
				temp_yaw_init_encoder_angle += 180;
				is_v_reverse = true;
			}
		}
		
		yaw_encoder_angle_e = (Zero_crossing_processing(temp_yaw_init_encoder_angle,
																										yaw_now_angle,
																										360) - yaw_now_angle);
																								
		//yaw
		//修正p的输出
		kpid_yaw_gy.kp = 0.3 - abs((int16_t)Chassis_to_Gimbal_Data.chGy_chassis_data_x) * 0.005;
		kpid_yaw_gy.kp = kpid_yaw_gy.kp < 0 ? 0 : kpid_yaw_gy.kp;
		if(isdf == 0)
		{
			sintt = 0;
			xxsdff = abcd;
			xxsdff1 = abcd1;
		}
		if(isdf == 1)
		{
			sintt += 0.001;
			xxsdff = angelse * sinf(2.0 * 3.1415926 * sintt * sintts * 0.8) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.2) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.5);
			xxsdff1 = angelse1 * sinf(2.0 * 3.1415926 * sintt * sintts * 0.8) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.2) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.5);
		}
		td_yaw_gy.td_quadratic(yaw_target_angle_gy);
		pid_yaw_gy.Set_customize_d(chGy_chassis.gy.ASz);
		td_yaw_target_angle_gy.td_quadratic((float)((int)(chGy_chassis.gy.Add_z * 100) * 0.01));
		pid_yaw_gy.GetNoLinePid(kpid_yaw_gy,td_yaw_gy.x1 + xxsdff,td_yaw_target_angle_gy.x1,max_yaw_t);
//		yaw_ff.UpData2(td_yaw_gy.x1 + xxsdff);

		td_pitch_Encoder_speed.td_quadratic(Motor6020.GetMotorDataSpeed(PITCH_MOTOR_ID));
		pid_pitch_angle.GetPidPos(kpid_pitch_angle, pitch_target_angle_Encoder, Motor6020.GetMotorDataPos(PITCH_MOTOR_ID), 30000);
		pid_pitch_speed.GetPidPos(kpid_pitch_speed, pid_pitch_angle.pid.cout, td_pitch_Encoder_speed.x1, 30000);

		//pitch				


//		ladrc_pitch_gy.set_customize_z2(chGy_chassis.data.ASx);
//		ladrc_pitch_gy.up_data(pitch_target_angle_gy + xxsdff1,chGy_chassis.data.y,2);
		
		//统一修改摩擦轮
		ladrc_friction_l.b = ladrc_friction_r.b = ladrc_friction_b;
		
		ladrc_friction_l.set_is_separate_wc(ladrc_friction_kp,ladrc_friction_kd);
		ladrc_friction_l.set_is_separate_w0(ladrc_friction_z1,ladrc_friction_z2,ladrc_friction_z3);
		
		ladrc_friction_r.set_is_separate_wc(ladrc_friction_kp,ladrc_friction_kd);
		ladrc_friction_r.set_is_separate_w0(ladrc_friction_z1,ladrc_friction_z2,ladrc_friction_z3);
		
		//拨盘双环控制
		pid_angle_dial.GetPidPos(kpid_angle_dial, Dial_angle, Motor2006.motorData->AddData, 16384);
		pid_speed_dial.GetPidPos(kpid_speed_dial, pid_angle_dial.pid.cout, Motor2006.GetMotorDataSpeed(Dial_MOTOR_R_ID), 16384);

		//左摩擦轮
		ladrc_friction_l.up_data(Friction_Speed,Motor3508.GetMotor(Friction_MOTOR_L_ID)->Data[Motor_Data_Speed],2);
		
		//右摩擦轮
		ladrc_friction_r.up_data(-Friction_Speed,Motor3508.GetMotor(Friction_MOTOR_R_ID)->Data[Motor_Data_Speed],2);
		
		//挂掉状态
//		dir = rmClicker.ISDir() | dm_pitch.is_dir(10);
		//陀螺仪获取端
		#if GY_GET_SIG == GY_CAN
			dir |= chGy_chassis.GetDir();
		#elif GY_GET_SIG == GY_232
			dir |= ch_gyro_232_is_dir(&ch_gyro_232_uart_chassis,&chGy_chassis);
		#endif
		
		//断连清空adrc的z3
//		if(Motor3508.ISDir() != 0)
//		{
//			ladrc_friction_l.z3 = ladrc_friction_r.z3= 0;
//		}
		if(Motor2006.ISDir() != 0)
		{
			ladrc_dial.z3 = 0;
		}
		
		if((Emergency_Stop == true) | (is_run == false))//停止模式
		{                                                                                                                                   
			pid_pitch_angle.clearPID();//清空pid的所有数据
			pid_pitch_speed.clearPID();//清空pid的所有数据

			setMSD(&msd_6020,0,Get_MOTOR_SET_ID_6020(PITCH_MOTOR_ID));
			setMSD(&msd_3508_2006,pid_speed_dial.pid.cout,Get_MOTOR_SET_ID_2006(Dial_MOTOR_R_ID));//拨盘
			setMSD(&msd_3508_2006,ladrc_friction_l.u,Get_MOTOR_SET_ID_3508(Friction_MOTOR_L_ID));//左摩擦轮
			setMSD(&msd_3508_2006,ladrc_friction_r.u,Get_MOTOR_SET_ID_3508(Friction_MOTOR_R_ID));//右摩擦轮

			yaw_target_angle_gy = (int)chGy_chassis.gy.Add_z;
			
		}
		else
		{
			//设置发送数据
			setMSD(&msd_6020, pid_pitch_speed.pid.cout, Get_MOTOR_SET_ID_6020(PITCH_MOTOR_ID));
//			setMSD(&msd_6020,-ladrc_pitch_gy.u,1);
						
			#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
			
			#elif PITCH__GET_SIG == PITCH_GY

			#endif			
			setMSD(&msd_3508_2006,pid_speed_dial.pid.cout,Get_MOTOR_SET_ID_2006(Dial_MOTOR_R_ID));//拨盘
			setMSD(&msd_3508_2006,ladrc_friction_l.u,Get_MOTOR_SET_ID_3508(Friction_MOTOR_L_ID));//左摩擦轮
			setMSD(&msd_3508_2006,ladrc_friction_r.u,Get_MOTOR_SET_ID_3508(Friction_MOTOR_R_ID));//右摩擦轮
		}
	}
	if(htim == &htim5)
	{
//		Send_Gimbal_to_Chassis();		
		//发送数据
		if(send_motor_ms == 0)
		{
			Send_6020_CAN();
		}
		else
		{
			Send_3508_2006_CAN();
		}
		send_motor_ms++;
		send_motor_ms %= 2;
		
	*((float*)&send_str2[0]) = Motor2006.GetMotorDataSpeed(0x201);
	*((float*)&send_str2[4]) = Dial_angle/32764;
	*((float*)&send_str2[8]) = RC_LX;


	*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
	HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str2, sizeof(float) * (7 + 1));
		//给底盘发送数据
	}
}
