#pragma once
#include "RM_stm32fxxx_hal.h"
#include "RM_StaticTime.h"
#define KEY_PRESSED_OFFSET_W       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 0  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_S       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 1  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_A       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 2  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_D       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 3  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_SHIFT   (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 4  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_CTRL    (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 5  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_Q       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 6  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_E       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 7  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_R       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 8  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_F       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 9  & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_G       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 10 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_Z       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 11 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_X       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 12 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_C       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 13 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_V       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 14 & (uint8_t)0x01)
#define KEY_PRESSED_OFFSET_B       (uint8_t)(RM_Clicker::RC_Ctl.key.v >> 15 & (uint8_t)0x01)
#define MOUSE_RDOWN								 (uint8_t)(RM_Clicker::RC_Ctl.mouse.press_r)	 
#define MOUSE_LDOWN								 (uint8_t)(RM_Clicker::RC_Ctl.mouse.press_l)
#define MOUSE_X										 (float)(RM_Clicker::RC_Ctl.mouse.x)
#define MOUSE_Y										 (float)(RM_Clicker::RC_Ctl.mouse.y)
#define RC_LX                      (int16_t)(RM_Clicker::RC_Ctl.rc.ch2 - 1024)
#define RC_LY                      (int16_t)(RM_Clicker::RC_Ctl.rc.ch3 - 1024)
#define RC_RX                      (int16_t)(RM_Clicker::RC_Ctl.rc.ch0 - 1024)
#define RC_RY                      (int16_t)(RM_Clicker::RC_Ctl.rc.ch1 - 1024)
#define RC_SW                      (int16_t)(RM_Clicker::RC_Ctl.rc.sw - 1024)

#define ClickerHuart huart3
//结构体
typedef struct
{
 struct 
 {
	 uint16_t ch0;
	 uint16_t ch1;
	 uint16_t ch2;
	 uint16_t ch3;
	 uint8_t s1;
	 uint8_t s2;
	 int16_t sw;
 }rc;
 struct 
 {
	 int16_t x;
	 int16_t y;
	 int16_t z;
	 uint8_t press_l;
	 uint8_t press_r;
 }mouse;
 struct 
 {
	 uint16_t v;
 }key;
 char Dir;
}RC_Ctl_t;

class RM_Clicker
{
private:
  //遥控器数据解析
  static RC_Ctl_t ParseData();
public:
	RM_Clicker();
  static RC_Ctl_t RC_Ctl;
  static RM_StaticTime dirTime;
  static uint8_t pData[18];
  //解除ore
  static void ClearORE(UART_HandleTypeDef* huart,uint8_t* pData,int Size);
  //遥控器初始化
  static void Init();
  //判断遥控器断连
  static bool ISDir();
  //遥控器数据解析
  static void Parse(UART_HandleTypeDef* huart,int Size);
};

RC_Ctl_t RM_Clicker::RC_Ctl = { 0 };//初始化
RM_StaticTime RM_Clicker::dirTime = { 0 };//初始化
uint8_t RM_Clicker::pData[18] = { 0 };//初始化
static RM_Clicker rmClicker;

inline void RM_Clicker::ClearORE(UART_HandleTypeDef* huart,uint8_t* pData,int Size)
{
	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_ORE ) != RESET )
		{
			__HAL_UART_CLEAR_OREFLAG( huart );
			HAL_UARTEx_ReceiveToIdle_IT( huart, pData, Size );
		}
}

inline void RM_Clicker::Init()
{
	HAL_UARTEx_ReceiveToIdle_IT(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
}

inline RC_Ctl_t RM_Clicker::ParseData()
{
	RC_Ctl_t RC_CtrlData = { 0 };    
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
	RC_CtrlData.rc.sw = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8) ;
	return RC_CtrlData;
}

inline RM_Clicker::RM_Clicker()
{
	RM_Clicker::Init();
}

inline bool RM_Clicker::ISDir()
{
  char Dir = 0;
	Dir |= ~(RC_Ctl.rc.ch0 >= 364) & (RC_Ctl.rc.ch0 <= 1684);
	Dir |= ~(RC_Ctl.rc.ch1 >= 364) & (RC_Ctl.rc.ch1 <= 1684);
	Dir |= ~(RC_Ctl.rc.ch2 >= 364) & (RC_Ctl.rc.ch2 <= 1684);
	Dir |= ~(RC_Ctl.rc.ch3 >= 364) & (RC_Ctl.rc.ch3 <= 1684);
	RC_Ctl.Dir = RM_Clicker::dirTime.ISDir(50) | Dir;
	if(RC_Ctl.Dir)RM_Clicker::ClearORE(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
  return RC_Ctl.Dir;
}

inline void RM_Clicker::Parse(UART_HandleTypeDef *huart, int Size)
{
  //本机遥控器
	if(huart == &ClickerHuart && Size == sizeof(RM_Clicker::pData))
	{
		RM_Clicker::RC_Ctl = RM_Clicker::ParseData();
		RM_Clicker::dirTime.UpLastTime();
	}
	HAL_UARTEx_ReceiveToIdle_IT(&ClickerHuart,RM_Clicker::pData,sizeof(RM_Clicker::pData));
}
