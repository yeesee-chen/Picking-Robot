
/************************* 头文件 *************************/
#include "Function.h"
#include "LED.h"
#include "OLED.h"
#include "Serial.h"
#include "Timer.h"
#include "Move.h"
#include "UpperCP.h"
#include "Voice.h"
#include "Servo.h"
#include "Bujin.h"
#include "MyServo.h"
#include "IWDG.h"

/************************* 宏定义 *************************/

/************************ 变量定义 ************************/
uint16_t time=0;

/************************ 函数定义 ************************/


/*模块初始化*/
void System_Init(void)
{
	LED_Init();
	OLED_Init();		//OLED初始化
	Serial_Init();
	Timer2_Init(5000,7200);
	IWDG_Init(5,625);    //与分频数为128,重载值为625,溢出时间为2s	   
}

void UsrFunction(void)
{
	DiffX4_Wheel_Speed_Model_Config(0,0);	//上电电机停止
	ArmInit();
	while(1)
	{
		UpperCP_RX();   	//上位机数据接收
		Move_Updata();      //移动数据更新
		OLED_ShowData();	//OLED刷新
		IWDG_Feed();		//喂狗
	}
}

void OLED_ShowData(void)
{
	static uint8_t once_flag = 1;
	if(once_flag)
	{
		OLED_ShowCHinese(0,0,6);
		OLED_ShowCHinese(16,0,0);//显示汉字
		OLED_ShowCHinese(32,0,1);
		OLED_ShowCHinese(48,0,2);
		OLED_ShowCHinese(64,0,3);
		OLED_ShowCHinese(80,0,4);
		OLED_ShowCHinese(96,0,5);
		OLED_ShowCHinese(112,0,6);
		once_flag = 0;
	}
	OLED_ShowNum(72,2,time++, 4, 16, 0);
		
	OLED_ShowFloat(0, 2, speed.real, 3, 16);
	OLED_ShowFloat(0, 4, angle_speed.real, 3, 16);
}

/****************************End*****************************/

