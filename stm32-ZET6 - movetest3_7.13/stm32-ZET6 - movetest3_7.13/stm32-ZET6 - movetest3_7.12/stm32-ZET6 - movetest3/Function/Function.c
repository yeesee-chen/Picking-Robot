
/************************* ͷ�ļ� *************************/
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

/************************* �궨�� *************************/

/************************ �������� ************************/
uint16_t time=0;

/************************ �������� ************************/


/*ģ���ʼ��*/
void System_Init(void)
{
	LED_Init();
	OLED_Init();		//OLED��ʼ��
	Serial_Init();
	Timer2_Init(5000,7200);
	IWDG_Init(5,625);    //���Ƶ��Ϊ128,����ֵΪ625,���ʱ��Ϊ2s	   
}

void UsrFunction(void)
{
	DiffX4_Wheel_Speed_Model_Config(0,0);	//�ϵ���ֹͣ
	ArmInit();
	while(1)
	{
		UpperCP_RX();   	//��λ�����ݽ���
		Move_Updata();      //�ƶ����ݸ���
		OLED_ShowData();	//OLEDˢ��
		IWDG_Feed();		//ι��
	}
}

void OLED_ShowData(void)
{
	static uint8_t once_flag = 1;
	if(once_flag)
	{
		OLED_ShowCHinese(0,0,6);
		OLED_ShowCHinese(16,0,0);//��ʾ����
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

