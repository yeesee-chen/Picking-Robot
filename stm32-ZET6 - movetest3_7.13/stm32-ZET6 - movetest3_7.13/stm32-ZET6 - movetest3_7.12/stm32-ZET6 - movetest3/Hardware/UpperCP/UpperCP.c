
/************************* 头文件 *************************/
#include "UpperCP.h"
#include "Serial.h"
#include "Move.h"
#include "Voice.h"
#include "Servo.h"
#include "Bujin.h"
#include "QR.h"
#include "MyServo.h"
#include "Timer.h"
#include "LED.h"


#define Pai	3.1415926

/************************ 变量定义 ************************/
static uint8_t Timer_Count = 0;
float A1;
float A2;
float B1;
float B2;
float C1;
float C2;
float D1;
float D2;

double thet1=90,thet2=0;
char *ret=NULL;

/************************ 函数定义 ************************/
void UpperCP_RX(void)
{ 
	char Rx_buffer[256];
	uint8_t p_buff=0;
	if(Ring_tail!=Ring_head)
	{
		Serial5_Printf("head=%hu,tail=%hu\r\n",Ring_head,Ring_tail);
		while(Ring_tail!=Ring_head)
		{
			uint8_t ch = RingBuff[Ring_tail];
			Ring_tail = (Ring_tail+1)%256;
			if(ch == ';'){
				Rx_buffer[p_buff] = '\0';
				break;
			}else{
				Rx_buffer[p_buff++] = ch;
			}
		}
		ret = strtok(Rx_buffer,":");
//		Serial5_Printf("ret = %s\r\n",ret);
	}
	cmd_func();
	speed_func();
	angle_speed_func();
	voice_func();
	voice_set_func();
	servo_func();
	QR_func();
	ZhuaZi_func();
	ActGroup_func();
	Look_func();
}

/* 测试命令 */
void cmd_func(void)
{
	if(memcmp(ret,"cmd",3) == 0)
	{
		float temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%f",&temp_num);
			Serial5_Printf("num = %f\r\n",temp_num);
		}
		LED3_Turn();
		
		//清除串口缓存
		*ret = 0;
	}
}

/* 速度 */
void speed_func(void)
{
	if(memcmp(ret,"速度",4) == 0)
	{
		int temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%d",&temp_num);
			Serial5_Printf("speed = %d\r\n",temp_num);
		}

		/****** 执行程序 ******/
		speed.tar = temp_num;
		speed.diff = (speed.tar - speed.real)/8;
		
		/******   END   ******/
		*ret = 0;
	}
}

/* 角速度 */
void angle_speed_func(void)
{
	if(memcmp(ret,"角速度",6) == 0)
	{
		int temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%d",&temp_num);
			Serial5_Printf("angle_speed = %d\r\n",temp_num);
		}

		/****** 执行程序 ******/
		angle_speed.tar = temp_num;
		angle_speed.diff = (angle_speed.tar - angle_speed.real)/8;
		
		/******   END   ******/
		*ret = 0;
	}
}

/* 语音播报 */
void voice_func(void)
{
	if(memcmp(ret,"语音",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("voice_num = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		Voice_Num(temp_num);
		/******   END   ******/
		*ret = 0;
	}
}

/* 音量设置 */
void voice_set_func(void)
{
	if(memcmp(ret,"音量",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("volumn_set = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		JQ8x00_Volumn(temp_num);
		
		/******   END   ******/
		*ret = 0;
	}
}

/* 机械臂 */
void servo_func(void)
{
	static FlagStatus Run_flag = RESET;
	if(memcmp(ret,"机械臂",6) == 0 || Run_flag == SET)
	{		
		/****** 执行一次 ******/
		static FlagStatus OnceFlag = SET;
		if(OnceFlag == 1)
		{
			float temp_num;
			char *p_num;
			float Data[3];
			int i=0;
			/****** 提取数字 ******/
			for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
			{
				sscanf(p_num,"%f",&temp_num);
				Data[i++] = temp_num;
				Serial5_Printf("num = %f\r\n",temp_num);
			}
			
			/****** 测试程序 ******/
			#if 1
			Serial5_Printf("\r\nData1 = %f\r\n",Data[0]);
			Serial5_Printf("Data2 = %f\r\n",Data[1]);
			Serial5_Printf("Data3 = %f\r\n",Data[2]);
			#endif
			
			/****** 执行程序 ******/
			uint8_t T = 24;
			GetPulse(Data[0],Data[1], Data[2]);
			OnceFlag = RESET;
			A1 =  3.0*(Pulse1_Tar - Pulse1_Real)/T/T;
			A2 = -2.0*(Pulse1_Tar - Pulse1_Real)/T/T/T;
			B1 =  3.0*(Pulse2_Tar - Pulse2_Real)/T/T;
			B2 = -2.0*(Pulse2_Tar - Pulse2_Real)/T/T/T;
			C1 =  3.0*(Pulse3_Tar - Pulse3_Real)/T/T;
			C2 = -2.0*(Pulse3_Tar - Pulse3_Real)/T/T/T;
			D1 =  3.0*(Pulse0_Tar - Pulse0_Real)/T/T;
			D2 = -2.0*(Pulse0_Tar - Pulse0_Real)/T/T/T;
			
			Timer2_Init(1000,7200);
			Run_flag = SET;
			LED2_Turn();
		}
		if(Timer_Flag == SET)
		{
			Timer_Count++;
			uint16_t T2 = Timer_Count*Timer_Count;
			uint16_t T3 = Timer_Count*Timer_Count*Timer_Count;
			moveServo(0, (uint16_t)(Pulse0_Real + D1*T2 + D2*T3), 100);//运动时间为100*Timer_Count
			moveServo(1, (uint16_t)(Pulse1_Real + A1*T2 + A2*T3), 100);
			moveServo(2, (uint16_t)(Pulse2_Real + B1*T2 + B2*T3), 100);
			moveServo(3, (uint16_t)(Pulse3_Real + C1*T2 + C2*T3), 100);
			Timer_Flag = RESET;
			
			Serial5_Printf("\r\n Running... \r\n");		//测试
		}
		if(Timer_Count == 24)
		{
			TIM_Cmd(TIM2, DISABLE);
			Pulse0_Real = Pulse0_Tar;
			Pulse1_Real = Pulse1_Tar;
			Pulse2_Real = Pulse2_Tar;
			Pulse3_Real = Pulse3_Tar;
			LED2_Turn();
			Timer_Count = 0;
			OnceFlag = SET;
			Run_flag = RESET;
			*ret = 0;
		}
	}
}

/* 二维码 */
void QR_func(void)
{
	if(memcmp(ret,"二维码",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
//			Serial5_Printf("num = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		if(temp_num == 1)
		{
			QR_Star();
			Delay_ms(1000);//扫描时间+接收时间
			if(Serial4_GetRxFlag() == 1)	//接收成功，关闭QR，发送二维码信息
			{
				QR_Stop();
//				Delay_ms(100);
				Serial5_SendArray(Serial4_RxPacket+4, Serial4_RxPacketLen);
				memset(Serial4_RxPacket,0,sizeof(Serial4_RxPacket));
			}else							//接收失败，关闭QR，发送失败信息
			{
				QR_Stop();
//				Delay_ms(100);
				Serial5_SendByte(0xDD);
			}		
		}
		/******   END   ******/
		*ret = 0;
	}
}

/* 爪子 */
void ZhuaZi_func(void)
{
	if(memcmp(ret,"爪子",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("num = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		if(temp_num == 0)
		{
			ZhuaZi_close();
			LED1_Turn();
		}
		if(temp_num == 1)
		{
			ZhuaZi_open();
			LED1_Turn();
		}
		/******   END   ******/
		*ret = 0;
	}
}

/* 观测位 */
void Look_func(void)
{
	if(memcmp(ret,"观测位",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("look_pos = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		Looking(temp_num);
		
		/******   END   ******/
		*ret = 0;
	}
}

/* 动作组 */
void ActGroup_func(void)
{
	if(memcmp(ret,"动作组",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** 提取数字 ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("action = %hhu\r\n",temp_num);
		}
		
		/****** 执行程序 ******/
		runActionGroup(temp_num,1);
		
		/******   END   ******/
		*ret = 0;
	}
}
