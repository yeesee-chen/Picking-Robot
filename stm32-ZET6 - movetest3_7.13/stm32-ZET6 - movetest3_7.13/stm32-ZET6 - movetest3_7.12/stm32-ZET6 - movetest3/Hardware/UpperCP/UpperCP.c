
/************************* ͷ�ļ� *************************/
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

/************************ �������� ************************/
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

/************************ �������� ************************/
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

/* �������� */
void cmd_func(void)
{
	if(memcmp(ret,"cmd",3) == 0)
	{
		float temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%f",&temp_num);
			Serial5_Printf("num = %f\r\n",temp_num);
		}
		LED3_Turn();
		
		//������ڻ���
		*ret = 0;
	}
}

/* �ٶ� */
void speed_func(void)
{
	if(memcmp(ret,"�ٶ�",4) == 0)
	{
		int temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%d",&temp_num);
			Serial5_Printf("speed = %d\r\n",temp_num);
		}

		/****** ִ�г��� ******/
		speed.tar = temp_num;
		speed.diff = (speed.tar - speed.real)/8;
		
		/******   END   ******/
		*ret = 0;
	}
}

/* ���ٶ� */
void angle_speed_func(void)
{
	if(memcmp(ret,"���ٶ�",6) == 0)
	{
		int temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%d",&temp_num);
			Serial5_Printf("angle_speed = %d\r\n",temp_num);
		}

		/****** ִ�г��� ******/
		angle_speed.tar = temp_num;
		angle_speed.diff = (angle_speed.tar - angle_speed.real)/8;
		
		/******   END   ******/
		*ret = 0;
	}
}

/* �������� */
void voice_func(void)
{
	if(memcmp(ret,"����",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("voice_num = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
		Voice_Num(temp_num);
		/******   END   ******/
		*ret = 0;
	}
}

/* �������� */
void voice_set_func(void)
{
	if(memcmp(ret,"����",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("volumn_set = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
		JQ8x00_Volumn(temp_num);
		
		/******   END   ******/
		*ret = 0;
	}
}

/* ��е�� */
void servo_func(void)
{
	static FlagStatus Run_flag = RESET;
	if(memcmp(ret,"��е��",6) == 0 || Run_flag == SET)
	{		
		/****** ִ��һ�� ******/
		static FlagStatus OnceFlag = SET;
		if(OnceFlag == 1)
		{
			float temp_num;
			char *p_num;
			float Data[3];
			int i=0;
			/****** ��ȡ���� ******/
			for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
			{
				sscanf(p_num,"%f",&temp_num);
				Data[i++] = temp_num;
				Serial5_Printf("num = %f\r\n",temp_num);
			}
			
			/****** ���Գ��� ******/
			#if 1
			Serial5_Printf("\r\nData1 = %f\r\n",Data[0]);
			Serial5_Printf("Data2 = %f\r\n",Data[1]);
			Serial5_Printf("Data3 = %f\r\n",Data[2]);
			#endif
			
			/****** ִ�г��� ******/
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
			moveServo(0, (uint16_t)(Pulse0_Real + D1*T2 + D2*T3), 100);//�˶�ʱ��Ϊ100*Timer_Count
			moveServo(1, (uint16_t)(Pulse1_Real + A1*T2 + A2*T3), 100);
			moveServo(2, (uint16_t)(Pulse2_Real + B1*T2 + B2*T3), 100);
			moveServo(3, (uint16_t)(Pulse3_Real + C1*T2 + C2*T3), 100);
			Timer_Flag = RESET;
			
			Serial5_Printf("\r\n Running... \r\n");		//����
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

/* ��ά�� */
void QR_func(void)
{
	if(memcmp(ret,"��ά��",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
//			Serial5_Printf("num = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
		if(temp_num == 1)
		{
			QR_Star();
			Delay_ms(1000);//ɨ��ʱ��+����ʱ��
			if(Serial4_GetRxFlag() == 1)	//���ճɹ����ر�QR�����Ͷ�ά����Ϣ
			{
				QR_Stop();
//				Delay_ms(100);
				Serial5_SendArray(Serial4_RxPacket+4, Serial4_RxPacketLen);
				memset(Serial4_RxPacket,0,sizeof(Serial4_RxPacket));
			}else							//����ʧ�ܣ��ر�QR������ʧ����Ϣ
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

/* צ�� */
void ZhuaZi_func(void)
{
	if(memcmp(ret,"צ��",4) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("num = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
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

/* �۲�λ */
void Look_func(void)
{
	if(memcmp(ret,"�۲�λ",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("look_pos = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
		Looking(temp_num);
		
		/******   END   ******/
		*ret = 0;
	}
}

/* ������ */
void ActGroup_func(void)
{
	if(memcmp(ret,"������",6) == 0)
	{
		uint8_t temp_num;
		char *p_num;
		/****** ��ȡ���� ******/
		for(p_num = strtok(NULL,",");p_num != NULL;p_num = strtok(NULL,","))
		{
			sscanf(p_num,"%hhu",&temp_num);
			Serial5_Printf("action = %hhu\r\n",temp_num);
		}
		
		/****** ִ�г��� ******/
		runActionGroup(temp_num,1);
		
		/******   END   ******/
		*ret = 0;
	}
}
