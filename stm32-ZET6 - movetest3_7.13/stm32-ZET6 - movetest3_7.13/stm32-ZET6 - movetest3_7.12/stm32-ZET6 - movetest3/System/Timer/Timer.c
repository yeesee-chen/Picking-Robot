#include "Timer.h"
#include "Bujin.h"
#include "Move.h"
#include "Serial.h"
#include "LED.h"

FlagStatus Timer_Flag = RESET;
FlagStatus Timer1_Flag = RESET;
FlagStatus Timer4_Flag = RESET;

extern uint8_t cmd_RxPacket[];
uint16_t Flag99=0;


 /*
  *  ��ʱ�жϳ�ʼ��  *	�߼�1,8 ͨ��2,3,4,5 ����6,7
  */
void Timer2_Init(uint16_t ARR, uint16_t PSC)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//����TIM2��ʱ��

	/*����ʱ��Դ*/
	TIM_InternalClockConfig(TIM2);		//ѡ��TIM2Ϊ�ڲ�ʱ�ӣ��������ô˺�����TIMĬ��ҲΪ�ڲ�ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	//����TIM2
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//�������ڣ���ARR��ֵ2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ	

	/*�ж��������*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//�����ʱ�����±�־λ
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//����TIM2�ĸ����ж�

	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����NVICΪ����2
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//ѡ������NVIC��TIM2��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	/*TIMʹ��*/
	TIM_Cmd(TIM2, ENABLE);			//ʹ��TIM2����ʱ����ʼ����
}

void Timer3_Init(uint16_t ARR, uint16_t PSC)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//����TIM3��ʱ��
	
	/*����ʱ��Դ*/
	TIM_InternalClockConfig(TIM3);		//ѡ��TIM3Ϊ�ڲ�ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	//����TIM3
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//�������ڣ���ARR��ֵ2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ	
	
	/*�ж��������*/
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);						//�����ʱ�����±�־λ
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);					//����TIM3�ĸ����ж�
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				//ѡ������NVIC��TIM2��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	TIM_Cmd(TIM3, ENABLE);
}

void Timer4_Init(uint16_t ARR, uint16_t PSC)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//����TIM3��ʱ��
	
	/*����ʱ��Դ*/
	TIM_InternalClockConfig(TIM4);		//ѡ��TIM4Ϊ�ڲ�ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	//����TIM3
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//�������ڣ���ARR��ֵ2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ	
	
	/*�ж��������*/
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);						//�����ʱ�����±�־λ
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);					//����TIM3�ĸ����ж�
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//ѡ������NVIC��TIM2��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	TIM_Cmd(TIM4, ENABLE);
}

//�������ж�ʱ��
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Timer_Flag = SET;
		LED1_Turn();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)//����1��û�н��յ���λ���ĵ���ָ���ٶ��Զ�����
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		
		Timer1_Flag=SET;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		Flag99++;
		if(Flag99==10){
			//DiffX4_Wheel_Speed_Model_Config(0,0);
			//Serial5_Printf("99=%d",Flag99);			
			/*Emm_V5_Stop_Now(1,0);
			Emm_V5_Stop_Now(2,0);
			Emm_V5_Stop_Now(3,0);
			Emm_V5_Stop_Now(4,0);*/
			cmd_RxPacket[0] = 0x01;
			cmd_RxPacket[1] = 0x00;
			cmd_RxPacket[2] = 0x00;
			Flag99=0;
		}
	}
}

//��е�ۿ��ƶ�ʱ��
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_Cmd(TIM4, DISABLE);
		Timer4_Flag = SET;
		RxState = 0;
		pRxPacket = 0;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

