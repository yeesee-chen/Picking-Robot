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
  *  定时中断初始化  *	高级1,8 通用2,3,4,5 基本6,7
  */
void Timer2_Init(uint16_t ARR, uint16_t PSC)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//开启TIM2的时钟

	/*配置时钟源*/
	TIM_InternalClockConfig(TIM2);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器才会用到
	//配置TIM2
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//计数周期，即ARR的值2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元	

	/*中断输出配置*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//清除定时器更新标志位
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//开启TIM2的更新中断

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//选择配置NVIC的TIM2线
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*TIM使能*/
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
}

void Timer3_Init(uint16_t ARR, uint16_t PSC)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM3);		//选择TIM3为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器才会用到
	//配置TIM3
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//计数周期，即ARR的值2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);				//将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元	
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);						//清除定时器更新标志位
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);					//开启TIM3的更新中断
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				//选择配置NVIC的TIM2线
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设
	
	TIM_Cmd(TIM3, ENABLE);
}

void Timer4_Init(uint16_t ARR, uint16_t PSC)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);			//开启TIM3的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM4);		//选择TIM4为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器才会用到
	//配置TIM3
	TIM_TimeBaseInitStructure.TIM_Period = ARR - 1;				//计数周期，即ARR的值2//
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC - 1;				//预分频器，即PSC的值
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);				//将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元	
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);						//清除定时器更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);					//开启TIM3的更新中断
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//选择配置NVIC的TIM2线
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//指定NVIC线路的抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设
	
	TIM_Cmd(TIM4, ENABLE);
}

//程序运行定时器
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Timer_Flag = SET;
		LED1_Turn();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)//超过1秒没有接收到上位机的底盘指令速度自动清零
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

//机械臂控制定时器
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

