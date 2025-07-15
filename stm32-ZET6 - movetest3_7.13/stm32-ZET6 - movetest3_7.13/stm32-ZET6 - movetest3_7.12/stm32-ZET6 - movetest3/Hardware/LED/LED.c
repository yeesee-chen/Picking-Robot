#include "LED.h"

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_ResetBits(GPIOF, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
}

/**
  * 函    数：LED1状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_8) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
		GPIO_SetBits(GPIOF, GPIO_Pin_8);					//则设置PA1引脚为高电平
	else													//否则，即当前引脚输出高电平
		GPIO_ResetBits(GPIOF, GPIO_Pin_8);					//则设置PA1引脚为低电平
}

/**
  * 函    数：LED2状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_9) == 0)
		GPIO_SetBits(GPIOF, GPIO_Pin_9);
	else 
		GPIO_ResetBits(GPIOF, GPIO_Pin_9);
}

/**
  * 函    数：LED3状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED3_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_10) == 0)
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
	else 
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
}

