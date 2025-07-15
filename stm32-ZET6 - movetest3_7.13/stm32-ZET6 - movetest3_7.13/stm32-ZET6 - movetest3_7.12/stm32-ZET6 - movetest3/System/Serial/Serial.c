#include "HeaderFiles.h"
#include "Serial.h"
#include "QR.h"
#include "Timer.h"
#include "LED.h"

#define RingBuff_Lenth 256

uint8_t Serial1_RxFlag;					//定义接收数据包标志位
uint8_t Serial2_RxFlag;					//定义接收数据包标志位
uint8_t Serial3_RxFlag;					//定义接收数据包标志位
uint8_t Serial4_RxFlag;					//定义接收数据包标志位
uint8_t Serial5_RxFlag=0;					//定义接收数据包标志位

uint8_t cmd_RxPacket[5];// 				//定义命令数据包数组
uint8_t Serial4_RxPacketLen = 0;
uint8_t Serial4_RxPacket[100];

/* 上位机变量 */
uint8_t Serial5_TxPacket[100];				//定义发送数据包数组，数据包格式：FF 01 02 03 FE
uint8_t Serial5_RxPacket[100];				//定义接收数据包数组
uint8_t RingBuff[RingBuff_Lenth];			//定义环形缓冲区数组
volatile uint16_t Ring_head=0;  // 写入位置
volatile uint16_t Ring_tail=0;  // 读取位置

uint8_t Serial1_RxPacket[100];				//定义接收数据包数组
uint8_t RxState = 0;		//定义表示当前状态机状态的静态变量
uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
uint8_t Rx_DataCount = 0;
uint8_t Rx_NumCount = 0;

extern uint16_t Flag99;

/* PID变量 */
float error = 0;      //角度误差
float last_error = 0; //上次误差
float ErrorInt = 0;   //误差累积
float SpeedOut = 0;   //输出结果
float kp_A2 = 0.08;   //旋转kp
float ki_A2 = 0;      //旋转ki
float kd_A2 = 0.08;   //旋转kd


/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/******************** 串口1配置 ********************/
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(USART1_RCC, ENABLE);			//开启USART1的时钟
	RCC_APB2PeriphClockCmd(USART1_GPIO_RCC, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	//TX初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = USART1_PIN_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	//RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = USART1_PIN_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART1初始化*/
	USART_InitTypeDef USART_InitStructure;							//定义结构体变量
	USART_InitStructure.USART_BaudRate = 9600;					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;				//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);						//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2	4抢占4优先
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
	
	 
	/******************** 串口2配置 ********************/
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE );
	 
	/*GPIO初始化*/
	//TX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA2引脚初始化为复用推挽输出
	//RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA3引脚初始化为上拉输入
	 
	/*USART1初始化*/
	USART_InitStructure.USART_BaudRate = 115200;					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;				//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART2, &USART_InitStructure);						//将结构体变量交给USART_Init，配置USART2	 
	 
	/*中断输出配置*/
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
	/*NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//指定NVIC线路的响应优先级为0
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART2, ENABLE);								//使能USART2，串口开始运行
	 
	 
	/******************** 串口3配置 ********************/
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(USART3_GPIO_RCC, ENABLE );
	RCC_APB1PeriphClockCmd(USART3_RCC, ENABLE );
	 
	/*GPIO初始化*/
	//TX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = USART3_PIN_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);
	//RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = USART3_PIN_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);
	 
	/*USART3初始化*/
	USART_InitStructure.USART_BaudRate = 9600;					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);						//将结构体变量交给USART_Init，配置USART3
	 
	/*中断输出配置*/
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
	/*NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//选择配置NVIC的USART3线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//指定NVIC线路的响应优先级为2
	NVIC_Init(&NVIC_InitStructure);
	
	/*USART使能*/
	USART_Cmd(USART3, ENABLE);								//使能USART3，串口开始运行
	
	
	/******************** 串口4配置 ********************/
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(USART4_GPIO_RCC, ENABLE );
	RCC_APB1PeriphClockCmd(USART4_RCC, ENABLE);
	 
	/*GPIO初始化*/
	//TX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = USART4_PIN_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART4_GPIO, &GPIO_InitStructure);
	//RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = USART4_PIN_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART4_GPIO, &GPIO_InitStructure);
	 
	/*USART4初始化*/
	USART_InitStructure.USART_BaudRate = 9600;					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(UART4, &USART_InitStructure);						//将结构体变量交给UART_Init，配置UART4
	 
	/*中断输出配置*/
	USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		//选择配置NVIC的USART4线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//指定NVIC线路的响应优先级为2
	NVIC_Init(&NVIC_InitStructure);
	
	/*USART使能*/
	USART_Cmd(UART4, ENABLE);								//使能UART4，串口开始运行
	

	/******************** 串口5配置 ********************/
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(USART5_GPIO_RCC, ENABLE );
	RCC_APB1PeriphClockCmd(USART5_RCC, ENABLE );
	 
	/*GPIO初始化*/
	//TX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = USART5_PIN_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART5_GPIO_TX, &GPIO_InitStructure);
	//RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = USART5_PIN_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART5_GPIO_RX, &GPIO_InitStructure);
	 
	/*USART5初始化*/
	USART_InitStructure.USART_BaudRate = 115200;					//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(UART5, &USART_InitStructure);						//将结构体变量交给USART_Init，配置USART5
	 
	/*中断输出配置*/
	USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
		
	/*NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;		//选择配置NVIC的USART5线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//指定NVIC线路的响应优先级为2
	NVIC_Init(&NVIC_InitStructure);
	
	/*USART使能*/
	USART_Cmd(UART5, ENABLE);								//使能USART5，串口开始运行
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}
void Serial2_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}
void Serial3_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}
void UART3_SendCode(uint8_t *pointer ,uint8_t len)
{
	for(uint8_t i=0; i<len; i++)
	{
		USART_SendData(USART3, pointer[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
}
void Serial4_SendByte(uint8_t Byte)
{
	USART_SendData(UART4, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}
void Serial5_SendByte(uint8_t Byte)
{
	USART_SendData(UART5, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}


/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial1_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial2_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial3_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial3_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial4_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial4_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial5_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial5_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial1_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial1_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial2_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial2_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial3_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial3_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial4_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial4_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
void Serial5_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial5_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}


/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial1_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial1_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}
void Serial2_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial2_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}
void Serial3_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial3_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}
void Serial4_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial4_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}
void Serial5_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial5_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc1(int ch, FILE *f)
{
	Serial1_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}
int fputc2(int ch, FILE *f)
{
	Serial2_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}
int fputc3(int ch, FILE *f)
{
	Serial3_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}
int fputc4(int ch, FILE *f)
{
	Serial4_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}
int fputc5(int ch, FILE *f)
{
	Serial5_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial1_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial1_SendString(String);		//串口发送字符数组（字符串）
}
void Serial2_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial2_SendString(String);		//串口发送字符数组（字符串）
}
void Serial3_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial3_SendString(String);		//串口发送字符数组（字符串）
}
void Serial4_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial4_SendString(String);		//串口发送字符数组（字符串）
}
void Serial5_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial5_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：获取串口接收数据包标志位
  * 参    数：无
  * 返 回 值：串口接收数据包标志位，范围：0~1，接收到数据包后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial1_GetRxFlag(void)
{
	if (Serial1_RxFlag == 1)			//如果标志位为1
	{
		Serial1_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
uint8_t Serial2_GetRxFlag(void)
{
	if (Serial2_RxFlag == 1)			//如果标志位为1
	{
		Serial2_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
uint8_t Serial3_GetRxFlag(void)
{
	if (Serial3_RxFlag == 1)			//如果标志位为1
	{
		Serial3_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
uint8_t Serial4_GetRxFlag(void)
{
	if (Serial4_RxFlag == 1)			//如果标志位为1
	{
		Serial4_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
uint8_t Serial5_GetRxFlag(void)
{
	if (Serial5_RxFlag == 1)			//如果标志位为1
	{
		Serial5_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}


/***********  配置了串口中断一定要写中断服务函数!!!!!!!  ***************/
void USART1_IRQHandler(void)
{	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET){
		uint8_t RxData = USART_ReceiveData(USART1);
		Serial1_SendByte(RxData);
	}
}

void USART2_IRQHandler(void)
{	
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET){
		uint8_t RxData = USART_ReceiveData(USART2);
		Serial2_SendByte(RxData);
	}
}

void USART3_IRQHandler(void)
{	
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET){
		uint8_t RxData = USART_ReceiveData(USART3);
		Serial3_SendByte(RxData);
	}
}


/**
  * 函    数：USART4中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */

void UART4_IRQHandler(void)
{
	static uint8_t RxState = 0;		//定义表示当前状态机状态的静态变量
	static uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET){		//判断是否是USART3的接收事件触发的中断
		uint8_t RxData = USART_ReceiveData(UART4);		//读取数据寄存器，存放在接收的数据变量
		
//		Serial5_SendByte(RxData);

		Serial4_RxPacket[pRxPacket++] = RxData;
		if(RxData == 0x0D)  //结束符
		{
			Serial4_RxPacketLen=pRxPacket-1;
			Serial4_RxFlag = 1;
			RxData = 0;
			pRxPacket=0;
//			Serial5_SendArray(Serial4_RxPacket, Serial4_RxPacketLen);
		}
	}
}


/**
  * 函    数：USART5中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void UART5_IRQHandler(void)
{	
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET){
		uint8_t RxData = USART_ReceiveData(UART5);
		Serial5_SendByte(RxData);
		RingBuff[Ring_head++] = RxData;
		Ring_head = (Ring_head)%256;
	}
}


