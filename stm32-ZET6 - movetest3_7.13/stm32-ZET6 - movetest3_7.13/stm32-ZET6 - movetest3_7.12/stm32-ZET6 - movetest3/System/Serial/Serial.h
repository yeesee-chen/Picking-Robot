#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"                  // Device header

/* 串口1定义(舵机) */
#define USART1_RCC			RCC_APB2Periph_USART1
#define USART1_GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART1_GPIO			GPIOA
#define	USART1_PIN_TX		GPIO_Pin_9
#define	USART1_PIN_RX		GPIO_Pin_10

/* 串口2定义(电机) */
#define USART2_RCC			RCC_APB1Periph_USART2
#define USART2_GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART2_GPIO			GPIOA
#define	USART2_PIN_TX		GPIO_Pin_2
#define	USART2_PIN_RX		GPIO_Pin_3

/* 串口3定义(语音) */
#define USART3_RCC			RCC_APB1Periph_USART3
#define USART3_GPIO_RCC		RCC_APB2Periph_GPIOB
#define USART3_GPIO			GPIOB
#define	USART3_PIN_TX		GPIO_Pin_10
#define	USART3_PIN_RX		GPIO_Pin_11

/* 串口4定义(二维码) */
#define USART4_RCC			RCC_APB1Periph_UART4
#define USART4_GPIO_RCC		RCC_APB2Periph_GPIOC
#define USART4_GPIO			GPIOC
#define	USART4_PIN_TX		GPIO_Pin_10
#define	USART4_PIN_RX		GPIO_Pin_11

/* 串口5定义(上位机) */
#define USART5_RCC			RCC_APB1Periph_UART5
#define USART5_GPIO_RCC		RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
#define USART5_GPIO_TX		GPIOC
#define USART5_GPIO_RX		GPIOD
#define	USART5_PIN_TX		GPIO_Pin_12
#define	USART5_PIN_RX		GPIO_Pin_2

extern uint8_t Serial5_RxFlag;
extern uint8_t Serial4_RxPacket[100];

extern uint8_t cmd_RxPacket[];		//OLED显示
extern uint8_t Serial4_RxPacketLen;	//二维码长度

extern uint8_t RxState;
extern uint8_t pRxPacket;
extern uint8_t Rx_DataCount;
extern uint8_t Rx_NumCount;
extern uint8_t Serial5_RxPacket[100];
extern uint8_t Serial1_RxPacket[100];

extern uint8_t RingBuff[];			//定义环形缓冲区数组
extern volatile uint16_t Ring_head;  // 写入位置
extern volatile uint16_t Ring_tail;  // 读取位置


void Serial_Init(void);

/** 串口1函数 **/
void Serial1_SendByte(uint8_t Byte);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
void Serial1_SendString(char *String);
void Serial1_SendNumber(uint32_t Number, uint8_t Length);
void Serial1_Printf(char *format, ...);
uint8_t Serial1_GetRxFlag(void);


/** 串口2函数 **/
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);
void Serial2_SendNumber(uint32_t Number, uint8_t Length);
void Serial2_Printf(char *format, ...);
uint8_t Serial2_GetRxFlag(void);


/** 串口3函数 **/
void Serial3_SendByte(uint8_t Byte);
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendString(char *String);
void Serial3_SendNumber(uint32_t Number, uint8_t Length);
void Serial3_Printf(char *format, ...);
uint8_t Serial3_GetRxFlag(void);
void UART3_SendCode(uint8_t *pointer ,uint8_t len);

/** 串口4函数 **/
void Serial4_SendByte(uint8_t Byte);
void Serial4_SendArray(uint8_t *Array, uint16_t Length);
void Serial4_SendString(char *String);
void Serial4_SendNumber(uint32_t Number, uint8_t Length);
void Serial4_Printf(char *format, ...);
uint8_t Serial4_GetRxFlag(void);

/** 串口5函数 **/
void Serial5_SendByte(uint8_t Byte);
void Serial5_SendArray(uint8_t *Array, uint16_t Length);
void Serial5_SendString(char *String);
void Serial5_SendNumber(uint32_t Number, uint8_t Length);
void Serial5_Printf(char *format, ...);
uint8_t Serial5_GetRxFlag(void);


#endif
