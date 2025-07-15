#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"                  // Device header

/* ����1����(���) */
#define USART1_RCC			RCC_APB2Periph_USART1
#define USART1_GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART1_GPIO			GPIOA
#define	USART1_PIN_TX		GPIO_Pin_9
#define	USART1_PIN_RX		GPIO_Pin_10

/* ����2����(���) */
#define USART2_RCC			RCC_APB1Periph_USART2
#define USART2_GPIO_RCC		RCC_APB2Periph_GPIOA
#define USART2_GPIO			GPIOA
#define	USART2_PIN_TX		GPIO_Pin_2
#define	USART2_PIN_RX		GPIO_Pin_3

/* ����3����(����) */
#define USART3_RCC			RCC_APB1Periph_USART3
#define USART3_GPIO_RCC		RCC_APB2Periph_GPIOB
#define USART3_GPIO			GPIOB
#define	USART3_PIN_TX		GPIO_Pin_10
#define	USART3_PIN_RX		GPIO_Pin_11

/* ����4����(��ά��) */
#define USART4_RCC			RCC_APB1Periph_UART4
#define USART4_GPIO_RCC		RCC_APB2Periph_GPIOC
#define USART4_GPIO			GPIOC
#define	USART4_PIN_TX		GPIO_Pin_10
#define	USART4_PIN_RX		GPIO_Pin_11

/* ����5����(��λ��) */
#define USART5_RCC			RCC_APB1Periph_UART5
#define USART5_GPIO_RCC		RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
#define USART5_GPIO_TX		GPIOC
#define USART5_GPIO_RX		GPIOD
#define	USART5_PIN_TX		GPIO_Pin_12
#define	USART5_PIN_RX		GPIO_Pin_2

extern uint8_t Serial5_RxFlag;
extern uint8_t Serial4_RxPacket[100];

extern uint8_t cmd_RxPacket[];		//OLED��ʾ
extern uint8_t Serial4_RxPacketLen;	//��ά�볤��

extern uint8_t RxState;
extern uint8_t pRxPacket;
extern uint8_t Rx_DataCount;
extern uint8_t Rx_NumCount;
extern uint8_t Serial5_RxPacket[100];
extern uint8_t Serial1_RxPacket[100];

extern uint8_t RingBuff[];			//���廷�λ���������
extern volatile uint16_t Ring_head;  // д��λ��
extern volatile uint16_t Ring_tail;  // ��ȡλ��


void Serial_Init(void);

/** ����1���� **/
void Serial1_SendByte(uint8_t Byte);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
void Serial1_SendString(char *String);
void Serial1_SendNumber(uint32_t Number, uint8_t Length);
void Serial1_Printf(char *format, ...);
uint8_t Serial1_GetRxFlag(void);


/** ����2���� **/
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);
void Serial2_SendNumber(uint32_t Number, uint8_t Length);
void Serial2_Printf(char *format, ...);
uint8_t Serial2_GetRxFlag(void);


/** ����3���� **/
void Serial3_SendByte(uint8_t Byte);
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendString(char *String);
void Serial3_SendNumber(uint32_t Number, uint8_t Length);
void Serial3_Printf(char *format, ...);
uint8_t Serial3_GetRxFlag(void);
void UART3_SendCode(uint8_t *pointer ,uint8_t len);

/** ����4���� **/
void Serial4_SendByte(uint8_t Byte);
void Serial4_SendArray(uint8_t *Array, uint16_t Length);
void Serial4_SendString(char *String);
void Serial4_SendNumber(uint32_t Number, uint8_t Length);
void Serial4_Printf(char *format, ...);
uint8_t Serial4_GetRxFlag(void);

/** ����5���� **/
void Serial5_SendByte(uint8_t Byte);
void Serial5_SendArray(uint8_t *Array, uint16_t Length);
void Serial5_SendString(char *String);
void Serial5_SendNumber(uint32_t Number, uint8_t Length);
void Serial5_Printf(char *format, ...);
uint8_t Serial5_GetRxFlag(void);


#endif
