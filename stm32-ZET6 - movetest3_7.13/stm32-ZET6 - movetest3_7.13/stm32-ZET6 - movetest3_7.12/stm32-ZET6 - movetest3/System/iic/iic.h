#ifndef __IIC1_H__
#define __IIC1_H__

#include "stm32f10x.h"
#include "OLED.h"


///*�˿�����*/
//#define IIC_SCL         PBout(6)       //SCLK  ʱ��
//#define IIC_SDA         PBout(7)       //SDA   д����
//#define READ_SDA        PBin(7)        //SDA   ������


void I2C1_Init(void);
void I2C1_Write_Byte(u8 device_address,u8 register_address,u8 data);
u8 I2C1_Read_Data(u8 device_address,u8 register_address);


#endif

