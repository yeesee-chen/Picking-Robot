#ifndef __OLED_H
#define __OLED_H


#include "HeaderFiles.h"
#include "iic.h"


#define OLED_Address			0x78
#define OLED_Cmd_Address		0x00
#define OLED_Data_Address		0x40

/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOF, GPIO_Pin_6, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOF, GPIO_Pin_7, (BitAction)(x))

//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2, u8 point);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void fill_picture(unsigned char fill_Data);
void OLED_Refresh_Gram(void);
void Boot_Animation(void);
void OLED_DrawCircle(u8 x0,u8 y0,u8 r);
void OLED_DrawLine(unsigned int x1, unsigned int y1, unsigned int x2,unsigned int y2);
void OLED_ShowFloat(u8 x,u8 y,float num,u8 len,u8 size2);

void OLED_Write_Command(unsigned char IIC_Command);
void OLED_Write_Data(unsigned char IIC_Data);

	
#endif  
