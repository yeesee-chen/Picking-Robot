#include "OLED.h"
#include "OLED_Font.h"

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127 	   

u8 OLED_GRAM[128][8];	


void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
 	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}



/**********************************************
// IIC Write Command
**********************************************/
void OLED_Write_Command(unsigned char IIC_Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(IIC_Command); 
	OLED_I2C_Stop();
}


/**********************************************
// IIC Write Data
**********************************************/
void OLED_Write_Data(unsigned char IIC_Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(IIC_Data);
	OLED_I2C_Stop();
}


void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
		OLED_Write_Data(dat);
	}
	else
	{
   OLED_Write_Command(dat);
	}
}


/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);			//page0-page1
		OLED_WR_Byte(0x00,0);				//low column start address
		OLED_WR_Byte(0x10,0);				//high column start address
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(fill_Data,1);
		}
	}
}


//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0+y,0);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,0);
	OLED_WR_Byte((x&0x0f),0);
}


//开启OLED显示
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,0);  //SET DCDC命令
	OLED_WR_Byte(0X14,0);  //DCDC ON
	OLED_WR_Byte(0XAF,0);  //DISPLAY ON
}


//关闭OLED显示
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,0);  //SET DCDC命令
	OLED_WR_Byte(0X10,0);  //DCDC OFF
	OLED_WR_Byte(0XAE,0);  //DISPLAY OFF
}


//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
	u8 i,n;
	for(n=0;n<8;n++)
	{
		for(i=0;i<128;i++)
		{
			OLED_GRAM[i][n]=0;
		}
	}
	OLED_Refresh_Gram();
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,0);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,0);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,0);      //设置显示位置—列高地址
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(0,1);
		}
	}	//更新显示
}


//画点
//x:0~127
//y:0~7
//t:1 填充 0,清空
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)
	{
		return;//超出范围了
	}
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)
	{
		OLED_GRAM[x][pos]|=temp;
	}
	else 
	{
		OLED_GRAM[x][pos]&=~temp;
	}
	OLED_Refresh_Gram();
}


//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63
//dot:0,清空;1,填充
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)
{
	u8 x,y;
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)
		{
			OLED_DrawPoint(x,y,dot);
		}
	}
	OLED_Refresh_Gram();//更新显示
}


void OLED_On(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,0);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,0);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,0);      //设置显示位置—列高地址
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(1,1);
		}
	}	//更新显示
}


//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~7
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{
	unsigned char c=0,i=0;
	c=chr-' ';//得到偏移后的值
	if(x>128-1)
	{
		x=0;
		y=y+2;
	}
	if(Char_Size ==16)
	{
		OLED_Set_Pos(x,y);
		for(i=0;i<8;i++)
		{
			OLED_WR_Byte(F8X16[c*16+i],1);
		}
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		{
			OLED_WR_Byte(F8X16[c*16+i+8],1);
		}
	}
	else
	{
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		{
			OLED_WR_Byte(F6x8[c][i],1);
		}
	}
}


//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)
	{
		result*=m;
	}
	return result;
}


//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
//point:1整数， 0小数
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2, u8 point)
{             
    u8 t,temp;
    u8 enshow=0;                           
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;//依次提出最高位
        if(enshow==0 && t<(len))
        {
            if(temp==0 && point)
            {
				if(t!=len-1)
				{
					OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
					continue;
				}
            }
            else 
                enshow=1;   
        }
         OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2);
    }
} 

//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{
		OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120)
		{
			x=0;
			y+=2;
		}
		j++;
	}
}


//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{
	u8 t,adder=0;
	OLED_Set_Pos(x,y);
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no][t],1);
		adder+=1;
	}
	OLED_Set_Pos(x,y+1);
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no+1][t],1);
		adder+=1;
	}
}

//更新显存到
void OLED_Refresh_Gram(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,0);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,0);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,0);      //设置显示位置—列高地址
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(OLED_GRAM[n][i],1);
		}
	}
}


/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;
	
  if(y1%8==0)
	{
		y=y1/8;
	}
  else
	{
		y=y1/8+1;
	}
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
		{
			OLED_WR_Byte(BMP[j++],1);
		}
	}
}


//初始化OLED
void OLED_Init(void)
{
	OLED_I2C_Init();
	Delay_ms(100);
	OLED_WR_Byte(0xAE,0);//--display off
	OLED_WR_Byte(0x00,0);//---set low column address
	OLED_WR_Byte(0x10,0);//---set high column address
	OLED_WR_Byte(0x40,0);//--set start line address
	OLED_WR_Byte(0xB0,0);//--set page address
	OLED_WR_Byte(0x81,0); // contract control
	OLED_WR_Byte(0xFF,0);//--128
	OLED_WR_Byte(0xA1,0);//set segment remap
	OLED_WR_Byte(0xA6,0);//--normal / reverse
	OLED_WR_Byte(0xA8,0);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,0);//--1/32 duty
	OLED_WR_Byte(0xC8,0);//Com scan direction
	OLED_WR_Byte(0xD3,0);//-set display offset
	OLED_WR_Byte(0x00,0);//
	
	OLED_WR_Byte(0xD5,0);//set osc division
	OLED_WR_Byte(0x80,0);//
	
	OLED_WR_Byte(0xD8,0);//set area color mode off
	OLED_WR_Byte(0x05,0);//
	
	OLED_WR_Byte(0xD9,0);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,0);//
	
	OLED_WR_Byte(0xDA,0);//set com pin configuartion
	OLED_WR_Byte(0x12,0);//
	
	OLED_WR_Byte(0xDB,0);//set Vcomh
	OLED_WR_Byte(0x30,0);//
	
	OLED_WR_Byte(0x8D,0);//set charge pump enable
	OLED_WR_Byte(0x14,0);//
	
	OLED_WR_Byte(0xAF,0);//--turn on oled panel
	
	OLED_Clear();
}


/* 开机动画 */
void Boot_Animation(void)
{
	static u8 x=0,y=0;
	for(x = 63;x>=18;x--)
	{
		OLED_DrawPoint(108-0.7*x,x,1);	//画斜线 斜率≈√3/3
		OLED_DrawPoint(17 +0.7*x,x,1);
		y = 64-x;
		OLED_DrawPoint(64-0.7*y,y,1);
		OLED_DrawPoint(64+0.7*y,y,1);
		OLED_Refresh_Gram();						//更新显示到OLED
	}
	for(x = 30;x <= 94;x++)
	{
		OLED_DrawPoint(125-x,47,1);
		OLED_DrawPoint(x,18,1);
		OLED_Refresh_Gram();						//更新显示到OLED
	}
}


/*
连线函数
入口参数：
x1：起点的x坐标；
y1：起点的y坐标；
x2：终点的x坐标；
y2：终点的y坐标；
*/
void OLED_DrawLine(unsigned int x1, unsigned int y1, unsigned int x2,unsigned int y2)
{
	unsigned int t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1;	//计算坐标增量
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1;	//设置单步方向
	else if(delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;//水平线
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )//画线输出
	{
		OLED_DrawPoint(uRow,uCol,1);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

void OLED_ShowFloat(u8 x,u8 y,float num,u8 len,u8 size2)
{
	int zheng=(int)num;
	int count = 0;
	if(zheng == 0) count = 1;
	zheng = zheng < 0 ? -zheng:zheng;
	while(zheng > 0)
	{
		zheng /= 10;
		count++;
	}
	if(num < 0)
	{
		OLED_ShowString(x-size2/2 ,y, "-", size2);
	}
	if(len > count+1)
	{
		OLED_ShowNum(x,y,num, count, size2, 1);
		OLED_ShowString(x+count*size2/2,y,".", size2);
		OLED_ShowNum(x+(count+1)*size2/2,y,num*oled_pow(10, len-count), len-count, size2, 0);
	}
	if(len == count)
	{
		OLED_ShowNum(x,y,(int)num, count, size2, 1);
	}
	if(len < count)
	{
		OLED_ShowString(x ,y, "errro", size2);
	}
}
