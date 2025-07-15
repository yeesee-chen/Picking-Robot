#include "Voice.h"

char num1[] = {"/00001"};
char num2[] = {"/00002"};
char num3[] = {"/00003"};
char num4[] = {"/00004"};
char num5[] = {"/00005"};
char num6[] = {"/00006"};
char num7[] = {"/00007"};
char num8[] = {"/00008"};
char num9[] = {"/00009"};
char num10[] ={"/00010"};
char Apple[] ={"/00011"};
char Pear[]  ={"/00012"};
char Pumpkin[]={"/00013"}; 
char Pepper[]= {"/00014"}; 
char Tomato[]= {"/00015"};
char Onion[]={"/00016"};
char Teaminfor[]={"/00017"};
char Mature[] = {"/00018"};
char Inmature[] ={"/00019"};
char ErWeiMa[] ={"/00020"};
char Music[] = {"/00099"};

//const uint8_t quantity;
//char* Fruit[] ={Apple,Pear,Pumpkin,Pepper,Tomato,Eggplant};
char* numArray[] = {NULL, num1, num2, num3, num4, num5, num6, num7, num8, num9, num10};
void playFruitSound( int quantity,char* Fruitname) {
    if (quantity >= 1 && quantity <= 10) {
        JQ8x00_RandomPathPlay(JQ8X00_FLASH,Fruitname);
		Serial5_Printf("apple");
        Delay_ms(1000);
        JQ8x00_RandomPathPlay(JQ8X00_FLASH,numArray[quantity]);
		Serial5_Printf("111");
        Delay_ms(1500);
    }	
}	
	
/**************************串口控制************************************/

/************************************************************************
功能描述：起始码-指令类型-数据长度-校验和
入口参数： 	MODE：模式
返 回 值： none
其他说明： 无数数据传入
**************************************************************************/
void  JQ8x00_Command(UartCommand Command)
{
	uint8_t Buffer[4] ={0xaa};

    Buffer[1] = Command;            //指令类型			
    Buffer[2] = 0x00;           //数据长度
    Buffer[3] = Buffer[0] +  Buffer[1] +  Buffer[2];            //校验和

	JQ8x00_UART(Buffer,4);
}

/************************************************************************
功能描述：起始码-指令类型-数据长度-数据-校验和
入口参数： 	*DAT：字符串指针,Len字符串长度
返 回 值： none
其他说明： 
**************************************************************************/
void  JQ8x00_Command_Data(UartCommandData Command,uint8_t DATA)
{
	uint8_t Buffer[6] ={0xaa};
    uint8_t DataLen = 0;
    Buffer[1] = Command;       //指令类型
    if((Command != AppointTrack) && (Command != SetCycleCount) && (Command != SelectTrackNoPlay) && (Command != AppointTimeBack) && (Command != AppointTimeFast))        //只含一个数据指令    
    {
        Buffer[2] = 1;			//数据长度
        Buffer[3] = DATA;       //数据
        Buffer[4] = Buffer[0] +  Buffer[1] +  Buffer[2] + Buffer[3];            //校验和
        DataLen = 5;
    }
    else                                                                                        //含两个数据指令 
    {
        Buffer[2] = 2;			//数据长度
        Buffer[3] = DATA/256;       //数据
        Buffer[4] = DATA%256;       //数据
        Buffer[5] = Buffer[0] +  Buffer[1] +  Buffer[2] + Buffer[3] + Buffer[4];  
        DataLen = 6;
    }
    
	JQ8x00_UART(Buffer,DataLen);
}

/************************************************************************
功能描述：播放任意路径下的的音频文件
入口参数：JQ8X00_Symbol:系统盘符，*DATA:需要播放的音频文件路径
返 回 值： none
其他说明：播放说明例 如 ：/广告/小米手机.mp3,可以按如下格式
        /广告* /小米*???,“广告*”代表前两个字为“广告的文件夹”，*为通配符
        注意格式，下一个目录符号前要加*，如/测试1* /测试2* /00002*???
        JQ_8x00_RandomPathPlay(JQ8X00_FLASH,"广告* /小米")
        播放FLASH根目录下文件名为00001.mp3的音频，则传JQ_8x00_RandomPathPlay(JQ8X00_FLASH,"/00001");
**************************************************************************/
void JQ8x00_RandomPathPlay(JQ8X00_Symbol symbol,char *DATA)
{
    uint8_t Buffer[52] ={0xaa,0x08};
    uint8_t i,j;
    Buffer[2] = 11;       //长度,1为盘符，4为*？？？strlen(DATA)
    Buffer[3] = symbol;        //盘符
    i = 4;
		for(int k=0; k<6; k++)
		{
			Buffer[i++] = DATA[k];
		}
//    while(*DATA)
//    {
//       Buffer[i++] =  *DATA;
//       DATA++;
//    }
    Buffer[i++] = '*';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    for(j=0;j<i;j++)
    {
        Buffer[i] = Buffer[i] + Buffer[j];      //计算校验合
    }
    i++;
    UART3_SendCode(Buffer, i);
}

/************************************************************************
功能描述：插播任意路径下的的音频文件
**************************************************************************/
void JQ8x00_RandomPathPlay_Inserch(JQ8X00_Symbol symbol,char *DATA)
{
    uint8_t Buffer[52] ={0xaa,0x17};
    uint8_t i,j;
    Buffer[2] = 11;       //长度,1为盘符，4为*？？？strlen(DATA)
    Buffer[3] = symbol;        //盘符
    i = 4;
		for(int k=0; k<6; k++)
		{
			Buffer[i++] = DATA[k];
		}
//    while(*DATA)
//    {
//       Buffer[i++] =  *DATA;
//       DATA++;
//    }
    Buffer[i++] = '*';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    for(j=0;j<i;j++)
    {
        Buffer[i] = Buffer[i] + Buffer[j];      //计算校验合
    }
    i++;
    UART3_SendCode(Buffer, i);
}

/************************************************************************
功能描述：音量(0~30)
**************************************************************************/
void JQ8x00_Volumn(uint8_t vol)
{
	JQ8x00_Command_Data(0x13,vol);
//	uint8_t Buffer[5]={0xAA,0x13,0x01};
//	Buffer[3] = vol;
//	Buffer[4] = Buffer[0]+Buffer[1]+Buffer[2]+Buffer[3];
//	Serial3_SendArray(Buffer,5);
}

/************************************************************************
功能描述：结束播放
**************************************************************************/
void JQ8x00_Over(void)
{
	uint8_t Buffer[4]={0xAA,0x10,0x00,0xBA};
	Serial3_SendArray(Buffer,4);
}

/************************************************************************
功能描述：暂停
**************************************************************************/
void JQ8x00_Stop(void)
{
	uint8_t Buffer[4]={0xAA,0x03,0x00,0xAD};
	Serial3_SendArray(Buffer,4);
}

/************************************************************************
功能描述：停止
**************************************************************************/
void JQ8x00_Hold(void)
{
	uint8_t Buffer[4]={0xAA,0x04,0x00,0xAE};
	Serial3_SendArray(Buffer,4);
}

/************************************************************************
功能描述：单曲循环
**************************************************************************/
void JQ8x00_Loop(void)
{
	uint8_t Buffer[5]={0xAA,0x18,0x01,0x02,0xC5};
	Serial3_SendArray(Buffer,5);
}

/************************************************************************
功能描述：循环次数2次
**************************************************************************/
void JQ8x00_Loop_times2(void)
{
	uint8_t Buffer[6]={0xAA,0x19,0x02,0x00,0x02,0xC7};
	Serial3_SendArray(Buffer,6);
}

void Voice_Num(int Num)
{
	switch(Num){
		case 0:{
			JQ8x00_Over();
			break;
		}
		case 1:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num1);
			break;
		}
		case 2:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num2);
			break;
		}
		case 3:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num3);
			break;
		}
		case 4:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num5);
			break;
		}
		case 5:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num5);
			break;
		}
		case 6:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num6);
			break;
		}
		case 7:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num7);
			break;
		}
		case 8:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num8);
			break;
		}
		case 9:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num9);
			break;
		}
		case 10:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, num10);
			break;
		}
		case 11:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Apple);
			break;
		}
		case 12:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pear);
			break;
		}
		case 13:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pumpkin);
			break;
		}
		case 14:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Pepper);
			break;
		}
		case 15:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Tomato);
			break;
		}
		case 16:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Onion);
			break;
		}
		case 17:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Teaminfor);
			break;
		}
		case 18:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Mature);
			break;
		}
		case 19:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, Inmature);
			break;
		}	
		case 20:{
			JQ8x00_RandomPathPlay_Inserch(JQ8X00_FLASH, ErWeiMa);
			break;
		}
		case 99:{
			JQ8x00_Loop();
			JQ8x00_Loop_times2();
			JQ8x00_RandomPathPlay(JQ8X00_FLASH,Music);
			break;
		}
		default:
			break;
	}
}


