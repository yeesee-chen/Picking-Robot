#include "QR.h"
#include "Serial.h"
#include "Voice.h"
/**********
文本模式GBK

**********/
//苹果[C6 BB B9 FB]
//梨子[C0 E6 D7 D3]
//南瓜[C4 CF B9 CF]
//辣椒[C0 B1 BD B7]
//西红柿[CE F7 BA EC CA C1]
//茄子[C7 D1 D7 D3]

char QR[8][8];

void QR_Star(void)
{
	Serial4_SendString("S_CMD_020E");//连续模式
}

void QR_Stop(void)
{
	Serial4_SendString("S_CMD_020D");//命令模式
}

void QR_DataProcess(void)
{
	
}