#include "QR.h"
#include "Serial.h"
#include "Voice.h"
/**********
�ı�ģʽGBK

**********/
//ƻ��[C6 BB B9 FB]
//����[C0 E6 D7 D3]
//�Ϲ�[C4 CF B9 CF]
//����[C0 B1 BD B7]
//������[CE F7 BA EC CA C1]
//����[C7 D1 D7 D3]

char QR[8][8];

void QR_Star(void)
{
	Serial4_SendString("S_CMD_020E");//����ģʽ
}

void QR_Stop(void)
{
	Serial4_SendString("S_CMD_020D");//����ģʽ
}

void QR_DataProcess(void)
{
	
}