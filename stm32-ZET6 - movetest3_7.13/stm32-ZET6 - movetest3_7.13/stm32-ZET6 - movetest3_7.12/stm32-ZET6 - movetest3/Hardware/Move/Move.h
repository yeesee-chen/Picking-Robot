#ifndef MOVE_H
#define MOVE_H

#include "HeaderFiles.h"

/* ���ֲ��ٽṹ�� */
struct move{  
float tar;  //Ŀ���ٶ�
float real; //ʵ���ٶ�
float diff; //�ٶȲ�
};

extern struct move speed,angle_speed;

void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance);
void XuanZhuan(FlagStatus dir, uint8_t angle);
void Move_Updata(void);

#endif

