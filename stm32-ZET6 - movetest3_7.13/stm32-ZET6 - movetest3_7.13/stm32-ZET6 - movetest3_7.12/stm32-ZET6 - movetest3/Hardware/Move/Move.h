#ifndef MOVE_H
#define MOVE_H

#include "HeaderFiles.h"

/* 四轮差速结构体 */
struct move{  
float tar;  //目标速度
float real; //实际速度
float diff; //速度差
};

extern struct move speed,angle_speed;

void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance);
void XuanZhuan(FlagStatus dir, uint8_t angle);
void Move_Updata(void);

#endif

