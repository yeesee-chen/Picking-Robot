#include "Move.h"
#include "Bujin.h"
#include "OLED.h"

#define half_wide_size  0.2
#define wheel_radius    0.5

struct move speed = {0, 0, 0};
struct move angle_speed = {0, 0, 0};

float v[2] = {0}; //0为左轮，1为右轮
                  //地址1,4为左轮，2,3为右轮

/**
* @brief	 4轮差速
* @param	 Velocity 小车整体速度
* @param	 Palstance 小车旋转角速度，大于0右转，小于0左转
* @retval  None
*/
void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance)
{ 
  Palstance = -Palstance;

  v[0] = Velocity+Palstance*half_wide_size;
  v[1] = -(Velocity-Palstance*half_wide_size);

    if(v[0] > 0) //左轮正转
  {
    Emm_V5_Vel_Control(1, 1, v[0]*10, 255, 1);
    Emm_V5_Vel_Control(4, 1, v[0]*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }
  else         //左轮反转
  {
    Emm_V5_Vel_Control(1, 0, (-v[0])*10, 255, 1);
    Emm_V5_Vel_Control(4, 0, (-v[0])*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }

  if(v[1] > 0) //右轮正转
  {
    Emm_V5_Vel_Control(2, 1, v[1]*10, 255, 1);
    Emm_V5_Vel_Control(3, 1, v[1]*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }
  else         //右轮反转
  {
    Emm_V5_Vel_Control(2, 0, (-v[1])*10, 255, 1);
    Emm_V5_Vel_Control(3, 0, (-v[1])*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }

}

/**
* @brief	 原地旋转
* @param	 dir 小车旋转方向，大于0右转，小于0左转
* @param	 angle 小车旋转角速度
* @retval  None
*/
void XuanZhuan(FlagStatus dir, uint8_t angle)
{
  if(dir == 0)
  {
    Emm_V5_Pos_Control(1, 0, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(2, 0, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(3, 0, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(4, 0, 20, 25, angle*36, 0, 1);
    Delay_ms(10);

    Emm_V5_Synchronous_motion(0);
  }
  else
  {
    Emm_V5_Pos_Control(1, 1, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(2, 1, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(3, 1, 20, 25, angle*36, 0, 1);
    Delay_ms(10);
    Emm_V5_Pos_Control(4, 1, 20, 25, angle*36, 0, 1);
    Delay_ms(10);

    Emm_V5_Synchronous_motion(0);
  }
}


void Move_Updata(void)
{
	static float Last_Speed=0;
	static float Last_AngleSpeed=0;
	/***** 速度限幅 *****/
    if(fabs(speed.real - speed.tar) > 1e-4)//if(speed.real - speed.tar >0.01 | speed.real - speed.tar <-0.01)//
    {
		//最低限幅
		if(speed.diff < -1.2)	speed.diff = -1.2;
		//最高限幅
		if(speed.diff > 1.2)	speed.diff = 1.2;
		//最小限幅
		if(fabs(speed.diff) < 0.1)
		{
			if(speed.diff > 0)	speed.diff = 0.1;
			if(speed.diff < 0)	speed.diff = -0.1;
		}
		//速度赋值
        speed.real += speed.diff;
		//极限消除
		if(fabs(speed.real - speed.tar) < 1.2)	speed.real = speed.tar;
    }
	
	/***** 角速度限幅 *****/
    if(fabs(angle_speed.real - angle_speed.tar) > 1e-4)//if(angle_speed.real - angle_speed.tar >0.01 | angle_speed.real - angle_speed.tar <-0.01)//
    {
		//最低限幅
		if(angle_speed.diff < -2)	angle_speed.diff = -2;
		//最高限幅
		if(angle_speed.diff > 2)	angle_speed.diff = 2;
		//最小限幅
		if(fabs(angle_speed.diff) < 0.1)
		{
			if(angle_speed.diff > 0)	angle_speed.diff = 0.1;
			if(angle_speed.diff < 0)	angle_speed.diff = -0.1;
		}
		//角速度赋值
        angle_speed.real += angle_speed.diff;
		//极限消除
		if(fabs(angle_speed.real - angle_speed.tar) < 2)	angle_speed.real = angle_speed.tar;
    }
    
    //速度限幅
	if(fabs(speed.real) > 12)
	{
		if(speed.real > 0)	speed.real = 12;
		if(speed.real < 0)	speed.real = -12;
	}
	if(fabs(angle_speed.real) > 24)
	{
		if(angle_speed.real > 0)	angle_speed.real = 24;
		if(angle_speed.real < 0)	angle_speed.real = -24;
	}
	//四轮差速赋值
	if(Last_Speed != speed.real | Last_AngleSpeed != angle_speed.real)
	{
		Last_Speed = speed.real;
		Last_AngleSpeed = angle_speed.real;
		DiffX4_Wheel_Speed_Model_Config(speed.real, angle_speed.real);

	}
}

