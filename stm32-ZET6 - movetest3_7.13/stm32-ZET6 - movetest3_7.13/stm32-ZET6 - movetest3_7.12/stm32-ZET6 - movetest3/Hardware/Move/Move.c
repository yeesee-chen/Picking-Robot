#include "Move.h"
#include "Bujin.h"
#include "OLED.h"

#define half_wide_size  0.2
#define wheel_radius    0.5

struct move speed = {0, 0, 0};
struct move angle_speed = {0, 0, 0};

float v[2] = {0}; //0Ϊ���֣�1Ϊ����
                  //��ַ1,4Ϊ���֣�2,3Ϊ����

/**
* @brief	 4�ֲ���
* @param	 Velocity С�������ٶ�
* @param	 Palstance С����ת���ٶȣ�����0��ת��С��0��ת
* @retval  None
*/
void DiffX4_Wheel_Speed_Model_Config(float Velocity, float Palstance)
{ 
  Palstance = -Palstance;

  v[0] = Velocity+Palstance*half_wide_size;
  v[1] = -(Velocity-Palstance*half_wide_size);

    if(v[0] > 0) //������ת
  {
    Emm_V5_Vel_Control(1, 1, v[0]*10, 255, 1);
    Emm_V5_Vel_Control(4, 1, v[0]*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }
  else         //���ַ�ת
  {
    Emm_V5_Vel_Control(1, 0, (-v[0])*10, 255, 1);
    Emm_V5_Vel_Control(4, 0, (-v[0])*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }

  if(v[1] > 0) //������ת
  {
    Emm_V5_Vel_Control(2, 1, v[1]*10, 255, 1);
    Emm_V5_Vel_Control(3, 1, v[1]*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }
  else         //���ַ�ת
  {
    Emm_V5_Vel_Control(2, 0, (-v[1])*10, 255, 1);
    Emm_V5_Vel_Control(3, 0, (-v[1])*10, 255, 1);
    Emm_V5_Synchronous_motion(0);
  }

}

/**
* @brief	 ԭ����ת
* @param	 dir С����ת���򣬴���0��ת��С��0��ת
* @param	 angle С����ת���ٶ�
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
	/***** �ٶ��޷� *****/
    if(fabs(speed.real - speed.tar) > 1e-4)//if(speed.real - speed.tar >0.01 | speed.real - speed.tar <-0.01)//
    {
		//����޷�
		if(speed.diff < -1.2)	speed.diff = -1.2;
		//����޷�
		if(speed.diff > 1.2)	speed.diff = 1.2;
		//��С�޷�
		if(fabs(speed.diff) < 0.1)
		{
			if(speed.diff > 0)	speed.diff = 0.1;
			if(speed.diff < 0)	speed.diff = -0.1;
		}
		//�ٶȸ�ֵ
        speed.real += speed.diff;
		//��������
		if(fabs(speed.real - speed.tar) < 1.2)	speed.real = speed.tar;
    }
	
	/***** ���ٶ��޷� *****/
    if(fabs(angle_speed.real - angle_speed.tar) > 1e-4)//if(angle_speed.real - angle_speed.tar >0.01 | angle_speed.real - angle_speed.tar <-0.01)//
    {
		//����޷�
		if(angle_speed.diff < -2)	angle_speed.diff = -2;
		//����޷�
		if(angle_speed.diff > 2)	angle_speed.diff = 2;
		//��С�޷�
		if(fabs(angle_speed.diff) < 0.1)
		{
			if(angle_speed.diff > 0)	angle_speed.diff = 0.1;
			if(angle_speed.diff < 0)	angle_speed.diff = -0.1;
		}
		//���ٶȸ�ֵ
        angle_speed.real += angle_speed.diff;
		//��������
		if(fabs(angle_speed.real - angle_speed.tar) < 2)	angle_speed.real = angle_speed.tar;
    }
    
    //�ٶ��޷�
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
	//���ֲ��ٸ�ֵ
	if(Last_Speed != speed.real | Last_AngleSpeed != angle_speed.real)
	{
		Last_Speed = speed.real;
		Last_AngleSpeed = angle_speed.real;
		DiffX4_Wheel_Speed_Model_Config(speed.real, angle_speed.real);

	}
}

