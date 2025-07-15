#include "MyServo.h"
#include "Serial.h"
#include "Servo.h"

#define L1	15.3	//大臂长度
#define L2	16.1	//小臂长度
#define L3	15	//爪子长度
#define L4	0		//爪子偏移
#define Pai	3.1415926

double theta0, theta1, theta2, theta3;

uint16_t Pulse0_Real = 1500;
uint16_t Pulse1_Real = 950;
uint16_t Pulse2_Real = 1650;
uint16_t Pulse3_Real = 1280;
uint16_t Pulse4_Real = 1700;

uint16_t Pulse0_Tar;
uint16_t Pulse1_Tar;
uint16_t Pulse2_Tar;
uint16_t Pulse3_Tar;


/* 弧度 -> 角度 */
double TurnAngle(double theta)
{
	return theta*180.0/Pai;
}

/* 机械臂逆解算,theta>0顺时针 */
void GetPulse(double X, double Y, double Theta)
{	
	//爪子偏移计算
	X = X-L3;
	//爪子倾斜修正
	Y = Y+3;
	X = X-1;
	
	/* 运动学解算 */
	double fai;
	double acos_temp = (X*X+Y*Y-L1*L1-L2*L2)/(2.0*L1*L2);
	theta2 = acos(acos_temp);
	fai = acos( (L2*L2 - (X*X+Y*Y) - L1*L1)/( (-2)*L1*sqrt(X*X+Y*Y) ) );
	
	Serial5_Printf("acos:%lf\r\n",acos_temp);	
	Serial5_Printf("theta2:%lf\r\n",theta2);
	Serial5_Printf("acos:%lf\r\n",(L2*L2 - (X*X+Y*Y) - L1*L1)/( (-2)*L1*sqrt(X*X+Y*Y)));
	Serial5_Printf("fai:%lf\r\n",fai);
	
//	if(Y>=0)
//	{
		theta1 = atan2(Y,X) + fai;
//	}else
//	{
//		theta1 = -atan2(Y,X) + fai;
//	}
	theta3 = theta1 - theta2;
	
	Serial5_Printf("atan:%lf\r\n",atan2(Y,X));
	Serial5_Printf("theta1:%lf\r\n",theta1);
	Serial5_Printf("theta3:%lf\r\n",theta3);
		
	/* 弧度转角度 */
	double Angle0 = Theta;
	double Angle1 = TurnAngle(theta1);
	double Angle2 = TurnAngle(theta2);
	double Angle3 = TurnAngle(theta3);;
	
	Serial5_Printf("Angle1:%lf\r\n",Angle1);
	Serial5_Printf("Angle2:%lf\r\n",Angle2);
	Serial5_Printf("Angle3:%lf\r\n",Angle3);
	

	
	
	/* 角度转脉冲 */
	Pulse0_Tar = 1500-Angle0*1000/180;
	
	/**角度1**/
//	Pulse1_Tar = Get_PWM(Angle1);
	Pulse1_Tar = 1465+(90-Angle1)*2000/180;
	
	/**角度2**/
	Pulse2_Tar = 535+(Angle2+30)*(1460-535)/90.0;
//	/**角度3**/

	Pulse3_Tar = 1500-Angle3*2000/270;		//爪子



//	Serial5_Printf("Pulse3_Tar:%d\r\n",Pulse3_Tar);

	//脉冲保护
	/*if(Pulse2_Tar >2400)	Pulse2_Tar = 2400, Serial5_SendByte(0xA1);
	if(Pulse2_Tar <600)		Pulse2_Tar = 600 , Serial5_SendByte(0xA1);
	if(Pulse3_Tar <800)		Pulse3_Tar = 800 , Serial5_SendByte(0xA1);*/
}

/* 机械臂初状态 */
void ArmInit(void)
{
	moveServo(0,1500,1500);
	moveServo(1,950,1500);
	moveServo(2,1650,1500);
	moveServo(3,1280,1500);
	Pulse0_Real = 1500;
	Pulse1_Real = 950;
	Pulse2_Real = 1650;
	Pulse3_Real = 1280;
}

/* 爪子关闭 */
void ZhuaZi_close(void)
{
	moveServo(4,2390,500);
}

/* 爪子打开 */
void ZhuaZi_open(void)
{
	moveServo(4,1700,500);
}

/* 爪子闭合 */
void Put_Down(void)
{
	moveServo(4,2390,500);
}

/*观测位*/
void A_left(void)
{
	moveServo(0,1000,1500);
	moveServo(1,2020,1500);
	moveServo(2,500,1500);
	moveServo(3,620,1500);
	Pulse0_Real = 1500;
	Pulse1_Real = 2020;
	Pulse2_Real = 500;
	Pulse3_Real = 620;
}
void A_right(void)
{
	moveServo(0,1985,1500);
	moveServo(1,2020,1500);
	moveServo(2,500,1500);
	moveServo(3,620,1500);
	Pulse0_Real = 1985;
	Pulse1_Real = 2020;
	Pulse2_Real = 500;
	Pulse3_Real = 620;
}
void B_left(void)
{
	moveServo(0,950,1500);
	moveServo(1,630,1500);
	moveServo(2,1185,1500);
	moveServo(3,610,1500);
	Pulse0_Real = 950;
	Pulse1_Real = 630;
	Pulse2_Real = 1185;
	Pulse3_Real = 610;
}
void B_right(void)
{
	moveServo(0,2000,1500);
	moveServo(1,750,1500);
	moveServo(2,1225,1500);
	moveServo(3,670,1500);
	Pulse0_Real = 2000;
	Pulse1_Real = 750;
	Pulse2_Real = 1225;
	Pulse3_Real = 670;

}

/**
A和C区共用观测位。
A左为1，A右2；
B左侧树左侧为3，B左侧树右为4；B右侧树左侧为5，B右侧树右为6
**/
void Looking( int pul) {
    switch(pul)
	{
		case 0:{
			ArmInit();
			break;
		}
		case 1:{
			A_left();
			break;
		}
		case 2:{
			A_right();
			break;
		}
		case 3:{
			B_left();
			break;
		}
		case 4:{
			B_right();
			break;
		}
		default:
		break;
	}		
}	
