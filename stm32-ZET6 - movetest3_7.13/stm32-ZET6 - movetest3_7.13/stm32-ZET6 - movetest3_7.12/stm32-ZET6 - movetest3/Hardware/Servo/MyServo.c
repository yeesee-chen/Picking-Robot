#include "MyServo.h"
#include "Serial.h"
#include "Servo.h"

#define L1	15.3	//��۳���
#define L2	16.1	//С�۳���
#define L3	15	//צ�ӳ���
#define L4	0		//צ��ƫ��
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


/* ���� -> �Ƕ� */
double TurnAngle(double theta)
{
	return theta*180.0/Pai;
}

/* ��е�������,theta>0˳ʱ�� */
void GetPulse(double X, double Y, double Theta)
{	
	//צ��ƫ�Ƽ���
	X = X-L3;
	//צ����б����
	Y = Y+3;
	X = X-1;
	
	/* �˶�ѧ���� */
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
		
	/* ����ת�Ƕ� */
	double Angle0 = Theta;
	double Angle1 = TurnAngle(theta1);
	double Angle2 = TurnAngle(theta2);
	double Angle3 = TurnAngle(theta3);;
	
	Serial5_Printf("Angle1:%lf\r\n",Angle1);
	Serial5_Printf("Angle2:%lf\r\n",Angle2);
	Serial5_Printf("Angle3:%lf\r\n",Angle3);
	

	
	
	/* �Ƕ�ת���� */
	Pulse0_Tar = 1500-Angle0*1000/180;
	
	/**�Ƕ�1**/
//	Pulse1_Tar = Get_PWM(Angle1);
	Pulse1_Tar = 1465+(90-Angle1)*2000/180;
	
	/**�Ƕ�2**/
	Pulse2_Tar = 535+(Angle2+30)*(1460-535)/90.0;
//	/**�Ƕ�3**/

	Pulse3_Tar = 1500-Angle3*2000/270;		//צ��



//	Serial5_Printf("Pulse3_Tar:%d\r\n",Pulse3_Tar);

	//���屣��
	/*if(Pulse2_Tar >2400)	Pulse2_Tar = 2400, Serial5_SendByte(0xA1);
	if(Pulse2_Tar <600)		Pulse2_Tar = 600 , Serial5_SendByte(0xA1);
	if(Pulse3_Tar <800)		Pulse3_Tar = 800 , Serial5_SendByte(0xA1);*/
}

/* ��е�۳�״̬ */
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

/* צ�ӹر� */
void ZhuaZi_close(void)
{
	moveServo(4,2390,500);
}

/* צ�Ӵ� */
void ZhuaZi_open(void)
{
	moveServo(4,1700,500);
}

/* צ�ӱպ� */
void Put_Down(void)
{
	moveServo(4,2390,500);
}

/*�۲�λ*/
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
A��C�����ù۲�λ��
A��Ϊ1��A��2��
B��������Ϊ3��B�������Ϊ4��B�Ҳ������Ϊ5��B�Ҳ�����Ϊ6
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
