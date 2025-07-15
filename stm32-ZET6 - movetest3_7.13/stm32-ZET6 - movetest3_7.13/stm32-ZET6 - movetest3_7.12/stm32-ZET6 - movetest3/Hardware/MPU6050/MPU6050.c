#include "MPU6050.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050��I2C�ӻ���ַ


/**
  * ��    ����MPU6050�ȴ��¼�
  * ��    ����ͬI2C_CheckEvent
  * �� �� ֵ����
  */
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//������ʱ����ʱ��
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//ѭ���ȴ�ָ���¼�
	{
		Timeout --;										//�ȴ�ʱ������ֵ�Լ�
		if (Timeout == 0)								//�Լ���0�󣬵ȴ���ʱ
		{
			/*��ʱ�Ĵ�������룬������ӵ��˴�*/
			break;										//�����ȴ���������
		}
	}
}

/**
  * ��    ����MPU6050д�Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * ��    ����Data Ҫд��Ĵ��������ݣ���Χ��0x00~0xFF
  * �� �� ֵ����
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);										//Ӳ��I2C������ʼ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//�ȴ�EV6
	
	I2C_SendData(I2C2, RegAddress);											//Ӳ��I2C���ͼĴ�����ַ
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//�ȴ�EV8
	
	I2C_SendData(I2C2, Data);												//Ӳ��I2C��������
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//�ȴ�EV8_2
	
	I2C_GenerateSTOP(I2C2, ENABLE);											//Ӳ��I2C������ֹ����
}

/**
  * ��    ����MPU6050���Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * �� �� ֵ����ȡ�Ĵ��������ݣ���Χ��0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);										//Ӳ��I2C������ʼ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);	//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//�ȴ�EV6
	
	I2C_SendData(I2C2, RegAddress);											//Ӳ��I2C���ͼĴ�����ַ
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//�ȴ�EV8_2
	
	I2C_GenerateSTART(I2C2, ENABLE);										//Ӳ��I2C�����ظ���ʼ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//�ȴ�EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);		//Ӳ��I2C���ʹӻ���ַ������Ϊ����
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//�ȴ�EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);									//�ڽ������һ���ֽ�֮ǰ��ǰ��Ӧ��ʧ��
	I2C_GenerateSTOP(I2C2, ENABLE);											//�ڽ������һ���ֽ�֮ǰ��ǰ����ֹͣ����
	
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);				//�ȴ�EV7
	Data = I2C_ReceiveData(I2C2);											//�������ݼĴ���
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);									//��Ӧ��ָ�Ϊʹ�ܣ�Ϊ�˲�Ӱ��������ܲ����Ķ�ȡ���ֽڲ���
	
	return Data;
}

/**
  * ��    ����MPU6050��ʼ��
  * ��    ������
  * �� �� ֵ����
  */
void MPU6050_Init(void)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);		//����I2C2��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//����GPIOB��ʱ��
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//��PB10��PB11���ų�ʼ��Ϊ���ÿ�©���
	
	/*I2C��ʼ��*/
	I2C_InitTypeDef I2C_InitStructure;						//����ṹ�����
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;				//ģʽ��ѡ��ΪI2Cģʽ
	I2C_InitStructure.I2C_ClockSpeed = 50000;				//ʱ���ٶȣ�ѡ��Ϊ50KHz
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		//ʱ��ռ�ձȣ�ѡ��Tlow/Thigh = 2
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				//Ӧ��ѡ��ʹ��
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//Ӧ���ַ��ѡ��7λ���ӻ�ģʽ�²���Ч
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;				//�����ַ���ӻ�ģʽ�²���Ч
	I2C_Init(I2C2, &I2C_InitStructure);						//���ṹ���������I2C_Init������I2C2
	
	/*I2Cʹ��*/
	I2C_Cmd(I2C2, ENABLE);									//ʹ��I2C2����ʼ����
	
	/*MPU6050�Ĵ�����ʼ������Ҫ����MPU6050�ֲ�ļĴ����������ã��˴��������˲�����Ҫ�ļĴ���*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//��Դ����Ĵ���1��ȡ��˯��ģʽ��ѡ��ʱ��ԴΪX��������
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//��Դ����Ĵ���2������Ĭ��ֵ0���������������
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//�����ʷ�Ƶ�Ĵ��������ò�����
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//���üĴ���������DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//���������üĴ�����ѡ��������Ϊ��2000��/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//���ٶȼ����üĴ�����ѡ��������Ϊ��16g
}


/**
  * ��    ����MPU6050��ȡID��
  * ��    ������
  * �� �� ֵ��MPU6050��ID��
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//����WHO_AM_I�Ĵ�����ֵ
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
	*AccX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//��ȡ���ٶȼ�Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//��ȡ���ٶȼ�Y��ĵ�8λ����
	*AccY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//��ȡ���ٶȼ�Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//��ȡ���ٶȼ�Z��ĵ�8λ����
	*AccZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//��ȡ������X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//��ȡ������X��ĵ�8λ����
	*GyroX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//��ȡ������Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//��ȡ������Y��ĵ�8λ����
	*GyroY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
	*GyroZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
}

#define Acc_Gain 0.0001220f
#define Gyro_Gain 0.0609756f
#define Gyro_Gr 0.0010641f
#define G 9.80665f

short ax,ay,az;
short gx,gy,gz;

float ax_float, ay_float, az_float;
float gx_float, gy_float, gz_float;

static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void Prepare_Data(void)
{
	MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz);
	
	ax_float = (float)(ax * Acc_Gain * G);
	ay_float = (float)(ay * Acc_Gain * G);
	az_float = (float)(az * Acc_Gain * G);
	
	gx_float = (float)gx * Gyro_Gr;
	gy_float = (float)gy * Gyro_Gr;
	gz_float = (float)gz * Gyro_Gr;
}



#define Kp 6.00f
#define Ki 0.01f
#define halfT 0.1f	

float Yaw=0,Pitch=0,Roll=0;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;

void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az)
{
	uint8_t i;
	float vx,vy,vz;							
	float ex,ey,ez;							
	float norm;
	
 	float q0q0 = q0*q0;
 	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
 	float q1q2 = q1*q2;
 	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if(ax*ay*az == 0)
	return;
	
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	vx = 2*(q1q3 - q0q2);												
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;	

	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;  
	q3 = q3 * norm;

	Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
	Pitch = -asin(2.f * (q1q3 - q0q2))* 57.3f * 90 / 80.75;
	Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f * 90 / 81.45;
}

//���ú��ֱ�Ӷ�ȡYaw��Pitch��Roll;
void Imu_GetData(void)
{
	Prepare_Data();
	Imu_Update(gx_float,gy_float,gz_float,ax_float,ay_float,az_float);	
}

