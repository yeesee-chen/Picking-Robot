#ifndef MYSERVO_H_
#define MYSERVO_H_

#include "HeaderFiles.h"

extern uint16_t Pulse0_Real;
extern uint16_t Pulse1_Real;
extern uint16_t Pulse2_Real;
extern uint16_t Pulse3_Real;
extern uint16_t Pulse0_Tar;
extern uint16_t Pulse1_Tar;
extern uint16_t Pulse2_Tar;
extern uint16_t Pulse3_Tar;

double TurnAngle(double theta);
void GetPulse(double X, double Y, double Theta);
void ArmInit(void);
void ZhuaZi_close(void);
void ZhuaZi_open(void);
void Put_Down(void);
void Looking(int pul);

#endif


