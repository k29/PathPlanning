#ifndef _LEG_H
#define _LEG_H
#include "motor.h"
#include "communication.h"
#include "commondefswalk.h"

#include <stdio.h>
#include <math.h>


class Leg
{

private:
	Communication *comm;
	Motor* m[MOTOR_COUNT];
	static const double l1=170.0;
	static const double l2=69.5;
  	static const double l3=170.0;
  	static const double delta=4096.0/360.0;
	static const double legHeight = 170.0+170.0+69.5+10;	

	float x;
	float y;
	float z;
	float theta;
	
	

public:
	int leg;
	Leg(int leg, Communication* comm, int* offsets);
	void setIDList(int* idList, int n);
	int move(double x, double y, double z, double t4);
	int setSpeed(int speed);
	int getLoad();
	int* pingLeg();
	int getMotorLoad(int motorID);
	int setMotorGainP(int p, int motorSelect);
	int pingMotor(int id);
	int ledonm(int id);
	int GP(int id);
	double*  getSensedCurrentAmps();
	
};
#endif
