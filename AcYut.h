#ifndef _ACYUT_H_
#define _ACYUT_H_

#include "commondefswalk.h"
#include "communication.h"
#include "leg.h"
#include "motor.h"
#include "hand.h"
#include <time.h>

#define ACYUT_MOTOR 31

class AcYut
{

private:
	int offsets[ACYUT_MOTOR];
	Communication *comm;

	static const double legHeight = 390;	

public:
	AcYut(Communication *comm);
	void initialize();
	Leg *right_leg, *left_leg;
	Leg* leg[2];
	Hand *right_hand, *left_hand;
	int updateBot();
	int* pingBot();
	int writeSensedCurrent(FILE *&fp);
};


#endif
