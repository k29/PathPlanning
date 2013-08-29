#include "AcYut.h"


void AcYut::initialize()
{
	left_leg->setSpeed(50);
	right_leg->setSpeed(50);
	left_hand->setSpeed(50);
	right_hand->setSpeed(50);
	comm->syncFlush();
	left_leg->move(legHeight,0,0,0);
	right_leg->move(legHeight,0,0,0);
	left_hand->init();
	right_hand->init();
	comm->syncFlush();
	sleep(3);
	left_leg->setSpeed(0);
	right_leg->setSpeed(0);
	left_hand->setSpeed(0);
	right_hand->setSpeed(0);
	comm->syncFlush();
}

AcYut::AcYut(Communication* comm)
{
	this->comm = comm;
	// offsets[0] = 0;
	// offsets[1] = 0;
	// offsets[2] = 0;
	// offsets[3] = -0;
	// offsets[4] = 0;
	// offsets[5] = 10;
	// offsets[6] = 280;

	// offsets[20] = 0;
	// offsets[21] = 0;
	// offsets[22] = 0;
	// offsets[23] = -0;
	// offsets[24] = 0;
	// offsets[25] = -10;
	// offsets[26] = -256;



	offsets[0] = 0;
	offsets[1] = 0;
	offsets[2] = 16;
	offsets[3] = -16;
	offsets[4] = 0;
	offsets[5] = -40;
	offsets[6] = 280;

	offsets[20] = 10+10+10;
	offsets[21] = 0;
	offsets[22] = 16;
	offsets[23] = -16;
	offsets[24] = 0;
	offsets[25] = 0;
	offsets[26] = -256;

	// offsets[0] = 0;
	// offsets[1] = 0;
	// offsets[2] = 64;
	// offsets[3] = 48;
	// offsets[4] = 0;
	// offsets[5] = 0;
	// offsets[6] = 280;

	// offsets[20] = -0;
	// offsets[21] = 0;
	// offsets[22] = 64;
	// offsets[23] = 64;
	// offsets[24] = 0;
	// offsets[25] = 0;
	// offsets[26] = -256;

	// offsets[0] = 0;
	// offsets[1] = 0;
	// offsets[2] = 0;
	// offsets[3] = 0;
	// offsets[4] = 0;
	// offsets[5] = 0;
	// offsets[6] = 300;


	// offsets[20] = -0;
	// offsets[21] = 0;
	// offsets[22] = 0;
	// offsets[23] = 0;
	// offsets[24] = 0;
	// offsets[25] = 0;
	// offsets[26] = -256;


	this->left_leg = new Leg(LEFT,comm,&(offsets[0]));
	this->right_leg = new Leg(RIGHT,comm,&(offsets[20]));
	this->left_hand=new Hand(LEFT,comm);
	this->right_hand=new Hand(RIGHT,comm);
	leg[0] = left_leg;
	leg[1] = right_leg;
	// this->pingBot();
	initialize();
}

int AcYut::updateBot()
{
	comm->syncFlush();
}

int* AcYut::pingBot()
{
	left_leg->pingLeg();
	right_leg->pingLeg();
	left_hand->pingHand();
	right_hand->pingHand();
}

int AcYut::writeSensedCurrent(FILE *&fp)
{
	double *rightleg,*leftleg;
	rightleg=right_leg->getSensedCurrentAmps();
	leftleg=left_leg->getSensedCurrentAmps();
	
	char time_buf[21];
	time_t now;
	time(&now);
	strftime(time_buf, 21, "%Y-%m-%dT%H:%S:%MZ", gmtime(&now));
	
	for(int i=0;i<MOTOR_COUNT;i++)
	{
		fprintf(fp,"\nLeft %d %lf\t",i,leftleg[i]);
	}
	for(int i=0;i<MOTOR_COUNT;i++)
	{
		fprintf(fp,"\nRight %d %lf\t",i,rightleg[i]);
	}
	fprintf(fp,"\n");
	delete rightleg;
	delete leftleg;
}
