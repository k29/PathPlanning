
#define MINTHETA 15.0//degrees
#define STEPLEN 150.0//mm
#define WHEELDIAmm 100.0 //mm
#define WHEELDIST 240.0 //mm
#define tbFORWARD 0
#define tbBACKWARD 1
#define tbLEFT 3
#define tbRIGHT 2
#define tbSTOP 0,0
#define MSPEED 440.0
void callWalk(float walkr,float walktheta,int override=0)
{
	//walkr--distance to walk in mm
	//walktheta-- angle to move in degrees
	//rotate
	double delta;
	double arclen;
	printf("[walkthread]   walk cmd received angle=%f",enAngle);
	if(override==1)//overrides min values//1=encircle override//
	{
		if(walktheta<0)
		{
			walktheta*=-1;
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			// pthread_mutex_lock(&mutex_walkstr);
			// walkstr.mm.theta=-walktheta;
			// pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(walktheta>0)
		{
			// pthread_mutex_lock(&mutex_walkstr);
			// walkstr.mm.theta=walktheta;
			// pthread_mutex_unlock(&mutex_walkstr);
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
		}
		delta=(walkr/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
		testBotWalk1(tbFORWARD,512);
		usleep((int)(delta/0.6));
		testBotWalk1(tbSTOP);
		// pthread_mutex_lock(&mutex_walkstr);
		// walkstr.mm.r=walkr;
		// walkstr.mm.updated=1;
		// pthread_mutex_unlock(&mutex_walkstr);
		return ;
	}

	if(walktheta<0)
	{//turn left for DELTA usecs
		//s=r.theta
		//so to move theta degrees, s=(theta/180)*2*3.1415*WHEELDIST
		/* though this formula is wrong(by some factor, maybe 2^(some n)), adjustments to achieve accurate turning have been made in MSPEED for this error,
		 any change in the formula will require recalibration of mspeed for angle rotation!!!*/
		walktheta*=-1;
		if(walktheta>MINTHETA)
		{
			arclen=(MINTHETA/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=-MINTHETA;
			walkstr.mm.theta2=0;
			pthread_mutex_unlock(&mutex_walkstr);

		}
		else
		{
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=-walktheta;//IMP:Minus sign over here is because of multipication by -1 in the line above before "if"
			walkstr.mm.theta2=0;
			pthread_mutex_unlock(&mutex_walkstr);
		}

	}
	else if(walktheta>0)
	{//turn right for DELTA usecs
		if(walktheta>MINTHETA)
		{
			arclen=(MINTHETA/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep((int)(delta/0.6));
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=MINTHETA;
			walkstr.mm.theta2=0;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else
		{
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=walktheta;
			walkstr.mm.theta2=0;
			pthread_mutex_unlock(&mutex_walkstr);
		}

	}
	if((walktheta>=0&&walktheta<=5)||(walktheta<=0&&walktheta>=-5))
	{
		if(walkr>STEPLEN)
		{
			//if speed is 1023, angular speed is--476deg/sec for RX28!!!!!!!!!!
			//speed is 512 so angular speed is MSPEEDdeg/sec
			// check the above facts...
			//therefore (pi D)*(MSPEED/360) mm in one second ie speed=MSPEED*3.1415*WHEELDIAmm/360 mm per sec
			//therefore DELTA=(walkr/3.1415*WHEELDIAmm*MSPEED/360)
			delta=(STEPLEN/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbFORWARD,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=STEPLEN;
			//walkstr.mm.updated=1;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else
		{
			delta=(walkr/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbFORWARD,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=walkr;
			//walkstr.mm.updated=1;
			pthread_mutex_unlock(&mutex_walkstr);
		}
	}
	pthread_mutex_lock(&mutex_walkstr);
	walkstr.mm.updated=1;
	pthread_mutex_unlock(&mutex_walkstr);
}


void* walk_thread(void*)
{
	WalkCommands bodycommandlocal;
	while(1)
	{
		walkstructure localwalkstr;

		pthread_mutex_lock(&mutex_walkflag);
		Flag_moving_local=Flag_moving;
		pthread_mutex_unlock(&mutex_walkflag);

		pthread_mutex_lock(&mutex_bodycommand);
		bodycommandlocal=bodycommand;
		pthread_mutex_unlock(&mutex_bodycommand);

		pthread_mutex_lock(&mutex_walkstr);
		localwalkstr=walkstr;
		pthread_mutex_unlock(&mutex_walkstr);

		if(Flag_moving_local==1)//if walkflag is set ie. if behav is telling walkthread to walk
		{
			if(bodycommandlocal==WALK)
			{
				printf("[walkthread]   callWalk sent; r=%f theta=%f\n",localwalkstr.r,localwalkstr.theta);
				callWalk(localwalkstr.r,localwalkstr.theta);
				// pthread_mutex_lock(&mutex_walkflag);
				// Flag_moving=0;
				// pthread_mutex_unlock(&mutex_walkflag);
				stopWalk();
			}
			if(bodycommandlocal==ENCIRCLE)
			{
				printf("[walkthread]  encircling cmd sent; angle=%f\n",localwalkstr.encircleAngle);
				callEncircle(localwalkstr.encircleAngle);
				// pthread_mutex_lock(&mutex_walkflag);
				// Flag_moving=0;
				// pthread_mutex_unlock(&mutex_walkflag);
				stopWalk();
			}
		}
		else
		{
			usleep(20000);
			continue;
		}
	}
}

#define ENCIRCLERADIUS 400 //mm
int callEncircle(float enAngle)
{
  	// Function to en"circle" for diff drive bot
	//THIS FUNCTION ASSUMES: Bot faces the center of the circle of radius ENCIRCLERADIUS
	//then it takes steps of 20 degrees to land on the next point of the circle
	//+right, -left
	//mm(r,base,180-base)
	printf("[walkthread]   encircling cmd received angle=%f\n",enAngle);
	float base=(180 - MINenAngle)/2;
	if(enAngle>MINenAngle||enAngle<-MINenAngle)
	{
		//assuming a triangle made by 2 radius vectors from center at MINenAngle deg with each other, base angles are (180-minenangle)/2 so first rotate "base" degrees then reverse 100 degrees
		if(enAngle>0)
			callWalk(0,-base,1);
		else if(enAngle<0)
			callWalk(0,base,1);
		callWalk(2*ENCIRCLERADIUS*cos(base*3.14159/180.0)/*cos 15 degrees*/ ,0,1);
		if(enAngle>0)
			callWalk(0,(180-base),1);
		else if(enAngle<0)
			callWalk(0,-(180-base),1);
		pthread_mutex_lock(&mutex_walkstr);
		walkstr.mm.r=2*ENCIRCLERADIUS*cos(base*3.14159/180.0);
		walkstr.mm.theta=base;
		walkstr.mm.theta2=180-base;
		walkstr.mm.updated=1;
		pthread_mutex_unlock(&mutex_walkstr);
		cvWaitKey();
	}
}
