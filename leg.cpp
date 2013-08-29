#include "leg.h"

#define deg2rad(x) (x)*(3.1415926535/180.0)
#define rad2deg(x) (x)*(180.0/3.1415926535)

Leg::Leg(int leg, Communication* comm, int *offsets)
{
	this->comm = comm;
	for (int i = 0; i < 7; ++i)
	{
		//printf("Offset %d %d\n",leg+i,offsets[i]);
		m[i] = new Motor(MX106, leg+i, this->comm,offsets[i]);
	}
	m[FOOT_PITCH]->slaveMode();
	m[HIP_PITCH]->slaveMode();
	x = legHeight;
	y = 0;
	z = 0;
	theta = 0;
	this->leg = leg;
	
	
}

void Leg::setIDList(int* idList, int n)
{
	for (int i = 0; i < n && i < 7; ++i)
	{
		m[i]->setIDSoft(idList[i]);
	}
}

int Leg::move(double x, double y, double z, double t4)
{
	//printf("%s X %lf Y%lf Z%lf\n",(leg==LEFT)?"LEFT":"RIGHT",x,y,z );
	/*if (this->leg == LEFT)
		z = z + 5.0;
	else if(this->leg == RIGHT)
		x = x - 5.0;
	*/
	double t1,t2,t3,t5;
	if((x*x+y*y+z*z)>((l1+l2+l3)*(l1+l2+l3)))
	{
		printf("Invalid position: x:%lf y:%lf z:%lf\n", x, y, z);
		return EXIT_FAILURE;
	}
	double correctionfactor=0.;//when y and l both are zero denominator becomes exactly zero for phi, avoid this using correctionfactor,
	double l=sqrt(x*x+z*z)-l2;
	if(y==0.&&l==0.)
    	correctionfactor=0.000001;
	double phi=asin(y/(sqrt(y*y+l*l+correctionfactor)));
	t3=rad2deg(asin((l*l+l3*l3-l1*l1+y*y)/(2*l3*(sqrt(y*y+l*l+correctionfactor))))-phi);
	t2=180.0-rad2deg(acos((y-l3*cos(deg2rad(t3)))/l1));
	t5=rad2deg(atan2(z,x));
	t1=-t5;

	//Angles calculated, now calculating motor values
	double pos[7];
	pos[1]=(1024+delta*t2);//1
	pos[2]=4096-pos[1];//2
	pos[4]=(1024+delta*t3);//4
	pos[3]=4096-pos[4];//3
	pos[5]=(2048-delta*t5);//5
	pos[0]=4096-pos[5];//0
	if(this->leg==RIGHT)
		pos[6]=(2048+t4*delta);//6    /TODO finddirection and correct the angle for HIP_YAW motor
	else if(this->leg==LEFT)
		pos[6]=(2048-t4*delta);
	//need to verify motor values it this point maybe? relying on ik

	for (int i = 0; i < 7; ++i)
	{
		//printf("MOTOR[%d] %d\n",leg+i,(int)pos[i]);
		m[i]->setGoalPositionSync(pos[i]);
	}
}

int Leg::setSpeed(int speed)
{
	for(int i=0; i< MOTOR_COUNT; i++)
	{
		m[i]->setMovingSpeedSync(speed);
	}

}

int Leg::getLoad()
{
	for (int i = 0; i < MOTOR_COUNT; ++i)
	{
		m[i]->getPresentLoad();
	}
}

int* Leg::pingLeg()
{
	for (int i = 0; i < MOTOR_COUNT; ++i)
	{
		m[i]->ping();
	}
}

int Leg::pingMotor(int id)
{
	m[id]->ping();
}	

int Leg::ledonm(int id)
{
	m[id]->ledOn();
}

int Leg::GP(int id)
{
	m[id]->getGoalPosition();
}

int Leg::getMotorLoad(int motor)
{
	m[motor]->getPresentLoad();
}

int Leg::setMotorGainP(int p, int motorSelect)
{
	for (int motor = 0; motor < MOTOR_COUNT; motor++)
	{
		if((motorSelect&(int)pow(2,motor)) >> motor)
			if(m[motor]->setGainPSync(p))
				return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}

double*  Leg::getSensedCurrentAmps()
{
	double *current=new double[MOTOR_COUNT];
	for(int motor=0;motor< MOTOR_COUNT; motor++)
	{
		current[motor]=m[motor]->getSensedCurrent();
	}
	return current;
}
