#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <ftdi.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <fstream>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fstream>
using namespace std;

#include <pthread.h>
#include <string.h>
//path as it was on 21-9-2012(no initial orientation- only obs avoidance and final orientation) will only work on testbot

#define GOALKEEPER 0//keep value as 0 for normal bot

// #define GAMECONTROLLERACTIVE
#define SWITCHACTIVE
// #define WIFITRANSMISSION
#include "commondefs.h"
#include "headmotor.h"
#include "camcapture.h"
#include "camcontrol.h"
#include "featuredetection.h"
#include "localize.h"
#include "defines.h"
#include "path.hpp"

// #define WASD_CODE

// #ifndef TESTINGBOT
// #include "walk_thread.cpp"
// #endif

#ifdef SWITCHACTIVE
#include "acyut_gpio_func.cpp"
#endif

#ifdef GAMECONTROLLERACTIVE
#include "gamecontrollerfunc.h"
#include "gamecontrollerobj.h"
#endif


//TYPE DEFINITIONS
//Also see commondefs.h

enum WalkCommands {INIT,WALK,ENCIRCLE,GETUPFRONT,GETUPBACK,KICK,ORIENT};//behaviour will calculate r and theta from path and continue

typedef struct
{
	float r;
	float theta;
	float encircleAngle;
	char instr;
	Coords orientArray[100];
	int len_orientArray;
	MotionModel mm;
	WalkCommands command;
	bool isFresh;
} WalkStructure;


enum Goal{BLUEGOAL=0,YELLOWGOAL=1};

//OTHER DEFINITIONS
#define NO_OF_FRAMES_TO_WAIT_B4_KICKOFF 50
#define MINenAngle 15 //minimum angle for encircling

//For GOALKEEPER//values are set after testing//keep testing...keep updating... atleast after every major change(in IPcodes/motor/camera/acyutsystem)
#define GKFALLDISTTHRESHOLD 50//cm//the distance after which it is critical to save, so if velocity>didntmovevel then dive
#define GKMINFRAMES2GO 20//frames//minimum frames remaining for initiating dive, if the no of frames for ball to come close(ballR/velocity) is less than this, initiate dive
#define MINBALLANGLE4FALL 15//degrees//the angle above which dive will be towards right,below -(thisangle) it will be towards left
#define DIDNOTMOVEVELOCITY 1// cm/frame //velocity threshold,below this it will velocity, the ball is as good as still

//distances in cm

#define LM_PREVFACT 0.5
#define LM_WT 0.72165
#define B_PREVFACT 0.5
#define B_WT 1.0
#define B_WT2 1
#define TH 0.6
#define TL 0.03

#define STEP_WALKBEHAV 10
#define STEP_TURNBEHAV 7.0

//VARIABLE DECLARATIONS
//State currentstate=INITIAL;
BallReturns b;
GoalReturns g;
WalkCommands bodycommand;
Flags globalflags;
Goal mygoal=YELLOWGOAL;//0=blue,1=yellow
//int flag_encircle;
int Flag_moving=0;//0 if not walking
int Flag_moving_local;//for walkthread
int Flag_moving_mainlocal;//for main

pthread_mutex_t mutex_walkflag=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t mutex_encircleflag=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_walkstr=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t mutex_bodycommand=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t mutex_changewalkflag=PTHREAD_MUTEX_INITIALIZER;
WalkStructure walkstr;
//WalkStructure prevwalkstr;
CamCapture capture;
FeatureDetection fd(capture);
HeadMotor hdmtr;
CamControl camcont(capture);
Localize loc;
//float bwt,lmwt;
//int ballnotinframe;
MotionModel localmm;
double conf;
PathStructure pathstr;
Path pathobj;
//float prevballr,velocity;

//WALK RELATED
int global_CHANGEWALKFLAG;

#ifdef GAMECONTROLLERACTIVE
GameControllerAcyut GC;
#endif

#ifdef WIFITRANSMISSION
#include "wificommunication.cpp"
#endif


inline void stopWalk()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving=0;
	pthread_mutex_unlock(&mutex_walkflag);
}
//int walk3(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi) //walk charli

#ifndef TESTINGBOT
#define MINTHETA 15.0//degrees
#define STEPLEN 15//cm
#endif

#ifdef TESTINGBOT
#include "testbotfunctions.cpp"
#endif
#ifndef TESTINGBOT
WalkCommands localbodycommand;

// void read_walk_data()
// {
// 	pthread_mutex_lock(&mutex_walkflag);
// 	Flag_moving_local=Flag_moving;
// 	pthread_mutex_unlock(&mutex_walkflag);

// 	pthread_mutex_lock(&mutex_bodycommand);
// 	localbodycommand=bodycommand;
// 	pthread_mutex_unlock(&mutex_bodycommand);

// 	pthread_mutex_lock(&mutex_walkstr);
// 	localwalkstr=walkstr;
// 	prevwalkstr=walkstr;
// 	walkstr.r=0;
// 	walkstr.theta=0;
// 	pthread_mutex_unlock(&mutex_walkstr);
// }
#include "walk_thread.cpp"
#endif

enum MovingState {MOVE_BEHIND_BALL, MOVE_TO_BALL, MOVE_KICK, MOVE_STOP};

void convertPathData(int gc=1)
{
	//Path uses a coordinate system which rotates along with acyut.
	//in that coordinate system, acyut always faces positive X axis and **+Y axis lies to its right**
	pathstr.n_obstacles=fd.o.size();
	for(int i=0;i<fd.o.size();i++)//assuming right positive and left neagative for theta
	{
		pathstr.absObstacles[i].x=(fd.o[i].distance)*cos(deg2rad(fd.o[i].angle));
		pathstr.absObstacles[i].y=(fd.o[i].distance)*sin(deg2rad(fd.o[i].angle));
	}


	for (int i = 0; i < pathstr.n_obstacles; ++i)
	{
		//printf("Sent To Path: obstacle %d : %lf %lf\n", i, pathstr.absObstacles[i].x, pathstr.absObstacles[i].y);
	}
	AbsCoords goalcoords=loc.getGoalCoords(mygoal);

	if(gc == 1)
	{
		goalcoords.x = 0;
		goalcoords.y = 300.0;
	}
	else
	{
		goalcoords.x = 900;
		goalcoords.y = 300.0;
	}

	double tempx=goalcoords.x-loc.selfX;
	double tempy=goalcoords.y-loc.selfY;

	double tempr = sqrt(tempy*tempy + tempx*tempx);
	double temptheta = -(atan2(tempy, tempx) - deg2rad(loc.selfAngle)); //minus because ball angle is in that convention
	while(temptheta >= 2*PI)
		temptheta -= 2*PI;
	while(temptheta < 0)
		temptheta += 2*PI;

	pathstr.goal.x=tempr*cos(temptheta);
	pathstr.goal.y=tempr*sin(temptheta);

	printf("Sent To Path:goal coords x:%lf  y:%lf\n",pathstr.goal.x,pathstr.goal.y);

	pathstr.ball.x=fd.ball.r*cos(deg2rad(fd.ball.theta));
	pathstr.ball.y=fd.ball.r*sin(deg2rad(fd.ball.theta));

}

void moveAcyuttemp(char c)
{
	pthread_mutex_lock(&mutex_walkstr);

	walkstr.instr = c;
	walkstr.isFresh = true;

	pthread_mutex_unlock(&mutex_walkstr);
}

void check_switch()
{
	int localswitch1;
	pthread_mutex_lock(&mutex_switch);
	localswitch1=switch1;
	pthread_mutex_unlock(&mutex_switch);

	if(localswitch1!=1)
	{
		//printf("[behaviour]  Switch is OFF\n");
		exit(0);
	}
}

void updateWalkFlag()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving_mainlocal=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);
}

int getImuAngle()
{
		fstream f1;
		f1.open("xsens/imuyaw.angle", ios::in);
		int value;
		f1>>value;
		printf("Value is %d\n", value);
		f1.close();
		return value;
}

int main()
{
	pthread_t thread_id_walkthread;
	while(1)
	{
		pthread_create (&thread_id_walkthread, NULL, walk_thread, NULL);
		printf("[behaviour]  WALKTHREAD CREATED\n");

		hdmtr.bootup_files();
		capture.init();
		globalflags.reset();
		int nolocframecount=0;
		int landmarkstate1tries=0;

		sleep(2);

		//initialize
		//prevballr=0;
		double myAngle;
		int no_of_frames2=0;
		MovingState movingState;
		int ms = 0;
		int no_of_frames = 0;
		bool overrideFlag = false;

		while(1)
		{
			hdmtr.update();
			capture.getImage();
			#ifdef SWITCHACTIVE
			check_switch();
			#endif

			updateWalkFlag();
			MotionModel mm;
			fd.getLandmarks(capture, hdmtr, walkstr.mm);
			loc.doLocalize(fd, mm, getImuAngle());
			conf = loc.confidence();

			if(ms == 0)
			{
				int ballreturn = camcont.findBall(fd, hdmtr);
				printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);

				if(ballreturn != BALLFOUND)
				{
					moveAcyuttemp('x');
					no_of_frames++;

					if(no_of_frames > 180)
					{
						for (int i = 0; i < 100; ++i)
						{
							moveAcyuttemp('a');
							cvWaitKey(20);
						}
						no_of_frames = 0;
					}
					continue;
				}
				no_of_frames = 0;
				
				if(fd.ball.r > 90.0)
				{
					if(fabs(fd.ball.theta > 15))
					{
						if(fd.ball.theta > 0)
							moveAcyuttemp('d');
						else
							moveAcyuttemp('a');
					}
					else 
						moveAcyuttemp('w');
				}
				else
				{
					//stopwalk
					if(fd.ball.r > 70)
					{
						if(fabs(fd.ball.theta) > 8.0)
						{
							if(fd.ball.theta > 0)
								moveAcyuttemp('d');
							else
								moveAcyuttemp('a');
							continue;
						}

					}

					ms = 1;
					no_of_frames = 0;
					loc.randomize();
				}
			}

			else if(ms == 1)
			{
				camcont.search(hdmtr);
				moveAcyuttemp('x');
				if(conf > TH)
				{
					no_of_frames++;
					if(no_of_frames > 30)
					{
						ms = 2;
						no_of_frames = 0;
						moveAcyuttemp('x');
					}
				}
				else
				{
					no_of_frames2++;
					if(no_of_frames2 > 100)
					{
						overrideFlag = true;
						ms = 2;
					}
				}
			}

			else if(ms == 2)
			{
				convertPathData(1);
				printf("ball : %lf %lf\n",pathstr.ball.x, pathstr.ball.y );
				printf("goal: %lf %lf\n", pathstr.goal.x, pathstr.goal.y);

				pathstr.ball.y = 0;
				pathstr.ball.x = 50.0;
				double x = pathstr.goal.x - pathstr.ball.x;
				double y = pathstr.goal.y - pathstr.ball.y;
				double angle = atan2(y, x);
				angle = angle * 180.0/PI;
				// stopWalk();
				printf("~~~~~~~~~~~~~~~~~~~~~Angle = %lf\n", angle);

				if(fabs(angle) < 30)
					overrideFlag = true;

				
				if(!overrideFlag)
				{
					printf("Here in override\n");
					
					for (int i = 0; i < 35; ++i)
					{
						if(angle > 0)
							moveAcyuttemp('a');
						else
							moveAcyuttemp('d');
						cvWaitKey(100);
					}

					for (int i = 0; i < fabs(angle)/2.5; ++i)
					{
						moveAcyuttemp('w');
						cvWaitKey(100);
					}

					for (int i = 0; i < 50; ++i)
					{
						if(angle > 0)
							moveAcyuttemp('d');
						else
							moveAcyuttemp('a');
						cvWaitKey(100);
					}
				}
				else
				{
					overrideFlag = false;
				}
				
				ms = 3;
				no_of_frames = 0;
			}

			else if(ms == 3)
			{
				int ballreturn=camcont.findBall(fd,hdmtr);
				printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);
				
				if(ballreturn != BALLFOUND)
				{
					moveAcyuttemp('x');
					no_of_frames++;
					
					if(no_of_frames > 180)
					{
						for (int i = 0; i < 100; ++i)
						{
							moveAcyuttemp('a');
							cvWaitKey(20);
						}
						no_of_frames = 0;
					}
					continue;
				}
				
				if(fd.ball.r > 100)
				{
					ms = 0;
					no_of_frames = 0;
					continue;
				}

				if(fd.ball.r > 60)
				{
					if(fabs(fd.ball.theta) > 20.0)
					{
						if(fd.ball.theta > 0)
							moveAcyuttemp('d');
						else
							moveAcyuttemp('a');
					}
					else 
						moveAcyuttemp('w');
				}
				else
				{
					if(fabs(fd.ball.theta) > 15.0)
					{
						if(fd.ball.theta > 0)
							moveAcyuttemp('d');
						else
							moveAcyuttemp('a');
					}
					else 
					{
						if(fd.ball.r < 30.0)
						{
							printf("~~~~~!!!!! ThrowIn !!!~~~~~~~~\n");
							//throwin();
							break;
						}

					}
						
				}
			}

			break;
		}
	}
}
