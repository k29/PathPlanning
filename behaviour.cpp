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

//#define TESTINGBOT
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

#ifdef TESTINGBOT
#include "testbot.c"
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
pthread_mutex_t mutex_bodycommand=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_changewalkflag=PTHREAD_MUTEX_INITIALIZER;
WalkStructure walkstr;
WalkStructure prevwalkstr;
CamCapture capture;
FeatureDetection fd(capture);
HeadMotor hdmtr;
CamControl camcont(capture);
Localize loc;
float bwt,lmwt;
int ballnotinframe;
MotionModel localmm;
double conf;
PathStructure pathstr;
Path pathobj;
float prevballr,velocity;

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

//

//TODO: Resolve how to do Motion Model for Acyut Encircle

//
int callEncircle()
{
	//printf("Supposed to Encircle\n");
}

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


#ifdef SWITCHACTIVE
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
	//printf("[behaviour]  Switch is On\n");
}
#endif

inline void updateWalkFlag()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving_mainlocal=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);
}

void updatewt(double ballratio)
{
	lmwt=LM_WT*(1-conf);//lmwt should be greater
	bwt=B_WT*(ballratio);//*(1-ballnotinframe);
	
}

#define STEP_WALKBEHAV 10
#define STEP_TURNBEHAV 7.0

double getCapped(double var,double threshold)
{
	if(var>=threshold)
		return threshold;
	else
		return var;
}


void moveAcYuttemp(char c)
{
	// printf("Sending %c\n", c);
	pthread_mutex_lock(&mutex_walkstr);

	walkstr.instr = c;
	walkstr.isFresh = true;

	pthread_mutex_unlock(&mutex_walkstr);
}

void moveAcYut(double r,double theta,double enc_angle,WalkCommands com,Coords *orientArray=NULL,int length=0)//This Function sends data to walk thread using mutexlocks
{
	// pthread_mutex_lock(&mutex_walkflag);
	// int localflagmoving=Flag_moving;
	// pthread_mutex_unlock(&mutex_walkflag);
	//printf("\n[behaviour] Called moveAcYut r %lf theta %lf enc_angle %lf WalkCommand %d\n_____________________________",r,theta,enc_angle,com);
	// if(localflagmoving==0)
		//printf("Move acyut flag=0\n");


	pthread_mutex_lock(&mutex_walkstr);

	walkstr.command = com;
	walkstr.isFresh = true;
	walkstr.r = r;
	walkstr.theta = theta;

	pthread_mutex_unlock(&mutex_walkstr);
}

void initiateDive()
{
	if(fd.ball.theta>MINBALLANGLE4FALL)
	{
		//FALLRIGHT
		//printf("[behaviour] FALLRIGHT\n");
		cvWaitKey();
	}
	else if(fd.ball.theta<-MINBALLANGLE4FALL)
	{
		//FALLLEFT
		//printf("[behaviour] FALLLEFT\n");
		cvWaitKey();
	}
}


void convertPathData(int gc = 1)
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
	// printf("Goal coords are %lf %lf\n", goalcoords.x, goalcoords.y);
	// loc.selfX = 450;
	// loc.selfY = 300;
	// loc.selfAngle = 180;
	double tempx=goalcoords.x-loc.selfX;
	double tempy=goalcoords.y-loc.selfY;

	double tempr = sqrt(tempy*tempy + tempx*tempx);
	double temptheta = -(atan2(tempy, tempx) - deg2rad(loc.selfAngle)); //minus because ball angle is in that convention
	while(temptheta >= 2*PI)
		temptheta -= 2*PI;
	while(temptheta < 0)
		temptheta += 2*PI;

	// printf("Goal is r : %lf, theta: %lf\n", tempr, rad2deg(temptheta));
	// pathstr.goal.x= (tempx*cos(deg2rad(loc.selfAngle))) - (tempy* sin(deg2rad(loc.selfAngle)));//Rotating coordinate system.
	// pathstr.goal.y= (tempx*sin(deg2rad(loc.selfAngle))) + (tempy* cos(deg2rad(loc.selfAngle)));

	pathstr.goal.x=tempr*cos(temptheta);
	pathstr.goal.y=tempr*sin(temptheta);

	printf("Sent To Path:goal coords x:%lf  y:%lf\n",pathstr.goal.x,pathstr.goal.y);

	// fd.ball.r = 50;
	// fd.ball.theta = 0;
	////printf("goal coords y:%lf\n",pathstr.goal.x);
	pathstr.ball.x=fd.ball.r*cos(deg2rad(fd.ball.theta));
	pathstr.ball.y=fd.ball.r*sin(deg2rad(fd.ball.theta));

	// pathstr.ball.x = 50;
	// pathstr.ball.y = 0;
	//printf("Relative ball:R=%f  Theta=%f\n",fd.ball.r,fd.ball.theta);
	//printf("Sent To Path:ball coords x:%lf  y:%lf\n",pathstr.ball.x,pathstr.ball.y);
}


bool moveToPoint(CvPoint2D64f goalPoint)
{
	double angleTowardPoint = atan2(goalPoint.y, goalPoint.x);
	while(angleTowardPoint > CV_PI)
    	angleTowardPoint -= 2*CV_PI;
    while(angleTowardPoint < -CV_PI)
    	angleTowardPoint += 2*CV_PI;
    angleTowardPoint = -angleTowardPoint;
    double distance = sqrt(goalPoint.x*goalPoint.x + goalPoint.y*goalPoint.y);
	printf("Angle to point = %lf, Distance = %lf\n", angleTowardPoint*180./CV_PI, distance);

    #define ANGLE_THRESHOLD_COARSE 20.0*CV_PI/180.0
	#define DISTANCE_THRESHOLD_COARSE 60.0
	#define ANGLE_THRESHOLD_FINE 10.0*CV_PI/180.0
	#define DISTANCE_THRESHOLD_FINE 30.0

	if(angleTowardPoint > ANGLE_THRESHOLD_COARSE)
	{
		moveAcYuttemp('a');
	}
	else if(angleTowardPoint < -ANGLE_THRESHOLD_COARSE)
	{
		moveAcYuttemp('d');
	}
	else
	{
		if(distance > DISTANCE_THRESHOLD_COARSE)
		{
			moveAcYuttemp('w');
		}
		else
		{
			if(angleTowardPoint > ANGLE_THRESHOLD_FINE)
			{
				moveAcYuttemp('a');
			}
			else if(angleTowardPoint < - ANGLE_THRESHOLD_FINE)
			{
				moveAcYuttemp('d');
			}
			else
			{
				if(distance > DISTANCE_THRESHOLD_FINE)
				{
					moveAcYuttemp('w');
				}
				else
				{
					return true;
				}
			}
		}
	}
	return false;
}



enum MovingState {MOVE_BEHIND_BALL, MOVE_TO_BALL, MOVE_KICK, MOVE_STOP};


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
	//bootup
	//initialise
	int kickofftimecount=0;
	// Imu imu;
	// imu.init();


	// while(1) 
	// {
	// 	printf("angle %lf\n", imu.yaw);
	// 	sleep(1);
	// }


	// convertPathData();
	// printf("ball : %lf %lf\n",pathstr.ball.x, pathstr.ball.y );
	// printf("goal: %lf %lf\n", pathstr.goal.x, pathstr.goal.y);
	// double x = pathstr.goal.x - pathstr.ball.x;
	// double y = pathstr.goal.y - pathstr.ball.y;
	// printf("x : %lf y : %lf\n", x, y);
	// double angle = atan2(y, x);
	// angle = angle * 180.0/PI;
	// // stopWalk();
	// printf("~~~~~~~~~~~~~~~~~~~~~Angle = %lf\n", angle);
	// return 0;
// #define car2pol(x,y) sqrt((x)*(x)+(y)*(y))	


// 			double a=car2pol(pathstr.ball.x,pathstr.ball.y);
// 		double b=car2pol(pathstr.ball.x-pathstr.goal.x,pathstr.ball.y-pathstr.goal.y);
// 		double c=car2pol(pathstr.goal.x,pathstr.goal.y);
// 		double ballgoalangle=180- rad2deg(acos((a*a+b*b-c*c)/(2*a*b))); //using cosine rule to get angle required to encircle ( for understanding refer encircleangleidea.jpg in source)

// 		printf("angle = %lf\n", ballgoalangle);
// #undef car2pol


	#ifndef TESTINGBOT
		//bootup_files_walk();
		//initialize_acyut();
	#endif

	#ifdef TESTINGBOT
		testbot_bootup();
	#endif
	//create threads
		int movingState = -1;
	#ifdef GAMECONTROLLERACTIVE
	GameControllerAcyut GC;
	pthread_t thread_id_gc;
    pthread_create (&thread_id_gc, NULL, readGameController, NULL);//function defined in its appropriate header file
	printf("GC thread created.\n");
	RoboCupGameControlData localgcdata;
	#endif

	pthread_t thread_id_walkthread;
	pthread_t thread_id_wifi;

	#ifdef 	SWITCHACTIVE
    void *statusgpio;
    pthread_t thread_id_gpio;
    pthread_create(&thread_id_gpio,NULL,switchupdate,NULL);//function defined in its appropriate header file
	
	led(3,0);
	led(4,0);
	while(1)//Master while
	#endif
	{
		#ifdef 	SWITCHACTIVE
		check_switch();//if on, does nothing, if off, exits program
		#endif
		#ifdef WIFITRANSMISSION
		pthread_create(&thread_id_wifi,NULL,main_wifitransmission,NULL);
		#endif
		pthread_create (&thread_id_walkthread, NULL, walk_thread, NULL);
		printf("[behaviour]  WALKTHREAD CREATED\n");
		hdmtr.bootup_files();
		capture.init();
		globalflags.reset();
		int nolocframecount=0;
		int landmarkstate1tries=0;
		
		//INITIALISE---------------------------------------
		// pthread_mutex_lock(&mutex_bodycommand);
		// bodycommand=INIT;
		// pthread_mutex_unlock(&mutex_bodycommand);
		// pthread_mutex_lock(&mutex_walkflag);
		// Flag_moving=1;
		// pthread_mutex_unlock(&mutex_walkflag);
		sleep(2);
			
		// for (int i = 0; i < 40; ++i)
		// {
		// 	moveAcYuttemp('w');
		// 	cvWaitKey(100);
		// }
		//INITIALISE--------------------------------------
		prevballr=0;
		double myAngle;
		int no_of_frames2=0;
		MovingState movingState;
		int ms = 0;
		int no_of_frames = 0;
		bool overrideFlag = false;

		while(1)//main Behaviour starts
		{
			//printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

			// loc.printPosition();
			hdmtr.update();
			#ifndef WASD_CODE
			// capture.getImage();
			#endif
			#ifdef WASD_CODE
			cvShowImage("aa", capture.rgbimg);
			#endif
			#ifdef SWITCHACTIVE
			check_switch();
			#endif
			// //CHECK IMU DATA HERE AND IF FALLEN SET FALLEN FLAG TO NO/FRONT/BACK

			#ifdef WASD_CODE
			char temp = cvWaitKey(5)&0xff;
			switch(temp)
			{
				case 'w': 
				moveAcYuttemp('w');
				break;
				
				case 'a':
				moveAcYuttemp('a');
				break;
				
				case 'd':
				moveAcYuttemp('d');
				break;

				case 's':
				moveAcYuttemp('x');
				// moveAcYut(0.0, 0.0, 0, WALK);
				break;

				case 'q':
				moveAcYuttemp('q');
				break;

				case 'e':
				moveAcYuttemp('e');
				break;

				case 'p':
				moveAcYuttemp('p');
				break;

				case 'k':
				moveAcYuttemp('k');
				break;
			}
			continue;
			#endif
			if(GOALKEEPER==0)
			{
				if(globalflags.fallen==FRONT)
				{
					//FRONT GETUP
				}
				else if(globalflags.fallen==BACK)
				{
					//BACK GETUP
				}
			}
			else if(GOALKEEPER==1)
			{
				
			}

			#ifdef GAMECONTROLLERACTIVE
			pthread_mutex_lock(&mutex_GCData);
			localgcdata=GCData;
			pthread_mutex_unlock(&mutex_GCData);

			GC.update(localgcdata);
			int teamindex;
			teamindex=GC.getTeamIndex();
			printf("Here\n");
			mygoal=GC.updategoal(teamindex);
			switch(localgcdata.state)//checking gamecontroller
			{
				case STATE_READY:printf("[behaviour] currentGCstate=READY");
								cvWaitKey();
								continue;

				case STATE_PLAYING:printf("[behaviour] recieved PLAY Signal............................\n\n\n\n\n\n\n");
									break;

				default:printf("[behaviour]  Gamecontroller: No PLAY signal yet\n");
						stopWalk();
						#ifndef TESTINGBOT
							//INITIALISE---------------------------------------
							pthread_mutex_lock(&mutex_bodycommand);
							bodycommand=INIT;
							pthread_mutex_unlock(&mutex_bodycommand);
							pthread_mutex_lock(&mutex_walkflag);
							Flag_moving=1;
							pthread_mutex_unlock(&mutex_walkflag);
							sleep(2);
							//INITIALISE--------------------------------------
						#endif
						continue;
			}
			if(GOALKEEPER==0)
			{
				if(GC.isMyKickOff())
				{
					printf("its my kickoff!!! :D \n\n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
					//continue normally if not then dont walk for some time
				}
				else
				{
					printf("not my kickoff :/ \n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
					//wait
				}

				if(localgcdata.teams[localgcdata.kickOffTeam].teamNumber!=myteamnumber)
				{
					kickofftimecount++;
					if(kickofftimecount<NO_OF_FRAMES_TO_WAIT_B4_KICKOFF)
					{
						//printf("[behaviour] WALK stopped because of opponents KICKOFF\n\n");
						moveAcYuttemp('p');
						cvWaitKey(10);
						continue;
						// stopWalk();
					}
				}
			}
			#endif
			// continue;
			updateWalkFlag();
			


			
				// localmm=walkstr.mm;
				// printf("~~~~~~~~~~~~~Angle = %lf\n", imu.yaw - 127);
				//printf("Motion model updated=%d===----------------------------\n",walkstr.mm.updated);
				MotionModel mm;
				fd.getLandmarks(capture, hdmtr, walkstr.mm);

				loc.doLocalize(fd, mm, getImuAngle());
				conf = loc.confidence();



/* jugaad start */








				#ifdef WIFITRANSMISSION
				//printf("Called convert wifidata\n");
				convertTransmitData();
				#endif



				// int ballreturn=camcont.findBall(fd,hdmtr);
				// printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);
				// if(ballreturn != BALLFOUND)
				// {
				// 	moveAcYuttemp('x');
				// 	no_of_frames++;
				// 	if(no_of_frames > 180)
				// 	{
				// 		for (int i = 0; i < 100; ++i)
				// 		{
				// 			moveAcYuttemp('a');
				// 			cvWaitKey(20);
				// 		}
				// 		no_of_frames = 0;
				// 	}
				// 	continue;
				// }


				// if(fd.ball.r > 60)
				// {
				// 	if(fabs(fd.ball.theta) > 20.0)
				// 	{
				// 		if(fd.ball.theta > 0)
				// 			moveAcYuttemp('d');
				// 		else
				// 			moveAcYuttemp('a');
				// 	}
				// 	else 
				// 		moveAcYuttemp('w');
				// }
				// else
				// {
				// 	if(fabs(fd.ball.theta) > 30.0)
				// 	{
				// 		if(fd.ball.theta > 0)
				// 			moveAcYuttemp('d');
				// 		else
				// 			moveAcYuttemp('a');
				// 	}
				// 	else 
				// 		moveAcYuttemp('w');
				// }


				// continue;


		
			// ms = 0;
			// if(ms == -1)
			// {
			// 	camcont.search(hdmtr);
			// 	stopWalk();
			// 	if(conf > TH)
			// 		ms = 0;
			// }

				// ms = 3;
			if(ms == 0)
			{	
				int ballreturn=camcont.findBall(fd,hdmtr);
				printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);
				if(ballreturn != BALLFOUND)
				{
					moveAcYuttemp('x');
					no_of_frames++;
					if(no_of_frames > 180)
					{
						for (int i = 0; i < 100; ++i)
						{
							moveAcYuttemp('a');
							cvWaitKey(20);
						}
						no_of_frames = 0;
					}
					continue;
				}
				no_of_frames = 0;
				if(fd.ball.r > 90.0)
				{
					if(fabs(fd.ball.theta) > 15.0)
					{
						if(fd.ball.theta > 0)
							moveAcYuttemp('d');
						else
							moveAcYuttemp('a');
					}
					else 
						moveAcYuttemp('w');
				}
				else
				{
					// stopWalk();
					if(fd.ball.r > 70)
					{
						if(fabs(fd.ball.theta) > 8.0)
						{
							if(fd.ball.theta > 0)
								moveAcYuttemp('d');
							else
								moveAcYuttemp('a');
							continue;
						}

					}
					// for (int i = 0; i < 40; ++i)
					// {
					// 	moveAcYuttemp('x');
					// 	cvWaitKey(100);
					// }

					// for (int i = 0; i < 20; ++i)
					// {
					// 	moveAcYuttemp('o');
					// 	cvWaitKey(100);
					// }
					ms = 1;
					no_of_frames = 0;
					loc.randomize();
				}
			}
			else if(ms == 1)
			{
				camcont.search(hdmtr);
				moveAcYuttemp('x');
				if(conf > TH)
				{
					no_of_frames++;
					if(no_of_frames > 30)
					{
						ms = 2;
						no_of_frames = 0;
						// for (int i = 0; i < 30; ++i)
						// {		
						// 	moveAcYuttemp('x');
						// 	cvWaitKey(100);
						// }
						moveAcYuttemp('x');
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
				// int ballreturn=camcont.findBall(fd,hdmtr);
				// if(ballreturn != BALLFOUND)
				// {
				// 	moveAcYuttemp('p');
				// 	continue;
				// }


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

				// moveAcYuttemp('l');
				// 	sleep(2);

				if(!overrideFlag)
				{
					printf("Here in overrided\n");
										// for (int i = 0; i < 30; ++i)
					// {		
					// 	moveAcYuttemp('x');
					// 	cvWaitKey(100);
					// }
					// sleep(100);
					// stopWalk();

					for (int i = 0; i < 35; ++i)
					{
						if(angle > 0)
							moveAcYuttemp('a');
						else
							moveAcYuttemp('d');
						cvWaitKey(100);
					}

					for (int i = 0; i < fabs(angle)/2.5; ++i)
					{
						moveAcYuttemp('w');
						cvWaitKey(100);
					}

					for (int i = 0; i < 50; ++i)
					{
						if(angle > 0)
							moveAcYuttemp('d');
						else
							moveAcYuttemp('a');
						cvWaitKey(100);
					}

					// for (int i = 0; i < fabs(angle); ++i)
					// {
					// 	if(angle < 0)
					// 		moveAcYuttemp('e');
					// 	else
					// 		moveAcYuttemp('q');
					// 	cvWaitKey(100);
					// }
				}
				else
				{
					overrideFlag = false;
				}



				ms = 3;
				no_of_frames = 0;
				// if(fabs(angle) > 20)
				// {
				// 	if(angle < 0)
				// 		moveAcYuttemp('e');
				// 	else
				// 		moveAcYuttemp('q');
				// }
				// else
				// {
				// 	moveAcYuttemp('p');
				// 	ms = 3;
				// }
			}
			else if(ms == 3)
			{

				int ballreturn=camcont.findBall(fd,hdmtr);
				printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);
				if(ballreturn != BALLFOUND)
				{
					moveAcYuttemp('x');
					no_of_frames++;
					if(no_of_frames > 180)
					{
						for (int i = 0; i < 100; ++i)
						{
							moveAcYuttemp('a');
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
							moveAcYuttemp('d');
						else
							moveAcYuttemp('a');
					}
					else 
						moveAcYuttemp('w');
				}
				else
				{
					if(fabs(fd.ball.theta) > 15.0)
					{
						if(fd.ball.theta > 0)
							moveAcYuttemp('d');
						else
							moveAcYuttemp('a');
					}
					else 
					{
						if(fd.ball.r < 20.0)
						{
							for (int i = 0; i < 20; ++i)
							{
								moveAcYuttemp('k');
								cvWaitKey(100);
							}
							sleep(1);
							continue;
						}
						else
							moveAcYuttemp('w');
					}
						
				}

				continue;





				while(1)
				{
					moveAcYuttemp('p');
					cvWaitKey(100);
				}
				ballreturn=camcont.findBall(fd,hdmtr);
				printf("Ball: r: %lf theta:%lf\n", fd.ball.r, fd.ball.theta);
				if(ballreturn != BALLFOUND)
				{
					no_of_frames++;
					if(no_of_frames > 150)
						ms = 0;
					continue;
				}
				else
				{
					no_of_frames = 0;
				}

				if(fd.ball.r > 30.0)
				{
					if(fabs(fd.ball.theta) > 15.0)
					{
						if(fd.ball.theta > 0)
							moveAcYuttemp('d');
						else
							moveAcYuttemp('a');
					}
					else 
						moveAcYuttemp('w');

					continue;
				}

				// while(1)
				// {
				// 	moveAcYuttemp('p');
				// 	cvWaitKey(100);
				// }
				for (int i = 0; i < 90; ++i)
				{
					moveAcYuttemp('w');
					cvWaitKey(100);
				}
				no_of_frames = 0;
				ms = 0;
				continue;
				ballreturn=camcont.findBall(fd,hdmtr);
				if(ballreturn != BALLFOUND)
				{
					moveAcYuttemp('x');
					no_of_frames++;
					if(no_of_frames > 80)
					{
						ms = 0;
						no_of_frames = 0;
					}
					continue;
				}
				no_of_frames = 0;
				if(fd.ball.r > 90.0)
				{
					ms = 0;
				}
				else
				{
					if(fd.ball.r > 45)
					{
						if(fabs(fd.ball.theta) > 15.0)
						{
							if(fd.ball.theta > 0)
								moveAcYuttemp('d');
							else
								moveAcYuttemp('a');
						}
						else
						{
							ms = 4;
						}
					}
					else 
						ms = 4;
					// stopWalk();
				}
			}
			else
			{
				while(1)
				{
					moveAcYuttemp('p');
					printf("Reached?!?!?!\n");
					cvWaitKey(100000);
				}
			}
			
			

			continue;

/* jugaad end */
	

			//printf("[behaviour] confidence recieved = %lf\n",conf);

			// continue;
			//WEiGHTAGE FUNcTION ----------------------------------------------------------------------------------------------------------------
			updatewt(fd.ballRatio);
			
			if(globalflags.LocalizationState==0)
			{
				if(conf<TH)
				{
					camcont.search(hdmtr);
					//printf("[behaviour] Looking for LANDMARKS***** State=0\n\n");
					nolocframecount++;
					if(nolocframecount>20000)//if framecount exceeds 200 then turn a little and search for more land marks
					{
						// moveAcYut(0,45,0,WALK);
						moveAcYuttemp('a');
						nolocframecount=0;
					}
					globalflags.p=DONTGENERATE;
					globalflags.b=NOTFOUND;

				}
				else
					globalflags.LocalizationState=1;
			}
			if(globalflags.LocalizationState==1)
			{
				if(conf<TL)
				{
					globalflags.LocalizationState=0;
					continue;	//path might be made if continue not there
				}
				else
				{
					//printf("[behaviour] BALL FINDING*****\n\n");
					globalflags.p=DONTGENERATE;
					// if(bwt>lmwt)
					// {
						////printf("[behaviour] called findBall\n\n");
						int ballreturn=camcont.findBall(fd,hdmtr);
						switch(ballreturn)
						{
							case BALLFOUND:
											ballnotinframe=0;
											globalflags.b=FOUND;
											// if(fd.ball.r<BALLDIST_THRESHOLD)
											// {
											// 	globalflags.b=FOUNDNEAR;
											// 	//printf("FOUNDNEAR %lf\n",fd.ball.r);
											// }
											// else
											// {
											// 	globalflags.b=FOUNDFAR;
											// 	//printf("FOUNDFAR %lf\n",fd.ball.r);
											// }
											break;
							case TURNRIGHT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											moveAcYuttemp('d');
											// moveAcYut(0,30,0,WALK);
											break;
							case TURNLEFT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											// moveAcYut(0,-30,0,WALK);
											moveAcYuttemp('a');
											continue;
											break;
							case BALLFINDING:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											continue;
							default:continue;

						}
					// }
					// else
					// {
					// 	if(conf<TH)
					// 	{
					// 		camcont.search(hdmtr);
					// 		//printf("[behaviour] Looking for LANDMARKS***** State=1\n\n");
					// 		landmarkstate1tries++;
					// 	}
					// 	else if(landmarkstate1tries>200)
					// 	{
					// 		landmarkstate1tries=0;
					// 		globalflags.LocalizationState=0;

					// 	}
					// }
				}
			}

		




					printf("Ball : %f %f\n", fd.ball.r, fd.ball.theta);


			// int ballreturn=camcont.findBall(fd,hdmtr);
			// if(ballreturn != BALLFOUND)
			// 	continue;
			convertPathData();

			// PathReturns pathreturnval=pathobj.path_return(pathstr);
			#define DISTANCE_FROM_BALL 100.0 	// in cm
			// pathstr.goal.x, pathstr.goal.y, pathstr.ball.x, pathstr.ball.y
			CvPoint2D64f goalPoint;
			switch(movingState)
			{
				case MOVE_BEHIND_BALL:
				double vecx, vecy;
				vecx = pathstr.ball.x - pathstr.goal.x;
				vecy = pathstr.ball.y - pathstr.goal.y;
				double length;
				length = sqrt(vecx*vecx + vecy*vecy);
				if(length > 0)
				{
					vecx = vecx/length;
					vecy = vecy/length;
				}
				goalPoint.x = pathstr.ball.x + vecx*DISTANCE_FROM_BALL;
				goalPoint.y = pathstr.ball.y + vecy*DISTANCE_FROM_BALL;
				if(moveToPoint(goalPoint))
				{
					movingState = MOVE_TO_BALL;
					sleep(100);
				}
				break;

				case MOVE_TO_BALL:
				printf("Moving to ball...\n");
				goalPoint.x = pathstr.ball.x;
				goalPoint.y = pathstr.ball.y;
				if(moveToPoint(goalPoint))
				{
					// movingState = MOVE_KICK;
				}
				break;

				case MOVE_KICK:
				for (int i = 0; i < 2; ++i)
				{
					moveAcYut(15.0,0.0,0,WALK);
					sleep(2);
				}
				movingState = MOVE_STOP;
				break;

				case MOVE_STOP:
				printf("Stopped \n");
				break;
			}


continue;



			if(GOALKEEPER==0)
			{
				if(globalflags.b==FOUND)
				{
					globalflags.p=GENERATE;
				}

				
				if(Flag_moving_mainlocal==0)
				{

					//printf("[behaviour] Not Moving...\n");
					pthread_mutex_lock(&mutex_walkstr);
					localmm=walkstr.mm;
					pthread_mutex_unlock(&mutex_walkstr);

					if(globalflags.p==GENERATE)
					{
						//callpath
						//printf("[behaviour] CALLING PATH_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-\n\n");//CALLING PATH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						convertPathData();
						PathReturns pathreturnval=pathobj.path_return(pathstr);
						switch(pathreturnval)
						{
							case DOWALK:
										printf("[behaviour]-------Path Returns DOWALK--------\n");
										moveAcYut(pathobj.next.r,pathobj.next.theta,0,WALK);//REMEMBER Pathobj.next.theta has the opposite convention  hence the MINUS sign
										//cvWaitKey();
										break;
							case DOENCIRCLE:
										printf("[behaviour]-------Path Returns DOENCIRCLE--------\n");
										moveAcYut(0,pathobj.NextEncircleAngle,0,ENCIRCLE);
										//cvWaitKey();
										break;
							case DOKICK:
										printf("[behaviour]-------Path Returns DOKICK--------\n");
										moveAcYut(0,0,0,KICK);
										//cvWaitKey();
										//globalflags.reset();
										//This will pause the but when it is ready to kick. to change this, comment cvwaitkey and uncomment globalflags.reset
										//KICKING
										break;
							case NOPATH:
										printf("[behaviour]-------Path Returns NOPATH--------\n");
										//moveAcYut(path.r,path.theta,0,WALK);
										// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
										//cvWaitKey();
										break;
							case DOORIENT:
										printf("[behaviour]-------Path Returns DOORIENT--------\n");

										moveAcYut(0,0,0,ORIENT,pathobj.curvearray,pathobj.len_curvearray);
										cvWaitKey();
										break;
										
							default: break;//printf("What is happening?\n");

						}
						////printf("Preparing to walk %d\n",pathreturnval);
					}
					else
						stopWalk();	
					// else
					// {
					// 	//do something
					// 	pthread_mutex_lock(&mutex_walkstr);
					// 	walkstr.instr = 's';
					// 	pthread_mutex_unlock(&mutex_walkstr);
					// 	//continue;
					// }
				}
				//#ifdef WIFITRANSMISSION


				usleep(10000);
				

				
			}
			if(GOALKEEPER==1)
			{
				//ball is found and localised
				if(globalflags.b==FOUND)
				{
					float balldist=fd.ball.r*cos(deg2rad(fd.ball.theta));
					if(prevballr==0)
					{
						prevballr=balldist;
						continue;
					}
					velocity=prevballr-balldist;
					prevballr=balldist;
					//printf("[behaviour] Velocity(cm/frame) = %lf\nBall X=%f theta=%f\nprevballr=%f\n",velocity,balldist,fd.ball.theta,prevballr);
					if(velocity<-DIDNOTMOVEVELOCITY)
					{
						//ball is going away, good :D
					}
					if(balldist>GKFALLDISTTHRESHOLD)
					{
						if((balldist/velocity)<GKMINFRAMES2GO)//if no of frames left for ball to come close is less
						{
							initiateDive();
						}

					}
					if(balldist<GKFALLDISTTHRESHOLD)
					{
						if(velocity<DIDNOTMOVEVELOCITY && velocity>-DIDNOTMOVEVELOCITY)
						{
							//ball is stationary, go and kick it!!
							//maybe add path?
							moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
						}
						else
						{
							initiateDive();
						}
					}
					
				}
			}




			//
		}
	}

	//close threads
		void* status;
	#ifdef GAMECONTROLLERACTIVE
	void *status_gc;
	pthread_join (thread_id_gc, &status_gc);
	#endif
	#ifdef SWITCHACTIVE
	void *status_gpio;
	pthread_join (thread_id_gpio, &status_gpio);
	#endif
	#ifdef WIFITRANSMISSION
	void *status_wifi;
	pthread_join (thread_id_wifi, &status_wifi);
	#endif
	pthread_join(thread_id_walkthread,&status);

	return 0;

}
