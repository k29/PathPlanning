#include <opencv2/opencv.hpp>
#include "camcapture.h"
#include "defines.h"
#include "headmotor.h"
#include "commondefs.h"
#include "camcontrol.h"
#include "imgproc.h"





int CamControl::moveToPoint(int x, int y, HeadMotor &hm)
{
	// float hori = 0;
	// float vert = 0;
	float theta_x, theta_y;
	
	int return_value = 0;

	if(hm.read_motor(theta_x, theta_y)!=1)
		return -4;
	hm.speed_motor(700,700);//HeadMotor::MOTOR_SPEED,700);
	//Updating thetaX and thetaY when reading from motor. This is to probably save on reading from
	//motor repeatedly, if this feature of function is known.
	//Also, thetaX and thetaY variables are used directly by findReal.
	//Should probably change findReal
	thetaX = deg2rad(theta_x);
	thetaY = deg2rad(theta_y);

	//updating theta_x and _y to values to be sent to motor,
	// according to ball values x and y
/*	
	FIXME
	Earlier, *probably* camera was getting stuck in front of ball because
	x and y were in integer operations, and had not been typecasted
	to float
	So, if ball was on edge of the rectangle, the camera got stuck, as 
	integer operations reduced values of less than 1 to 0
	Maybe problem will still come, if so, try rounding up over here,
	or changing the 5 and 8 to larger values.
	If problem persists, read whole code again..
*/
#ifndef PID_BALL
	theta_x= theta_x + ((IMAGE_HEIGHT/2-(float)y)*15.0/(IMAGE_HEIGHT/2));//15
	theta_y = theta_y + (((float)x-(IMAGE_WIDTH/2))*20.0/(IMAGE_WIDTH/2));//30	
#endif

#ifdef PID_BALL
	theta_x = theta_x + ((float)e_x)*KP_X + (float)e_x_int*KI_X + (float)e_x_diff*KD_X;
	theta_y = theta_y - ((float)e_y)*KP_Y - (float)e_y_int*KI_Y - (float)e_y_diff*KD_Y;
#endif

	if((theta_x <= 1)||(theta_x >= 89))	//Max vertical limits. Maybe not hardcode them?
		return -2;

	if(theta_y <=LEFT_LIMIT +1)
		return_value = -10;
	if(theta_y >=RIGHT_LIMIT-1)
		return_value = -20;
			
	//Old technique
	// if( x < CENTRE_RECT_X1 )
	// 	hori = -1;
	// else if ( x > CENTRE_RECT_X2 )
	// 	hori = +1;
		
	// if (y < CENTRE_RECT_Y1)
	// 	vert = +1;
	// else if(y > CENTRE_RECT_Y2)
	// 	vert = -1;

	//	theta_x = theta_x + vert*5;
	//	theta_y = theta_y + hori*10;	
	
	if(theta_x < 0 )
	{
		theta_x = 0;
	}
	else if(theta_x > 90)
	{
		theta_x = 90;
	}
			
	if(theta_y < LEFT_LIMIT)
	{
		theta_y = LEFT_LIMIT;
	}
	else if(theta_y > RIGHT_LIMIT)
	{
		theta_y = RIGHT_LIMIT;
	}
	
	hm.write_motor(theta_x, theta_y);
	
	return return_value;
} 

