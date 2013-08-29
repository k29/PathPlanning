#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include <sys/time.h>
#include "AcYut.h"
#include "xsens/imu.h"
#include <signal.h>

struct timeval t0,t1;
int quit = 0;

void doquit(int para)
{
	quit = 1;
}

float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
	frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

float linear(float in,float fi,float t, float tot)
{
	if(t>=0&&t<=tot)
	return in*(1-t/tot)+fi*(t/tot);
	else if(t<0)
	return in;
	else
	return fi;
}



enum SupportPhase{DSP,SSP};
float lYin  = -10;
float slYin = 10;
float vYin  = 120;
float vYfi  = 120;
float vTTar = 120;
float lZin = 20 ;
float slZin = 20;
float vZin  = -170;
float vZfi  = 170;

enum state{Y,YR,YV,Z,ZR,ZV,R,RR};

float endState[8];

float* turn(float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin, float veloZin, float veloZfi, float zMax,float lift,float legRotin, float legRotfi, float supLegRotin, float supLegRotfi, Leg *leg, Leg *revLeg, Communication *comm/*, Imu *imu*/)
{
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 10;
	float hipLength = 130;
	
	float Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	float D_dsp1Time = 0.05;
	float D_dsp2Time = 0.05;
	
	printf("Tc\t\t%lf\n",Tc);
	printf("legZin\t\t%lf\n",legZin);
	printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	float sspZAmp   = zMax; 
	float sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	float sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	float sspZSupin = sspZAmp * cosh(sspZPhs);
	float sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	float sspTime   = sspZTime;
	float dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	float dsp2Time  = D_dsp2Time;
	float stepTime  = sspTime + dsp1Time + dsp2Time;
	
	float sspZin    = legZin + (-veloZin)*dsp1Time;
	float P_sspZAmp = zMax;
	float P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	float P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	float P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	float P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	float P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	float sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	
	float legZfi    = sspZfi - veloZfi*dsp2Time;
	float supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	printf("zMax\t\t%lf\n",zMax);
	printf("sspZPhs\t\t%lf\n",sspZPhs);
	printf("sspZTime\t%lf\n",sspZTime);
	printf("sspZin\t\t%lf\n",sspZin);
	printf("sspZSupin\t%lf\n",sspZSupin);
	printf("sspZfi\t\t%lf\n",sspZfi);
	printf("sspZSupfi\t%lf\n",sspZSupfi);
	printf("dsp1Time\t%lf\n",dsp1Time);
	printf("stepTime\t%lf\n",stepTime);
	printf("LegZfi\t\t%lf\n",legZfi);
	printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	float sspYin    = legYin + veloYin * dsp1Time;
	float sspYSupin = supLegYin + veloYin * dsp1Time;
	float sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	float sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	float veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	float P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	float P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	float P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	float P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	float sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	float sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	float legYfi    = sspYfi + veloYfi_d*dsp2Time;
	float supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	printf("sspYAmp\t\t%lf\n",sspYAmp);
	printf("sspYPhs\t\t%lf\n",sspYPhs);
	printf("sspYfi\t\t%lf\n",sspYfi);
	printf("sspYSupfi\t%lf\n",sspYSupfi);
	printf("legYfi\t\t%lf\n",legYfi);
	printf("suplegYfi\t%lf\n",supLegYfi);
	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	printf("P_sspZTime\t%lf\n",P_sspZTime);
	printf("P_sspYin\t\t%lf\n",P_sspYin);
	printf("P_legYin\t\t%lf\n",P_legYin);
	
	// Y (cubic)
	
	float a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	float b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	float c = -veloYin;
	float d = -sspYin;
	
	float height = 390;
	//float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time-1.0/60.0;
	float stopX  = stepTime-dsp2Time + 1.0/60.0;
	float xPhase =-3.14/2;
	
	float dampLift = 0;
	float dampFreq = 2*3.14;
	float dampStart = dsp1Time + sspTime - 4.0/60.0;
	float dampEnd   = stepTime;
	float dampPhase = -3.14/2;
	float dampDisplacement = 1;
	
	float fraction;
	
	float x,xr,y,yr,z,zr,phi,phiR;
	float yReal,yrReal,zReal,zrReal;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

	printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
			printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= linear(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=linear(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
			printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
			printf("DSP2\t");
		}
		else 
		{
			printf("************* SOMETHING WRONG*******************\n");
		}
		///printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		///printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		///printf("Z\t%lf\tZR\t%lf\n",z,zr);
		printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		
		if(leg->leg==LEFT)
		{
			zReal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yReal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrReal=zr*cos(phiR*3.14/180)-yr*sin(phiR*3.14/180);
			yrReal=zr*sin(phiR*3.14/180)+yr*cos(phiR*3.14/180);
		}
		else if(leg->leg==RIGHT)
		{
			zReal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yReal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrReal=zr*cos(-phiR*3.14/180)+yr*sin(phiR*3.14/180);
			yrReal=-zr*sin(-phiR*3.14/180)+yr*cos(phiR*3.14/180);
			
		}
		
		//printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		leg->move(x,-10,z+feetSeperation,phi);
		revLeg->move(xr,-10,zr+feetSeperation,phiR);
		comm -> syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	
	return endState;
}

float* turn2(float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin, float veloZin, float veloZfi, float zMax,float lift,float strafe,float legRotin, float legRotfi, float supLegRotin, float supLegRotfi, Leg *leg, Leg *revLeg, Communication *comm)
{
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 10;
	float hipLength = 130;
	
	float Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	float D_dsp1Time = 0.05;
	float D_dsp2Time = 0.05;
	
	printf("Tc\t\t%lf\n",Tc);
	printf("legZin\t\t%lf\n",legZin);
	printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	float sspZAmp   = zMax; 
	float sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	float sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	float sspZSupin = sspZAmp * cosh(sspZPhs);
	float sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	float sspTime   = sspZTime;
	float dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	float dsp2Time  = D_dsp2Time;
	float stepTime  = sspTime + dsp1Time + dsp2Time;
	
	float correction=0;
	
	float sspZin    = legZin + (-veloZin)*dsp1Time;
	float P_sspZAmp = zMax;
	float P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	float P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	float P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	float P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	float P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	float sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	
	float legZfi    = sspZfi - veloZfi*dsp2Time;
	float supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	printf("zMax\t\t%lf\n",zMax);
	printf("sspZPhs\t\t%lf\n",sspZPhs);
	printf("sspZTime\t%lf\n",sspZTime);
	printf("sspZin\t\t%lf\n",sspZin);
	printf("sspZSupin\t%lf\n",sspZSupin);
	printf("sspZfi\t\t%lf\n",sspZfi);
	printf("sspZSupfi\t%lf\n",sspZSupfi);
	printf("dsp1Time\t%lf\n",dsp1Time);
	printf("stepTime\t%lf\n",stepTime);
	printf("LegZfi\t\t%lf\n",legZfi);
	printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	float sspYin    = legYin + veloYin * dsp1Time;
	float sspYSupin = supLegYin + veloYin * dsp1Time;
	float sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	float sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	float veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	float P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	float P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	float P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	float P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	float sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	float sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	float legYfi    = sspYfi + veloYfi_d*dsp2Time;
	float supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	printf("sspYAmp\t\t%lf\n",sspYAmp);
	printf("sspYPhs\t\t%lf\n",sspYPhs);
	printf("sspYfi\t\t%lf\n",sspYfi);
	printf("sspYSupfi\t%lf\n",sspYSupfi);
	printf("legYfi\t\t%lf\n",legYfi);
	printf("suplegYfi\t%lf\n",supLegYfi);
	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	printf("P_sspZTime\t%lf\n",P_sspZTime);
	printf("P_sspYin\t\t%lf\n",P_sspYin);
	printf("P_legYin\t\t%lf\n",P_legYin);
	
	// Y (cubic)
	
	float a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	float b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	float c = -veloYin;
	float d = -sspYin;
	
	float height = 390;
	//float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time-1.0/60.0;
	float stopX  = stepTime-dsp2Time + 1.0/60.0;
	float xPhase =-3.14/2;
	
	float dampLift = 0;
	float dampFreq = 2*3.14;
	float dampStart = dsp1Time + sspTime - 4.0/60.0;
	float dampEnd   = stepTime;
	float dampPhase = -3.14/2;
	float dampDisplacement = 1;
	
	float fraction;
	
	float x,xr,y,yr,z,zr,phi,phiR;
	float yReal,yrReal,zReal,zrReal;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

//	printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
			printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= linear(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=linear(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			correction=linear(0,strafe,walkTime-dsp1Time,sspTime);
			state = SSP;
			printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
			correction=strafe;
			printf("DSP2\t");
		}
		else 
		{
			printf("************* SOMETHING WRONG*******************\n");
		}
		///printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		///printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		///printf("Z\t%lf\tZR\t%lf\n",z,zr);
	printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		//imu->__update();
		//printf("Roll: %lf Pitch: %lf Yaw: %lf\n", imu->roll, imu->pitch, imu->yaw);
		
		if(leg->leg==LEFT)
		{
			zReal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yReal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrReal=zr*cos(phiR*3.14/180)-yr*sin(phiR*3.14/180);
			yrReal=zr*sin(phiR*3.14/180)+yr*cos(phiR*3.14/180);
		}
		else if(leg->leg==RIGHT)
		{
			zReal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yReal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrReal=zr*cos(-phiR*3.14/180)+yr*sin(phiR*3.14/180);
			yrReal=-zr*sin(-phiR*3.14/180)+yr*cos(phiR*3.14/180);
			
		}
		
		//printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		leg->move(x,0,z+feetSeperation+correction,phi);
		revLeg->move(xr,0,zr+feetSeperation,phiR);
		comm -> syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	
	return endState;
}

float* dribble(float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin, float veloZin, float veloZfi, float zMax,float legRotin, float legRotfi, float supLegRotin, float supLegRotfi, Leg *leg, Leg *revLeg, Communication *comm/* ,Imu *imu */)
{
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 10;
	float hipLength = 130;
	
	float Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	float D_dsp1Time = 0.05;
	float D_dsp2Time = 0.05;
	
	printf("Tc\t\t%lf\n",Tc);
	printf("legZin\t\t%lf\n",legZin);
	printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	float sspZAmp   = zMax; 
	float sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	float sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	float sspZSupin = sspZAmp * cosh(sspZPhs);
	float sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	float sspTime   = sspZTime;
	float dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	float dsp2Time  = D_dsp2Time;
	float stepTime  = sspTime + dsp1Time + dsp2Time;
	
	float sspZin    = legZin + (-veloZin)*dsp1Time;
	float P_sspZAmp = zMax;
	float P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	float P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	float P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	float P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	float P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	float sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	
	float legZfi    = sspZfi - veloZfi*dsp2Time;
	float supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	printf("zMax\t\t%lf\n",zMax);
	printf("sspZPhs\t\t%lf\n",sspZPhs);
	printf("sspZTime\t%lf\n",sspZTime);
	printf("sspZin\t\t%lf\n",sspZin);
	printf("sspZSupin\t%lf\n",sspZSupin);
	printf("sspZfi\t\t%lf\n",sspZfi);
	printf("sspZSupfi\t%lf\n",sspZSupfi);
	printf("dsp1Time\t%lf\n",dsp1Time);
	printf("stepTime\t%lf\n",stepTime);
	printf("LegZfi\t\t%lf\n",legZfi);
	printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	float sspYin    = legYin + veloYin * dsp1Time;
	float sspYSupin = supLegYin + veloYin * dsp1Time;
	float sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	float sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	float veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	float P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	float P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	float P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	float P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	float sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	float sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	float legYfi    = sspYfi + veloYfi_d*dsp2Time;
	float supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	printf("sspYAmp\t\t%lf\n",sspYAmp);
	printf("sspYPhs\t\t%lf\n",sspYPhs);
	printf("sspYfi\t\t%lf\n",sspYfi);
	printf("sspYSupfi\t%lf\n",sspYSupfi);
	printf("legYfi\t\t%lf\n",legYfi);
	printf("suplegYfi\t%lf\n",supLegYfi);
	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	printf("P_sspZTime\t%lf\n",P_sspZTime);
	printf("P_sspYin\t\t%lf\n",P_sspYin);
	printf("P_legYin\t\t%lf\n",P_legYin);
	
	// Y (cubic)
	
	float a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	float b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	float c = -veloYin;
	float d = -sspYin;
	
	float height = 390;
	float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time-1.0/60.0;
	float stopX  = stepTime-dsp2Time + 1.0/60.0;
	float xPhase =-3.14/2;
	
	float dampLift = 0;
	float dampFreq = 2*3.14;
	float dampStart = dsp1Time + sspTime - 4.0/60.0;
	float dampEnd   = stepTime;
	float dampPhase = -3.14/2;
	float dampDisplacement = 1;
	
	float fraction;
	
	float x,xr,y,yr,z,zr,phi,phiR;
	float yReal,yrReal,zReal,zrReal;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

	printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
			printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= linear(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=linear(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
			printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
			printf("DSP2\t");
		}
		else 
		{
			printf("************* SOMETHING WRONG*******************\n");
		}
		///printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		///printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		///printf("Z\t%lf\tZR\t%lf\n",z,zr);
		printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		
		if(leg->leg==LEFT)
		{
			zReal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yReal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrReal=zr*cos(phiR*3.14/180)-yr*sin(phiR*3.14/180);
			yrReal=zr*sin(phiR*3.14/180)+yr*cos(phiR*3.14/180);
		}
		else if(leg->leg==RIGHT)
		{
			zReal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yReal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrReal=zr*cos(-phiR*3.14/180)+yr*sin(phiR*3.14/180);
			yrReal=-zr*sin(-phiR*3.14/180)+yr*cos(phiR*3.14/180);
			
		}
		
		//printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		leg->move(x,y,z+feetSeperation,0);
		revLeg->move(xr,yr,zr+feetSeperation,0);
		comm -> syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	
	return endState;
}

void* walk_thread(void*)
{
	//printf("Call_walk working1\n");
	double r,theta,theta1;
	float *state= endState;

	state[YR]=lYin;
 	state[Y]=slYin;
 	state[YV]=vYin;
 	state[ZR]=lZin;
 	state[Z]=slZin;
 	state[ZV]=-vZin;
	int flag=0;
	int tempFlag = 0;
	int start_flag=0;
	Communication comm;
	AcYut bot(&comm);
	return 0;
	flag=0;
				
	int l=0;
	state = turn(lYin,slYin,vYin,vYfi,lZin,slZin,vZin,vZfi,70,5,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
	flag=1-flag;
	state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,3,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
	flag=1-flag;
	while(l<3)
	{
		l++;
		state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,10*l,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
		flag=1-flag;
		state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,10*l,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
		flag=1-flag;
	}
	
	while(TRUE)
	{

		
		tempFlag++;
		read_walk_data();
		
		// pthread_mutex_lock(&mutex_walkflag);
		// Flag_moving_local=Flag_moving;
		// pthread_mutex_unlock(&mutex_walkflag);

		r=localwalkstr.r;
		theta=localwalkstr.theta;
		printf("[walk] running\n");

		//check for init/walk haere
		//if(localbodycommand==WALK){then walk} else if ==INIT initialise and  readwait till flagmoving is one
		// if(Flag_moving_local==1)
		{
			printf("YO\n");	
			// if(localbodycommand==WALK)
			// {	
			
				printf("``````````````````````````````````````~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`````````````````````````````````\n[Walk] Received Command WALK r=%lf theta=%lf\n",localwalkstr.r,localwalkstr.theta);
				theta1=abs(theta);
				float  r1=r;
				float x=0;
				float turn_dist1=0,turn_dist2=0;
				float walk_dist1=0,walk_dist2=0;
				
				switch(localwalkstr.instr)
				{
					case 'w' :	state=dribble(state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],70.0,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=0;
								walkstr.mm.theta2=0;
								walkstr.mm.r=state[Y]-state[YR];
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								pthread_mutex_unlock(&mutex_walkstr);
								break;
					
					case 'd' :	if(flag==0)
								{
									state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
									flag=1-flag;
								}
								state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,10,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=10;
								walkstr.mm.theta2=0;
								walkstr.mm.r=0;
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								pthread_mutex_unlock(&mutex_walkstr);
								break;
					case 'a' :	if(flag==1)
								{
									state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
									flag=1-flag;
								}
								state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,10,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag; 
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=-10;
								walkstr.mm.theta2=0;
								walkstr.mm.r=0;
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								pthread_mutex_unlock(&mutex_walkstr);
								break;
					case 'q' :	if(flag==1)
								{
									state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
									flag=1-flag;
								}
								state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,20,0,-10,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,-20,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag; 
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=-90;
								walkstr.mm.theta2=90;
								walkstr.mm.r=100;
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								pthread_mutex_unlock(&mutex_walkstr);
								break;			
					case 'e' :	if(flag==0)
								{
									state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
									flag=1-flag;
								}
								state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,20,0,-10,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,-20,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=90;
								walkstr.mm.theta2=-90;
								walkstr.mm.r=100;
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								pthread_mutex_unlock(&mutex_walkstr);
								break;
					case 'x' :	localwalkstr.instr='s';
								break;			
					default :	state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
								flag=1-flag;
								pthread_mutex_lock(&mutex_walkstr);
								walkstr.mm.theta=0;
								walkstr.mm.r=0;
								localwalkstr.instr='s';
								walkstr.mm.updated=1;
								walkstr.mm.theta2=0;
								pthread_mutex_unlock(&mutex_walkstr);
								break;									
				}
			
				// state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
				// x=x+state[Y]-state[YR];
				// flag=1-flag;
				// state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
				// x=x+state[Y]-state[YR];
				// flag=1-flag;
/*				if(theta<0&&flag==1)
				{
				state = dribble(state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],75.0,0,0,0,0,bot.leg[1],bot.leg[0],&comm);
				x=x+state[Y]-state[YR];
				flag=1-flag;
				}
				else if(theta>0&&flag==0)
				{
				state = dribble(state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],75.0,0,0,0,0,bot.leg[0],bot.leg[1],&comm);
				x=x+state[Y]-state[YR];
				flag=1-flag;	
				}
				while(theta1>10)
				{
					if(theta>0)
					{
						if(flag==0)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
						}
						else if(flag==1)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
						}
					}
					else if(theta<0)
					{
						if(flag==0)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
						}
						else if(flag==1)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
						}
					}
					theta1=theta1-10;
					pthread_mutex_lock(&mutex_walkstr);
					walkstr.mm.theta=(theta>0?1:-1)*20;
					walkstr.mm.r=turn_dist1+turn_dist2;
					walkstr.mm.updated=1;
					pthread_mutex_unlock(&mutex_walkstr);
				}

				if(theta>0)
				{
					if(flag==0)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
						}
						else if(flag==1)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
						}
					pthread_mutex_lock(&mutex_walkstr);
					walkstr.mm.theta=(theta>0?1:-1)*10;
					walkstr.mm.r=turn_dist1+turn_dist2;
					walkstr.mm.updated=1;
					pthread_mutex_unlock(&mutex_walkstr);
				}
				else if(theta<0)
				{
					if(flag==0)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
						}
						else if(flag==1)
						{
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,0,10,0,0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,30,state[RR],0,state[R],0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
						}
					pthread_mutex_lock(&mutex_walkstr);
					walkstr.mm.theta=(theta>0?1:-1)*20;
					walkstr.mm.r=turn_dist1+turn_dist2;
					walkstr.mm.updated=1;
					pthread_mutex_unlock(&mutex_walkstr);
				}

				while(x<r1)
				{
					if(flag==0)
						{
							state = dribble(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,0,10,0,0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							x=x+state[Y]-state[YR];
							state = dribble(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,state[RR],0,state[R],0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							x=x+state[Y]-state[YR];
						}
						else if(flag==1)
						{
							state = dribble(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,0,10,0,0,bot.leg[1],bot.leg[0],&comm);
							flag=1-flag;
							x=x+state[Y]-state[YR];
							walk_dist1=state[Y]-state[YR];
							
							state = dribble(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],75.0,state[RR],0,state[R],0,bot.leg[0],bot.leg[1],&comm);
							flag=1-flag;
							x=x+state[Y]-state[YR];
							walk_dist2=state[Y]-state[YR];
							
						}//Return Motion Model data
	*/				
	/*				pthread_mutex_lock(&mutex_walkstr);
					walkstr.mm.r=walk_dist1+walk_dist2;
					walkstr.mm.updated=1;
					pthread_mutex_unlock(&mutex_walkstr);
	*/
	/*				int breakflag=0;
				
					pthread_mutex_lock(&mutex_changewalkflag);
					if(global_CHANGEWALKFLAG==1)
					{
						//endwalk(flag,state[YR],state[Y],state[ZR],state[Z]);
						global_CHANGEWALKFLAG=0;
						breakflag=1;	
					}
					pthread_mutex_unlock(&mutex_changewalkflag);
					if(breakflag==1){break;}
					printf("!!!!!!!!!!!!!!! x = %lf \n\n\n\n\n\n\n\n", x);

					
				}
*/				//stopWalk();
			// }//if command==WALK
			// else if(localbodycommand==INIT)
			// {
			// 	// initialize_acyut();
			// 	bot.initialize();
			// //	stopWalk();
			// }
		}
		// else
		// {
		// 	usleep(20000);
		// 	continue;
		// }
	}//while ends
	printf("***\n");
	cvWaitKey();
}	

