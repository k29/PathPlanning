#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include <sys/time.h>
#include "AcYut.h"
// #include "xsens/imu.h"
#include <opencv2/opencv.hpp>

struct timeval t0,t1;
int quit = 0;
int flag = 0;

int kbhit()
{
	struct timeval tv;
	fd_set read_fd;
	tv.tv_sec=0;
	tv.tv_usec=0;
	FD_ZERO(&read_fd);
	FD_SET(0,&read_fd);
	if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
		return 0; /* An error occured */
	if(FD_ISSET(0,&read_fd))
		return 1;
	return 0;
}

void doquit(int para)
{
	quit = 1;
}

double scurve(double in,double fi,double t, double tot)
{
	double frac=(double)t/(double)tot;
	if(frac>1)
	frac=1;
	double retfrac=frac*frac*(3-2*frac);
	double mval=in + (fi-in)*retfrac;
	return mval;
}

double linear(double in,double fi,double t, double tot)
{
	if(t>=0&&t<=tot)
	return in*(1-t/tot)+fi*(t/tot);
	else if(t<0)
	return in;
	else
	return fi;
}



enum SupportPhase{DSP,SSP};
enum state{Y,YR,YV,Z,ZR,ZV,R,RR,S,SR};
double endState[10];
	
double* dribble(double legYin,
		double supLegYin,
		double veloYin,
		double veloYfi,
		double legZin,
		double supLegZin,
		double veloZin,
		double veloZfi,
		double zMax,
		double legRotin,
		double legRotfi,
		double supLegRotin,
		double supLegRotfi,
		double legStfin,
		double legStffi,
		double supLegStfin,
		double supLegStffi,
		Leg *leg,
		Leg *revLeg,
		Communication *comm,
		double lift=30)

{
//	////printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 3;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.05;
	double D_dsp2Time = 0.05;
	
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	////printf("\n\n\n");
	////printf("legYin\t\t%lf\n",legYin);
	////printf("supLegYin\t%lf\n",supLegYin);
	////printf("sspYin\t\t%lf\n",sspYin);
	////printf("sspYSupin\t%lf\n",sspYSupin);
	////printf("sspYAmp\t\t%lf\n",sspYAmp);
	////printf("sspYPhs\t\t%lf\n",sspYPhs);
	////printf("sspYfi\t\t%lf\n",sspYfi);
	////printf("sspYSupfi\t%lf\n",sspYSupfi);
	////printf("legYfi\t\t%lf\n",legYfi);
	////printf("suplegYfi\t%lf\n",supLegYfi);
	////printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	////printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	////printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	////printf("P_sspZTime\t%lf\n",P_sspZTime);
	////printf("P_sspYin\t\t%lf\n",P_sspYin);
	////printf("P_legYin\t\t%lf\n",P_legYin);
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 390;
	//double lift   = 30;
	double xfreq  = 2*3.14;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-3.14/2;
	
	double dampLift = 0;
	double dampFreq = 2*3.14;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -3.14/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
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
			s=legStfin;
			sr=supLegStfin;
	//		////printf("DSP1\t");
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
			s=linear(legStfin,legStffi,walkTime-dsp1Time,sspTime);
			sr=linear(supLegStfin,supLegStffi,walkTime-dsp1Time,sspTime);			
			state = SSP;
	//		////printf("SSP0\t");
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
			s= legStffi;
			sr= supLegStffi;			
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		//////printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		// z = z + s;
		// zr = zr + sr;
		
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
		
		//////printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		// ////printf("~~~~~~~~~~~~~~~s%f sr%f\n", s, sr);
		leg->move(x,y,z+feetSeperation+s,phi);
		revLeg->move(xr,yr,zr+feetSeperation+sr,phiR);
		comm->syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep*8/10);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	endState[S]  = legStffi;
	endState[SR] = supLegStffi;
	
	return endState;
}




double* kick(double legYin, double supLegYin, double veloYin, double veloYfi, double legZin, double supLegZin, double veloZin, double veloZfi, double zMax, double legRotin, double legRotfi, double supLegRotin, double supLegRotfi, double legStfin, double legStffi, double supLegStfin, double supLegStffi, Leg *leg, Leg *revLeg, Communication *comm)

{
//	////printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 3;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.05;
	double D_dsp2Time = 0.05;
	
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	////printf("\n\n\n");
	////printf("legYin\t\t%lf\n",legYin);
	////printf("supLegYin\t%lf\n",supLegYin);
	////printf("sspYin\t\t%lf",sspYin);
	////printf("sspYSupin\t%lf\n",sspYSupin);
	////printf("sspYAmp\t\t%lf\n",sspYAmp);
	////printf("sspYPhs\t\t%lf\n",sspYPhs);
	////printf("sspYfi\t\t%lf\n",sspYfi);
	////printf("sspYSupfi\t%lf\n",sspYSupfi);
	////printf("legYfi\t\t%lf\n",legYfi);
	////printf("suplegYfi\t%lf\n",supLegYfi);
	////printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	////printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	////printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	////printf("P_sspZTime\t%lf\n",P_sspZTime);
	////printf("P_sspYin\t\t%lf\n",P_sspYin);
	////printf("P_legYin\t\t%lf\n",P_legYin);
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double C=(-sspYfi+sspYin)*1.3;
	double A=(sspYin+C)/(exp(0.5)-1);
	double B=-sspYin-A; 
	
	double height = 390;
	double lift   = 60;
	double xfreq  = 2*3.14;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-3.14/2;
	
	double dampLift = 0;
	double dampFreq = 2*3.14;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -3.14/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
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
			s=legStfin;
			sr=supLegStfin;
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			if(fraction<=1)
			y = A*exp(fraction/2)+B;
			else
			y = scurve(A*exp(0.5)+B,-sspYfi,walkTime-dsp1Time,sspTime);
			//y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			if((fraction==0)||(fraction==2))
				//printf("\n%lf\n",y);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= linear(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=linear(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			s=linear(legStfin,legStffi,walkTime-dsp1Time,sspTime);
			sr=linear(supLegStfin,supLegStffi,walkTime-dsp1Time,sspTime);			
			state = SSP;
	//		////printf("SSP0\t");
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
			s= legStffi;
			sr= supLegStffi;			
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		//printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		// z = z + s;
		// zr = zr + sr;
		
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
		
		//////printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		// ////printf("~~~~~~~~~~~~~~~s%f sr%f\n", s, sr);
		leg->move(x,y,z+feetSeperation+s,phi);
		revLeg->move(xr,yr,zr+feetSeperation+sr,phiR);
		comm->syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep*8/10);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	endState[S]  = legStffi;
	endState[SR] = supLegStffi;
	
	return endState;
}


double* lipm_start(AcYut bot,Communication &comm)
{
	////printf("Lipm_start\n\n\n\n");
	double* state=endState;
	state=dribble(-1.5,1.5,10,10,25.15,25.15,-170,170,70.0,0,0,0,0,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm,5);
	flag=1-flag;
	////printf("%lf\n",state[YV] );
	int l=0;
	while(l<3)
	{	
		l++;
		state=dribble(state[YR],state[Y],state[YV],10,25.15,25.15,-170,170,70.0,0,0,0,0,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm,10*l);
		flag=1-flag;
		////printf("%lf\n",state[YV] );
		state=dribble(state[YR],state[Y],state[YV],10,25.15,25.15,-170,170,70.0,0,0,0,0,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm,10*l);
		flag=1-flag;
		////printf("%lf\n",state[YV] );
	}
	////printf("LIPM start ending\n%lf",state[YV]);
	return state;
}


double* turn2(double legYin, double supLegYin, double veloYin, double veloYfi, double legZin, double supLegZin, double veloZin, double veloZfi, double zMax,double lift,double strafe,double legRotin, double legRotfi, double supLegRotin, double supLegRotfi, Leg *leg, Leg *revLeg, Communication *comm)
{
	//////printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 10;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.05;
	double D_dsp2Time = 0.05;
	
	////printf("Tc\t\t%lf\n",Tc);
	////printf("legZin\t\t%lf\n",legZin);
	////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double correction=0;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
/*	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = 0;//veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = 0;//asinh(1/sqrt(pow(Tc,2) - pow(1,2)));
	double veloYfi_d   = 0;//sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = 0;//sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = 0;//-acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = 0;//P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = 0;//sspYAmp * sinh(sspTime/Tc + sspYPhs);*/
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	

	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	////printf("\n\n\n");
	////printf("legYin\t\t%lf\n",legYin);
	////printf("supLegYin\t%lf\n",supLegYin);
	////printf("sspYin\t\t%lf\n",sspYin);
	////printf("sspYSupin\t%lf\n",sspYSupin);
	////printf("sspYAmp\t\t%lf\n",sspYAmp);
	////printf("sspYPhs\t\t%lf\n",sspYPhs);
	////printf("sspYfi\t\t%lf\n",sspYfi);
	////printf("sspYSupfi\t%lf\n",sspYSupfi);
	////printf("legYfi\t\t%lf\n",legYfi);
	////printf("suplegYfi\t%lf\n",supLegYfi);
	////printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	////printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	////printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	////printf("P_sspZTime\t%lf\n",P_sspZTime);
	////printf("P_sspYin\t\t%lf\n",P_sspYin);
	////printf("P_legYin\t\t%lf\n",P_legYin);
	
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 390;
	//double lift   = 30;
	double xfreq  = 2*3.14;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-3.14/2;
	
	double dampLift = 0;
	double dampFreq = 2*3.14;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -3.14/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

//	////printf("*************************************** STEP **********************************************\n");	
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
			////printf("DSP1\t");
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
			////printf("SSP0\t");
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
			////printf("DSP2\t");
		}
		else 
		{
			////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		///////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
	////printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		//imu->__update();
		// ////printf("Roll: %lf Pitch: %lf Yaw: %lf\n", imu->roll, imu->pitch, imu->yaw);
		
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
		
		//////printf("Roll : %lf\tPitch : %lf\tYaw : %lf\n",imu->roll,imu->pitch,imu->yaw);
		
		leg->move(x,0,z+feetSeperation+correction,phi);
		revLeg->move(xr,0,zr+feetSeperation,phiR);
		comm -> syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep*8/10);
		
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
	// Communication comm;
	// AcYut bot(&comm);

	// flag=0;
				
	// int l=0;
	// state = turn(lYin,slYin,vYin,vYfi,lZin,slZin,vZin,vZfi,70,5,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
	// flag=1-flag;
	// state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,3,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
	// flag=1-flag;
	// while(l<3)
	// {
	// 	l++;
	// 	state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,10*l,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
	// 	flag=1-flag;
	// 	state = turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70,10*l,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
	// 	flag=1-flag;
	// }//LIPM start 



	Communication comm;

	AcYut bot(&comm);
	double *state = endState;
	double currVelo = 10;
	double legRot =0;
	double supLegRot =0;
	double legStf = 0;
	double supLegStf = 0;
	double ZMAX = 70;
	double veloStep = 1.72;
	state[Y]=1;
	state[YR]=-1;
	state[YV]=10;
	state[Z]=25.15;
	state[ZR]=25.15;
	state[ZV]=170;	
	// (void) signal(SIGINT,doquit);
	char ch = 's';
	double ang = fabs(14);
	double maxVelo = 90;
	double minVelo =9.5;
	state=lipm_start(bot,comm);

	while(1)
	{

		pthread_mutex_lock(&mutex_walkstr);
		WalkStructure local = walkstr;
		walkstr.isFresh = false;
		pthread_mutex_unlock(&mutex_walkstr);
		////printf("[walk] running\n");

		// r = local.r;
		// theta = local.theta;
		//check for init/walk haere
		//if(localbodycommand==WALK){then walk} else if ==INIT initialise and  readwait till flagmoving is one
		if(local.isFresh)
		{
			ch = walkstr.instr;
		}
		else
		{
			ch = 'x';
		}
		switch(ch)
		{
			case 'w': 
				if(currVelo*veloStep <=maxVelo)
					currVelo =currVelo*veloStep;
				////printf("%lf\n",currVelo );
				if(currVelo > maxVelo-5)
				{
					//printf("here\n");
					if(flag == 0)
						legRot = 2.9;
					else
						legRot = 0;
				}
				supLegRot =0;
				legStf=0;
				supLegStf=0;
				////printf("w\n");
			break;
			case 'a':
				if(currVelo/veloStep >=minVelo)
					currVelo =currVelo/veloStep;
				if(flag == 0)
				{
					legRot = ang;
					supLegRot = 0;
				}
				else
				{
					supLegRot = 0;
					legRot = -0;
				}
				legStf=0;
				supLegStf=0;
				////printf("a\n");
			break;
			case 'd':
				if(currVelo /veloStep >=minVelo)
					currVelo =currVelo/veloStep;
				if(flag == 1)
				{
					legRot = ang;
					supLegRot = 0;
				}
				else
				{
					supLegRot = 0;
					legRot = -0;
				}
				legStf=0;
				supLegStf=0;
				////printf("d\n");
			break;
			case 'x': if(currVelo /veloStep >=minVelo)
					currVelo =currVelo/veloStep;
				////printf("%lf\n",currVelo );
				legRot =0;
				supLegRot =0;
				legStf=0;
				supLegStf=0;////printf("x\n");
 			break;
 			case 'l': state=lipm_start(bot,comm);
 				  ////printf("l\n");
 				  continue;
 			case 'q': //////printf("ENCIRCLING LEFT\n");
 			
 				if(currVelo/veloStep>minVelo)
				{	currVelo =currVelo/veloStep;
					supLegRot=0;
 				  	legRot=0;
 				  	supLegStf=0;
 				  	legStf=0;
					break;
				}	
				else
				{	
				  if(flag==1)
				  {	
				  //	////printf("3rd if \n");
				  	
				  	supLegRot=0;
 				  	legRot=0;
 				  	supLegStf=0;
 				  	legStf=0;
 				  
				  	state = dribble(state[YR],state[Y],state[YV],currVelo,state[ZR],state[Z],-state[ZV],state[ZV],ZMAX,state[RR],legRot,state[R],supLegRot,state[SR],legStf,state[S],supLegStf,bot.leg[flag],bot.leg[1-flag],&comm);
				  	flag=1-flag;	
				  }
				  state = turn2( state[YR], state[Y], state[YV], currVelo, state[ZR], state[Z], -state[ZV], state[ZV], 70, 30, 20, 0, -10, 0, 0, bot.leg[0], bot.leg[1], &comm);
				  state = turn2( state[YR], state[Y], state[YV], currVelo, state[ZR], state[Z], -state[ZV], state[ZV], 70, 30, -20, state[RR], 0, state[R], 0, bot.leg[1], bot.leg[0], &comm); 
				    	continue;
				}
				////printf("q\n");
 			case 'e': 
 				 if(currVelo/veloStep>minVelo)
				{
					currVelo =currVelo / veloStep;
					supLegRot=0;
 				  	legRot=0;
 				  	supLegStf=0;
 				  	legStf=0;
					break;
				}
				else
				{	
				  if(flag==0)
				  {	
				  //	////printf("3rd if \n");
				  	
				  	supLegRot=0;
 				  	legRot=0;
 				  	supLegStf=0;
 				  	legStf=0;
 				  
				  	state = dribble(state[YR],state[Y],state[YV],currVelo,state[ZR],state[Z],-state[ZV],state[ZV],ZMAX,state[RR],legRot,state[R],supLegRot,state[SR],legStf,state[S],supLegStf,bot.leg[flag],bot.leg[1-flag],&comm);
				  	flag=1-flag;	
				  }
				  state = turn2( state[YR], state[Y], state[YV], currVelo, state[ZR], state[Z], -state[ZV], state[ZV], 70, 30, 20, 0, -10, 0, 0, bot.leg[1], bot.leg[0], &comm);
				  state = turn2( state[YR], state[Y], state[YV], currVelo, state[ZR], state[Z], -state[ZV], state[ZV], 70, 30, -20, state[RR], 0, state[R], 0, bot.leg[0], bot.leg[1], &comm); 
				    	continue;
				}


				case 'k': 
						//printf("\n\n\n\n\n\nKICK*************************************\n\n\n\n\n\n");
						if(currVelo*veloStep <=maxVelo/pow(veloStep,2))
					{
					currVelo =currVelo*veloStep;
					supLegRot=0;
 				  	legRot=0;
 				  	supLegStf=0;
 				  	legStf=0;
					break;
					}
				else
				{
				state=kick(state[YR],state[Y],state[YV],currVelo,state[ZR],state[Z],-state[ZV],state[ZV],ZMAX,state[RR],legRot,state[R],supLegRot,state[SR],legStf,state[S],supLegStf,bot.leg[flag],bot.leg[1-flag],&comm);
			flag = 1- flag;
				continue;
				}	    
				////printf("e\n");	
			default:
			break;
		};
		////printf("Flag %d\n",flag);
		////printf("cVelo %lf\n",currVelo );
		state = dribble(state[YR],state[Y],state[YV],currVelo,state[ZR],state[Z],-state[ZV],state[ZV],ZMAX,state[RR],legRot,state[R],supLegRot,state[SR],legStf,state[S],supLegStf,bot.leg[flag],bot.leg[1-flag],&comm);
		flag = 1- flag;
		legRot =0;
		supLegRot =0;
		////printf("State%lf\n",state[YV] );
		
//		sleep(1);
//		cvWaitKey(1);
	}















// 	while(TRUE)
// 	{
// 		state2[YRIN]=state[YR];
// 		state2[YIN]=state[Y];
// 		state2[YVIN]=state[YV];
// 		state2[ZRIN]=state[ZR];
// 		state2[ZIN]=state[Z];
// 		state2[ZVIN]=state[ZV];
// 		state2[FLAG]=flag;
		
// 		tempFlag++;
// 		// read_walk_data();
		
// 		// pthread_mutex_lock(&mutex_walkflag);
// 		// Flag_moving_local=Flag_moving;
// 		// pthread_mutex_unlock(&mutex_walkflag);


// 		pthread_mutex_lock(&mutex_walkstr);
// 		WalkStructure local = walkstr;
// 		walkstr.isFresh = false;
// 		pthread_mutex_unlock(&mutex_walkstr);
// 		////printf("[walk] running\n");

// 		r = local.r;
// 		theta = local.theta;
// 		//check for init/walk haere
// 		//if(localbodycommand==WALK){then walk} else if ==INIT initialise and  readwait till flagmoving is one
// 		if(local.isFresh)
// 		{
// 			////printf("YO\n");	
// 			if(local.command==WALK)
// 			{	
			
// //				////printf("``````````````````````````````````````~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`````````````````````````````````\n[Walk] Received Command WALK r=%lf theta=%lf\n",localwalkstr.r,localwalkstr.theta);
// 				double walk_dist1=0,walk_dist2=0;
				
// //				////printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n*********************RECEIVED*****************************8 command WALK r-%lf theta-%lf\n",r,theta);
// 				state2=p_walk(state2[FLAG],state2[YRIN],state2[YIN],state2[YVIN],state2[ZRIN],state2[ZIN],state2[ZVIN],r,theta,&bot,&comm);
// 				flag=state2[FLAG];
// 				state[Y]=state2[YIN];
// 				state[YR]=state2[YRIN];
// 				state[YV]=state2[YVIN];
// 				state[Z]=state2[ZIN];
// 				state[ZR]=state2[ZRIN];
// 				state[ZV]=state2[ZVIN];

// 				int breakflag=0;
			
// 				pthread_mutex_lock(&mutex_changewalkflag);
// 				if(global_CHANGEWALKFLAG==1)
// 				{
// 					//endwalk(flag,state[YR],state[Y],state[ZR],state[Z]);
// 					global_CHANGEWALKFLAG=0;
// 					breakflag=1;	
// 				}
// 				pthread_mutex_unlock(&mutex_changewalkflag);
// 				if(breakflag==1){break;}
						
// 				// stopWalk();
// 			}//if command==WALK
// 			else if(local.command==ENCIRCLE)
// 			{
// 				if(theta<0 && flag==1)
// 				{
// 					state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
// 					flag=1-flag;
// 				}
// 				else if(theta>0	&&flag==0)
// 				{
// 					state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
// 					flag=1-flag;
// 				}
// 				state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,20,0,-10,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
// 				flag=1-flag;
// 				state = turn2(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,-10,state[RR],0,state[R],0,bot.leg[flag],bot.leg[1-flag],&comm);
// 				flag=1-flag;
// 				pthread_mutex_lock(&mutex_walkstr);
// 				walkstr.mm.theta=90;
// 				walkstr.mm.theta2=-90;
// 				walkstr.mm.r=100;
// 				walkstr.mm.updated=1;
// 				pthread_mutex_unlock(&mutex_walkstr);
// 				// stopWalk();
// 			}
// 			else
// 			{
// 				////printf("GENORAI!!! Unknown value of walk command\n");
// 				// sleep(1);
// 			}
// 		}
// 		else
// 		{
// 		//	////printf("DRIBBLE\n");
// 		//	////printf("\nin else....walk thread \n");

// 			state=turn(state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],70.0,30,0,0,0,0,bot.leg[flag],bot.leg[1-flag],&comm);
// 			flag=1-flag;
// 		//	////printf("\ngen..1\n");
// 			pthread_mutex_lock(&mutex_walkstr);
// 			walkstr.mm.theta=0;
// 			walkstr.mm.r=0;
// 		//	////printf("\ngen..2\n");
// 			walkstr.mm.updated=1;
// 			walkstr.mm.theta2=0;
// 		//	////printf("\ngen..3\n");
// 			pthread_mutex_unlock(&mutex_walkstr);
// 		 	//usleep(20000);
// 		 	//continue;
// 		}
// 	}//while ends
// //	////printf("***\n");
	cvWaitKey();
}	




	
