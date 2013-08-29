#include "motor.h"
#include "communication.h"
#include <iostream>
using namespace std;

Motor::Motor(MotorType motorType, int id, Communication* comm, int offsetValue)
{
	this->comm = comm;
	this->motorType = motorType;
	if(motorType == EX106)
	{
		motorControlType = COMPLIANCE;
		motorControl.compliance.slope = 32;
		motorControl.compliance.margin = 0;
		motorControl.compliance.punch = 32;
		cwAngleLimit = 0;
		ccwAngleLimit = 4095;
	}
	else if(motorType == RX64 || motorType == RX28)
	{
		motorControlType = COMPLIANCE;
		motorControl.compliance.slope = 32;
		motorControl.compliance.margin = 0;
		motorControl.compliance.punch = 32;
		cwAngleLimit = 0;
		ccwAngleLimit = 1023;
	}
	else if(motorType == MX64 || motorType == MX106)
	{
		motorControlType = PID;
		motorControl.pid.gainP = 32;
		motorControl.pid.gainI = 0;
		motorControl.pid.gainD = 0;
		motorControl.pid.punch = 0;
		cwAngleLimit = 0;
		ccwAngleLimit = 4095;
	}
	
	motorID = id;
//	this->networkedMotor = networkedMotor;
	slave = false;
	offset = offsetValue;
	return;
}

int Motor::motorWrite(int location, int dataLength, byte data[], int id = DEFAULT_ID)
{
	byte data_D[dataLength+1];
	data_D[0] = location;
	for(int i =0;i<dataLength;i++)
	data_D[1+i]=data[i];
	return comm->motorSend(0x03,dataLength+1,data_D,id);
}

int Motor::motorRead(int location, int nBytes, byte data[], int id = DEFAULT_ID)
{
	int i =0;
	byte data_D[] = {(byte)location, (byte)nBytes};
	comm->motorSend(0x02,0x02,data_D,id);
	return comm->motorReceive(data);	
}

int Motor::setID(int id)
{
	int prevID = motorID;
	motorID = id;
	byte* data;
	if(setID())
	{
		motorID = prevID;
		return EXIT_FAILURE;
	}
	else
		return EXIT_SUCCESS;
}

int Motor::setID()
{
	byte data[] = {motorID};
	return motorWrite(0x03, 0x01, data, BROADCAST_ID);
}

int Motor::getID()
{
	byte data[1];
	motorRead(0x03, 1, data);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setBaud(int baud)
{
	byte data[] = {baud};
	if(motorWrite(0x04,0x01,data))
	{
		return EXIT_FAILURE;
	}
	else
		return EXIT_SUCCESS;
}

int Motor::getBaud()
{
	byte data[1];
	motorRead(0x04, 0x01, data, 0xfe);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}


int Motor::checkGoalPosition(int goalPos)
{
	int max = cwAngleLimit, min;
	if(max < ccwAngleLimit)
	{
		max = ccwAngleLimit;
		min = cwAngleLimit;
	}
	else
		min = ccwAngleLimit;
	if(goalPos < min || goalPos > max)
	{
		cout<<"Out of bound values to Motor "<< motorID <<"\n";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int Motor::setGoalPositionSync(int golPos)
{
	if(checkGoalPosition(golPos+offset)==EXIT_FAILURE || slave == true)
		return EXIT_FAILURE;
	
	byte goal[2];
	goal[0] = (golPos + offset)%256;
	goal[1] = (golPos + offset)/256;
	if(comm->addSyncWrite(0x1e, 2, goal, motorID))
		return EXIT_FAILURE;
	else
		return EXIT_SUCCESS;
}

int Motor::setMovingSpeedSync(int movSpeed)
{
	if(slave == true)
		return EXIT_FAILURE;
	byte speed[2];
	speed[0] = movSpeed%256;
	speed[1] = movSpeed/256;
	if(comm->addSyncWrite(0x20, 0x02, speed, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::setGainPSync(int p)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(slave==true)
		return EXIT_FAILURE;
		
		if(comm->addSyncWrite(0x1c, 0x01, (byte*)&p, motorID))
			return EXIT_FAILURE;
		else 
			return EXIT_SUCCESS;
	}
	else 
		return EXIT_FAILURE;
}

int Motor::setGoalPosition(int golPos)
{
	if(checkGoalPosition(golPos+offset)==EXIT_FAILURE)
		return EXIT_FAILURE;
	byte goal[2];
	goal[0] = (golPos + offset)%256;
	goal[1] = (golPos + offset)/256;
	if(motorWrite(0x1e, 0x02, goal, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::getGoalPosition()
{
	byte data[2];
	motorRead(0x1e, 0x02, data, motorID);
	int pos = data[0] + (data[1]<<8) - offset;
	cout<<"\n"<<pos<<"\n";
	return pos;
}

int Motor::setDriveMode(int dMode)
{
	if(motorWrite(0x0a, 0x01, (byte*)&dMode, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}	
}

int Motor::getDriveMode()
{
	byte data[1];
	motorRead(0x1a, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setMovingSpeed(int movSpeed)
{
	byte speed[2];
	speed[0] = movSpeed%256;
	speed[1] = movSpeed/256;
	if(motorWrite(0x20, 0x02, speed, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::getMovingSpeed()
{
	byte data[2];
	motorRead(0x20, 0x02, data, motorID);
	int speed = data[0] + (data[1]<<8);
	cout<<"\n"<<speed<<"\n";
	return speed;
}

int Motor::setCWAngleLimit(int angLimit)
{
	cwAngleLimit = angLimit;
}

int Motor::setCCWAngleLimit(int angLimit)
{
	ccwAngleLimit = angLimit;
}

int Motor::getCWAngleLimit()
{
	return cwAngleLimit;
}

int Motor::getCCWAngleLimit()
{
	return ccwAngleLimit;
}

int Motor::setCWAngleLimitHard(int angLimit)
{
	byte lim[2];
	lim[0] = angLimit%256;
	lim[1] = angLimit/256;
	if(motorWrite(0x06, 0x02, lim, motorID))
		return EXIT_FAILURE;
	else
	{
		cwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}	
}

int Motor::getCWAngleLimitHard()
{
	byte data[2];
	motorRead(0x06, 0x02, data, motorID);
	int angleLimit = data[0] + (data[1]<<8);
	cout<<"\n"<<angleLimit<<"\n";
	return angleLimit;
}

int Motor::setCCWAngleLimitHard(int angLimit)
{
	byte lim[2];
	lim[0] = angLimit%256;
	lim[1] = angLimit/256;
	if(motorWrite(0x08, 0x02, lim, motorID))
		return EXIT_FAILURE;
	else
	{
		ccwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}
}

int Motor::getCCWAngleLimitHard()
{
	byte data[2];
	motorRead(0x08, 0x02, data, motorID);
	int angleLimit = data[0] + (data[1]<<8);
	cout<<"\n"<<angleLimit<<"\n";
	return angleLimit;
}

int Motor::torqueEnable()
{
	byte data[] = {0x01};
	if(motorWrite(0x18, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::torqueDisable()
{
	byte data[] = {0x00};
	if(motorWrite(0x18, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::getTorqueStatus()
{
	byte data[1];
	motorRead(0x18, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setComplianceMargin(int margin)
{
	if(motorType == EX106 || motorType == RX28)
	{
		if(motorWrite(0x1a, 0x01, (byte*)&margin, motorID) && motorWrite(0x1b, 0x01, (byte*)&margin, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.compliance.margin = margin;
			return EXIT_SUCCESS;
		}
	}
	else 
		return EXIT_FAILURE;		
}

int Motor::setComplianceSlope(int slope)
{
	if(motorType == EX106 || motorType == RX28)
	{
		if(motorWrite(0x1c, 0x01, (byte*)&slope, motorID) && motorWrite(0x1d, 0x01, (byte*)&slope, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.compliance.slope = slope;
			return EXIT_SUCCESS;
		}
	}
	else
		return EXIT_FAILURE;
}

int Motor::setCompliancePunch(int punch)
{
	byte pun[2];
	pun[1] = punch%256;
	pun[2] = punch/256;
	if(motorType == EX106 || motorType == RX28)
	{
		if(motorWrite(0x30, 0x02, pun, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.compliance.punch = punch;
			return EXIT_SUCCESS;
		}
	}
	else
		return EXIT_FAILURE;
}

int Motor::getComplianceMargin()
{
	byte data[1];
	motorRead(0x1a, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getComplianceSlope()
{
	byte data[1];
	motorRead(0x1c, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getCompliancePunch()
{
	byte data[1];
	motorRead(0x30, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setGainP(int p)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1c, 0x01, (byte*)&p, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainP = p;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;		
}

int Motor::setGainI(int i)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1b, 0x01, (byte*)&i, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainI = i;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;	
}

int Motor::setGainD(int d)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1a, 0x01, (byte*)&d, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainD = d;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;	
}

int Motor::getGainP()
{
	byte data[1];
	motorRead(0x1c, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getGainI()
{
	byte data[1];
	motorRead(0x1b, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getGainD()
{
	byte data[1];
	motorRead(0x1a, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}


int Motor::ledOn()
{
	byte data[] = {0x01};
	if(motorWrite(0x19, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::ledOff()
{
	byte data[] = {0x00};
	if(motorWrite(0x19, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}	
}

int Motor::ledStatus()
{
	byte data[1];
	motorRead(0x19, 0x01, data, motorID);
	cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getPresentPosition()
{
	byte data[2];
	motorRead(0x24, 0x02, data, motorID);
	int pos = data[0] + (data[1]<<8) -offset;
	cout<<"\n"<<pos<<"\n";
	return pos;
}

int Motor::getPresentSpeed()
{
	byte data[2];
	motorRead(0x26, 0x02, data, motorID);
	int speed = data[0] + (data[1]<<8);
	cout<<"\n"<<speed<<"\n";
	return speed;
}

int Motor::getPresentLoad()
{
	byte data[2];
	motorRead(0x28, 0x02, data, motorID);
	int load = data[0] + ((data[1]&3)<<8);
	cout<<"Load:"<<motorID<<" "<<load<<"\n";
	return load;
}

int Motor::ping()
{
	byte* data = NULL;
	comm->motorSend(0x01, 0x00, NULL, motorID);
	if(comm->motorReceive(data))
	{
		cout<<"Failed "<<motorID<<"\n";
		return EXIT_FAILURE;
	}
	else
	{
		cout<<"Success "<<motorID<<"\n";
		return EXIT_SUCCESS;
	}
}

int Motor::slaveMode()
{
	slave = true;
	cout<<"Slave Mode set for ID " << motorID << endl;

}

int Motor::setOffset(int offsetValue)
{
	offset = offsetValue;
}

double Motor::getSensedCurrent()
{
	byte data[2];
	motorRead(0x44, 0x02, data, motorID);
	int CURRENT = data[0] + (data[1]<<8);
	double sensedCurrent = 0.045* (CURRENT-2048);
	//cout<<"\n"<<sensedCurrent<<"\n";
	return sensedCurrent;
}