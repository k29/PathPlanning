#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <ftdi.h>
#include <string.h>
#include <inttypes.h>
#include "communication.h"

typedef byte uint8_t;

enum MotorType { EX106, RX64, MX64, MX106, RX28};

struct ComplianceControl{
	int margin;
	int slope;
	int punch;
};

struct PidControl{
	int gainP;
	int gainI;
	int gainD;
	int punch;
};

union MotorControl{
	ComplianceControl compliance;
	PidControl pid;
};

enum MotorControlType {COMPLIANCE, PID};
	 
class Motor{
private: 
	static const int DEFAULT_ID = -1;
	static const int BROADCAST_ID = 0xfe;
	MotorType motorType; 
	int motorID;
	int cwAngleLimit;
	int ccwAngleLimit;
	MotorControl motorControl;
	bool motorControlType;
	Communication* comm;
//	bool networkedMotor; 
	int errorNo;
	bool slave;
	int offset;

	int motorWrite(int location, int dataLength, byte data[], int id);
	int motorRead(int location, int nBytes, byte *data, int id);
	int checkGoalPosition(int goalPos);
	
public:
	/* Accepts ID as a parameter and sets the ID in the motor structure as well as generates a packet for setting the ID on the motor if the networked flag is not set. packet generated is at braodcast ID. If more than one motor is connected all motors will be set to the same ID*/
	int setID(int id);
	/* Sets the ID of motor to that saved in the structure. Packet is sent at broascast ID and hence if more than one motor id connected all motor will be set to same ID */
	int setID();
	/* gets the ID from all motors connected to motor serial port. If more than one motor is connected function may return ID of first motor to respond or error */ 
	int getID();
	/* sets the serial line and motor baud to given value and also changes FTDI line to specified baud*/
	int setBaud(int baud);
	/* sets the serial line and motor baud to baudrate saved in the structure.*/
	int setBaud();
	/*returns motor baud rate*/
	int getBaud();
	int setGoalPosition(int goalPos);
	int getGoalPosition();
	int setDriveMode(int dMode);
	int getDriveMode();
	int setMovingSpeed(int movSpeed);
	int getMovingSpeed();
	//Normal ones(soft)
	int setCWAngleLimit(int angLimit);
	int getCWAngleLimit();
	int setCCWAngleLimit(int angLimit);
	int getCCWAngleLimit();
	//Hardware ones
	int setCWAngleLimitHard(int angLimit);
	int getCWAngleLimitHard();
	int setCCWAngleLimitHard(int angLimit);
	int getCCWAngleLimitHard();
	int torqueEnable();
	int torqueDisable();
	int getTorqueStatus();
	int setComplianceMargin(int margin);
	int setComplianceSlope(int slope);
	int setCompliancePunch(int punch);
	int getComplianceMargin();
	int getComplianceSlope();
	int getCompliancePunch();
	int setGainP(int p);
	int setGainI(int i);
	int setGainD(int d);
	int getGainP();
	int getGainI();
	int getGainD();
	int ledOn();
	int ledOff();
	int ledStatus();
	int getPresentPosition();
	int getPresentSpeed();
	int getPresentLoad();
	double getSensedCurrent();
	
	int ping();
	int setGoalPositionSync(int golPos);
	int setMovingSpeedSync(int movSpeed);
	int setGainPSync(int p);
	inline int setIDSoft(int id)
	{
		this->motorID = id;
	}
	Motor(MotorType motorType, int id, Communication* comm,int offsetValue = 0);
	int slaveMode();
	int setOffset(int offsetValue);
};
#endif
