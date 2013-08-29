#ifndef _COMMON_DEFS_
#define _COMMON_DEFS_


#include <inttypes.h>

#define byte uint8_t
// #define rad2deg(x) ((x)*180.0/3.1415926535)
// #define deg2rad(x) ((x)*3.1415926535/180.0)
#define sgn( a ) ( ((a) > 0) ? (1) : ( ((a) < 0) ? -1 : 0 ) )

enum {LEFT=0,RIGHT=20};
enum legMotors {FOOT_ROLL=0,FOOT_PITCH,KNEE1,KNEE2,HIP_PITCH,HIP_ROLL,HIP_YAW,MOTOR_COUNT};


#endif

