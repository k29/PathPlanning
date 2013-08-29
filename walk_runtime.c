#define MASS (5.733)
#define HEIGHT 0.82
#define BREADTH 0.29
#define WIDTH 0.10
#define PLOT 0

#define MOTOR_DATA 0
#define IMU_DATA 0
#define FT4232 "FTSBY5CA"//"FTSC152E"//"FTSC152E"
#define NOM 28                                  //Current number of Motors
#define LEN_SYNC_WRITE ((NOM*3)+4) 		//Total Lenght Of Packet
#define INST_SYNC_WRITE     0x83
#define SMOOTHSTEP(x) ((x) * (x)*(3-2*(x)))
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#define D2R (3.14/180)
#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A4007sgE" //"A900fDpz"////"A4007sgE" "A900fDpz"//"A4007sgE" //"A900fDpz" // "A4007sgE"//"A900fDpz" //"A700eSSZ"//"A900fDhp" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8	  A700eST2
#define serialimu_walk "A8004Yt8" //"A8008c3o" 
#define TIMESCALE 1

//#define TESTING
#include <math.h>
#include <stdio.h>
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

#ifdef TESTING

#include <math.h>
#include <stdio.h>
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



#endif



long int COUNTER=0;
long int COM_INC=0;
int COUNT;
char comfile[50]="\0";
char leftfile[50]="\0";
char rightfile[50]="\0";
char expect_com[50]="\0";
char hipfile[50]="\0";
char normalrctn[50]="\0";
long time_start, time_end,walk_prev_frame,walk_curr_frame;
struct timeval tv;
 
typedef uint8_t byte;
byte ignore_ids[]={10,15,16,17,18,30};  // enter ids to ignore :)
float q[5];
float mval[5];
	
float output[20];

	
///////////IMU DECLARATIONS//////////////////////////
/// GEN DECLARATIONS
int imu_init=0;

/// BASE VALUES 
float base_angle[3]={0};
float base_acc[3]={0};
float base_gyro[3]={0};
float gravity_magnitude=0;
float gravity[3]={0};

/// SENSOR DATA
float imu_angle[3]={0};  
float imu_acc[3]={0};
float imu_gyro[3]={0};


/// INFERRED
float current_angle[3]={0};
float current_acc[3]={0};
float held_angle[3]={0};
float gyro_angle[3]={0};
float prev_gyro[3]={0};
float ang_acc[3]={0};

/// NORMAL FORCE
float magnitude[3]={0};
float location[3]={0};

int boundaries_size=0, local_boundaries_size=0;
float boundaries[MAX_BOUNDARIES][3]={0};
float local_boundaries[MAX_BOUNDARIES][3]={0};

/////////////////////////////////////////////////

struct ftdi_context imu;

int /*flag_imu=1,*/temp_x;
 
int gflag=0;
 
FILE *f;
FILE *f1;
int fd,fb,imu_t;
struct ftdi_context ftdic1,ftdic2;  //ftdic1=usb2dyn   ftdic2=bluetooth

byte tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
byte last_tx_packet[128]={0};
typedef struct 
{
	byte ID;
	byte LB;
	byte HB;
}motor_data;

typedef struct 
{
	byte num_motors;
	byte address;
	motor_data motorvalues[NOM];
	byte checksum;
}frame1;

typedef struct
{
	byte header[2];
	int noframes;
	frame1 frames[720];
//	byte endbit[2];
}move;
//////Dhairya & Apoorv's GUI Comm defs//////
byte* gui_packet;
int element,noat,offset_status=0;
unsigned int length;

typedef struct
{
	float mat[4][4];
}matrix;

typedef struct
{
	matrix rotmat;
	float a;
	float d;
	float theta;
	float alpha;
}fk_frame;

typedef struct 
{
	float x;
	float y;
	float z;
}coordinates1;

typedef struct
{
	long int nof;
	coordinates1 coods[6000];
	coordinates1 left[6000];
	coordinates1 right[6000];
}zmp_file;

zmp_file zmp;

//////////////////// WALKING DEFINITIONS///////////////////////////

float Y_END=0,YR_END=0,Z_END=0,ZR_END=0,THETA_END=0,THETAR_END=0;
float com_y;
float past_com_z,com_z,future_com_z;
int dsp=1;
int touchdown=1;
float stcr=0;
float tgive=1;
float tipOver=45;
float r_leg_coods[3];

typedef struct
{
	/////// X-Lift Control /////
	float x_start;
	float x_end;
	float lift;
	float height;
	float freq;
	float displacement;	
	float phase;
}x_lift;

typedef struct
{
	/////// X-Land Control /////
	//float dec_time=0.75;	
	float x_land_start;//1-x_end    
	float x_land_increase_end;
	float x_land_decrease_start;
	float x_land_end;
	float x_land_freq;
	float x_land_displacement;
	float x_land_phase;
	float x_land_dec;
}x_land;

typedef struct 
{
	/////// X-Load Control /////
	float x_load_start;
	float x_load_increase_end;
	float x_load_decrease_start;
	float x_load_end;
	float x_load_displacement;//x_land_displacement;
	float x_load_phase;//-1.57;
	float x_load_freq;//3.14;
	float x_load_dec;
	//float x_load_window=0.05;
}x_load;

typedef struct
{
	x_lift lift;
	x_land land;
	x_load load;
	
}walk_x;

typedef struct
{
	
	/////// Z-Shift Control /////
	float peak_start;
	float peak_max;//0.5
	float peak_stay;//0.75
	float peak_end;
	float zmax;
	float zfreq;
	float zphase;
	float zdisplacement;
}walk_zshift;

typedef struct
{
	/////// Y Control /////
	float y_start;
	float y_end;
	float y_0;
	float y_move;
}walk_y;

typedef struct
{
	/////// Z Control /////
	float z_start;
	float z_0;
	float z_move;
	float z_end;
}walk_z;

typedef struct
{
	 ///////Separation control////
    	float seperation;
 	/////COMPLIANCE//
	float cptime;
}walk_misc;

typedef struct
{
	//////Turn Control////
	float turn_start;
	float turn_end;
}walk_turn;

typedef struct
{
	/////// COM Trajectory //////
	//Z Movement
	float z_com_start;
	float z_com_peak;
	float z_com_stay;
	float z_com_end;
	float z_com_max;
	float z_static_inc;
	
	//Y Movement
	float y_com_start;
	float y_com_end;
}walk_com;

typedef struct
{
	walk_x x;
	walk_zshift z_shift;
	walk_y y;
	walk_z z;
	walk_turn turn;
	walk_misc misc;
	walk_com com_trajectory;
}leg_config;
	
typedef struct
{
	float time;
	float exp_time;
	int fps;
	leg_config left;
	leg_config right;
}walk_config; 

walk_config walk;

	
////////////////////////////////////////////////////////////////////
///////////////////// FORWARD KINEMATICS, INVERSE KINEMATICS , COM GENERATION , ZMP CALCULATION DEFINITIONS
float left_joints[7][3],right_joints[7][3];
float left_com[3]={0},right_com[3]={0};
float com[3]={0},rot_com[3]={0},rot_hip[3]={0},rot_rleg[3],hip[3]={0};
int offset[33]={0};
int target_left[6],target_right[6];
float STEP=20;
	
/////////////////////////////////////////////////////////////////////
float* generateik(float x,float y,float z,int i );
float* generatespline(int gn,float gx[],float gy[]);
float scurve(float in,float fi,float t, float tot);
void initialize_acyut();
//void imucal();
//void get_imu_data();

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

int kbhit2()
{
	int	 count = 0;
	int	 error;
	static struct termios	Otty, Ntty;

	tcgetattr(STDIN_FILENO, &Otty);
	Ntty = Otty;

	Ntty.c_lflag &= ~ICANON; /* raw mode */
	Ntty.c_cc[VMIN] = CMIN; /* minimum chars to wait for */
	Ntty.c_cc[VTIME] = CTIME; /* minimum wait time	 */

	if (0 == (error = tcsetattr(STDIN_FILENO, TCSANOW, &Ntty)))
	{
		struct timeval	tv;
		error += ioctl(STDIN_FILENO, FIONREAD, &count);
		error += tcsetattr(STDIN_FILENO, TCSANOW, &Otty);
/* minimal delay gives up cpu time slice, and
* allows use in a tight loop */
		tv.tv_sec = 0;
		tv.tv_usec = 10;
		select(1, NULL, NULL, NULL, &tv);
	}
	return (error == 0 ? count : -1 );
}


int ipurge(int choose,int *termios_pointer, struct ftdi_context *ftdi_pointer)
{
	switch(choose)
	{
		case 0 : return ftdi_usb_purge_buffers(ftdi_pointer);
			break;
		case 1 : return tcflush(*termios_pointer,TCIOFLUSH);
			break;
		default : return -99999;
	}
}

int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	int i=0;
	//gettimeofday(&tv,NULL); gets the current system time into variable tv  
	//time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;

	//f=fopen("check","a");
	//if(f!=NULL)
	{
/*		for(i=0;i<length;i++)*/
/*		{*/
/*			printf("%x\t",packet[i]);*/
/*			//fprintf(f,"%02x",packet[i]);*/
/*			//fprintf(f,"\n");*/
/*		}*/
/*		printf("\n");*/
/*			*/
		//fclose(f);
	}
	
/*	pthread_mutex_lock(&mutex_Command);*/
/*	pthread_mutex_lock(&mutex_cam_in);*/
/*	pthread_mutex_lock(&mutex_GCData);*/
/*	pthread_mutex_lock(&mutex_curwalkpos);*/
/*	pthread_mutex_lock(&mutex_pack);*/
/*	pthread_mutex_lock(&mutex_finalpath);*/
	
	/*for(i=0;i<length;i++)
		{
			printf("%x\t",packet[i]);
			//fprintf(f,"%02x",packet[i]);
			//fprintf(f,"\n");
		}
		printf("\n");
		
	
	*/	
	int retjfasdhf=0;
	switch(choose)
	{
		case 0: retjfasdhf=ftdi_write_data(ftdi_pointer,packet,length);
			break;
		case 1 :retjfasdhf=write(*termios_pointer,packet,length);
			break;
		default : retjfasdhf=-99999;
	}
	
/*	pthread_mutex_unlock(&mutex_Command);*/
/*	pthread_mutex_unlock(&mutex_cam_in);*/
/*	pthread_mutex_unlock(&mutex_GCData);*/
/*	pthread_mutex_unlock(&mutex_curwalkpos);*/
/*	pthread_mutex_unlock(&mutex_pack);*/
/*	pthread_mutex_unlock(&mutex_finalpath);*/
	return retjfasdhf;
}

int iread(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	switch(choose)
	{
		case 0: return ftdi_read_data(ftdi_pointer,packet,length);
			break;
		case 1 :return read(*termios_pointer,packet,length);
			break;
		default : return -99999;
	}
}


int compare_float(float f1, float f2)
{
  float precision = 1.0/60;
  if (((f1 - precision) < f2) && 
      ((f1 + precision) > f2))
   {
    return 1;
   }
  else
   {
    return 0;
   }
}

/*
int get_lb( int word )
{
  unsigned short temp;
  temp = word % 256;
  return (int)temp;
}

int get_hb( int word )
{
  unsigned short temp;
  temp = word / 256;
  return (int)temp;
}*/
void print_matrix(matrix a)
{	
	int i=0,j=0;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			printf("%lf\t",a.mat[i][j]);
		}
		printf("\n");
	}
}
	
matrix multiply(matrix a, matrix b)
{
	matrix ans;
	int i=0,j=0;
	
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			ans.mat[i][j]=a.mat[i][0]*b.mat[0][j]+a.mat[i][1]*b.mat[1][j]+a.mat[i][2]*b.mat[2][j]+a.mat[i][3]*b.mat[3][j];
		}
	}
	
	return ans;
}

void identity_mat(matrix *a)
{
	int i,j;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			a->mat[i][j]=1;
			else
			a->mat[i][j]=0;
		}
	}
}
	
void form_matrix(fk_frame *a)
{
	a->rotmat.mat[0][0]=cos(a->theta*3.14/180);
	a->rotmat.mat[0][1]=-sin(a->theta*3.14/180)*cos(a->alpha*3.14/180);
	a->rotmat.mat[0][2]=sin(a->theta*3.14/180)*sin(a->alpha*3.14/180);
	a->rotmat.mat[0][3]=a->a*cos(a->theta*3.14/180);
	
	a->rotmat.mat[1][0]=sin(a->theta*3.14/180);
	a->rotmat.mat[1][1]=cos(a->theta*3.14/180)*cos(a->alpha*3.14/180);
	a->rotmat.mat[1][2]=-cos(a->theta*3.14/180)*sin(a->alpha*3.14/180);
	a->rotmat.mat[1][3]=a->a*sin(a->theta*3.14/180);
	
	a->rotmat.mat[2][0]=0;
	a->rotmat.mat[2][1]=sin(a->alpha*3.14/180);
	a->rotmat.mat[2][2]=cos(a->alpha*3.14/180);
	a->rotmat.mat[2][3]=a->d;

	a->rotmat.mat[3][0]=0;
	a->rotmat.mat[3][1]=0;
	a->rotmat.mat[3][2]=0;
	a->rotmat.mat[3][3]=1;
	
}

inline int read_position(int mot_no)
{
	//printf("READING MOTOR NO. %d\n",mot_no);
	
	//gettimeofday(&tv,NULL); gets the current system time into variable tv  */
	//time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;

	int try_=0;
	int noat=0;
	int start=0;
	int exit=0;
	int data_not_rcv=1;
	int i=0;
	byte read1;
	byte snd_packet[8];
	byte rcv_packet[6];
	byte chksum=0xff;

	snd_packet[0]=0xff;
	snd_packet[1]=0xff;
	snd_packet[2]=mot_no;
	snd_packet[3]=0x04;
	snd_packet[4]=0x02;
	snd_packet[5]=0x24;
	snd_packet[6]=0x02;
	snd_packet[7]=0xff;

	for(i=2;i<7;i++)
	snd_packet[7]-=snd_packet[i];
	
	for(try_=0;try_<1;try_++)
	{
		start=0;
		noat=0;
		//if(ftdi_write_data(&ftdic1,snd_packet,8)==8)
		if(iwrite(MOTOR_DATA,&fd,&ftdic1,snd_packet,8)>=0)
		{
			//gettimeofday (&tv, NULL);
			//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
			data_not_rcv=1;
			//printf("SEND TIME : %ld\n",-time_start+time_end);
			//if(ftdi_setrts(&ftdic1,1)==0)
			//printf("RTS SET 1\n");
						
			while(noat<(MOTOR_DATA==0?4:4000)&&data_not_rcv==1)
			{	
				//if(ftdi_read_data(&ftdic1,&read,1)==1&&read==0xff)
				if(iread(MOTOR_DATA,&fd,&ftdic1,&read1,1)>=0&&read1==0xff)
				start++;
				else
				{
					start=0;
					noat++;
				}
				
				if(start==2)
				{
					start=0;
					data_not_rcv==0;
					//if(ftdi_read_data(&ftdic1,rcv_packet,6)>0)
					if(iread(MOTOR_DATA,&fd,&ftdic1,rcv_packet,6)>0)
					{
						chksum=0xff;						
						for(i=0;i<5;i++)
						{
							//printf("%d\t",rcv_packet[i]);
							chksum-=rcv_packet[i];
						}			
						//printf("CHKSUM %d C %d\n",rcv_packet[i],chksum);			
						if(chksum==rcv_packet[5]&&rcv_packet[0]==mot_no)
						{	
							//gettimeofday (&tv, NULL);
							//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
							//if(ftdi_setrts(&ftdic1,0)==0)
							//printf("RTS SET 0\n");
							//printf("MOTOR READ %d\n TIME:%ld\n",noat,time_end-time_start);
							return rcv_packet[3]+rcv_packet[4]*256;
						}
						else
						noat++;
					}
				}
			}		
		}
	}
	//if(ftdi_setrts(&ftdic1,0)==0)
	//printf("RTS SET 0\n");
	//gettimeofday (&tv, NULL);
	//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
																			
	//printf("NOT READ\t %ld\n",time_end-time_start);
	return 9999;					
					
}

int forward_kinematics(int leg,int mval[6])   
{
	
	float hip_length=100;
	int i =0;
	float cnst[6];
	float coods[7][3]={0};
	cnst[0]=1675;	
	cnst[1]=3207;
	cnst[2]=1752;
	cnst[3]=1714;
	if(leg==0)
	cnst[4]=742;
	else if(leg==1)
	cnst[4]=274;
	
	cnst[5]=1703;

	for(i=0;i<6;i++)
	{
		//printf("%d\t%d\t",i,mval[i]);
		if(i==4)
		mval[i]=(mval[i]-cnst[i])*300/1024;   /// MOTOR 4 is RX64 ..... 
		else
		mval[i]=(mval[i]-cnst[i])*300/4096;
		//printf("%f\n",mval[i]);
	}
	fk_frame frames[7];
	matrix base_frame;
	identity_mat(&base_frame);
	
	frames[0].a=0;
	frames[0].d=0;
	frames[0].alpha=90;
	frames[0].theta=-90+pow(-1,leg)*mval[5];
	
	frames[1].a=0;
	frames[1].d=50;
	frames[1].alpha=90;
	frames[1].theta=-90-mval[4];
	
	frames[2].a=180;
	frames[2].d=0;
	frames[2].alpha=0;
	frames[2].theta=90-mval[3];
	
	frames[3].a=50;
	frames[3].d=0;
	frames[3].alpha=0;
	frames[3].theta=-mval[2];
	
	frames[4].a=180;
	frames[4].d=0;
	frames[4].alpha=0;
	frames[4].theta=-mval[2];
	
	frames[5].a=25;
	frames[5].d=0;
	frames[5].alpha=90;
	frames[5].theta=-mval[1];
	
	for(i=0;i<6;i++)
	{
		form_matrix(&frames[i]);
		//print_matrix(frames[i].rotmat);
		//printf("\n\n\n\n");
		base_frame=multiply(base_frame,frames[i].rotmat);
		coods[i+1][0]=base_frame.mat[0][3];
		coods[i+1][1]=base_frame.mat[1][3];
		coods[i+1][2]=base_frame.mat[2][3];
		//printf("%f\t%f\t%f\n",coods[i][0],coods[i][1],coods[i][2]);
		//print_matrix(base_frame);
	
	}
	
	if(leg==0)
	{
		for(i=0;i<7;i++)
		{
			left_joints[i][0]=coods[i][0];
			left_joints[i][1]=coods[i][1]-hip_length/2;   
			left_joints[i][2]=coods[i][2];
			//printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
		}	
	}	
	else if(leg==1)
	{
		for(i=0;i<7;i++)
		{
			right_joints[i][0]=coods[i][0];
			right_joints[i][1]=coods[i][1]+hip_length/2;
			right_joints[i][2]=coods[i][2];
			//printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
		}
	}	
	//print_matrix(base_frame);
	return EXIT_SUCCESS;
}

float* generate_leg_com(int leg)
{	
	int i=0,j=0;
	
	float coods[7][3];
	float leg_com[3];
	float motor_mass=150;
	float tot_mass=0;	

	float bracket_mass[6];
	bracket_mass[0]=0;  /// 5-4 bracket
	bracket_mass[1]=0;  /// 4-3 bracket
	bracket_mass[2]=112;   /// 3-12 bracket
	bracket_mass[3]=0;   /// 12-2 bracket
	bracket_mass[4]=112;   /// 2-1 bracket
	bracket_mass[5]=0;   /// 1-0 bracket
	
	if(leg==0)
	{
		for(i=0;i<7;i++)
		{
			coods[i][0]=left_joints[i][0];
			coods[i][1]=left_joints[i][1];
			coods[i][2]=left_joints[i][2];
		}	
	}	
	else if(leg==1)
	{
		for(i=0;i<7;i++)
		{
			coods[i][0]=right_joints[i][0];
			coods[i][1]=right_joints[i][1];
			coods[i][2]=right_joints[i][2];
		}	
	}		
		
	///////////// MOTORS 
	for(i=0;i<7;i++)
	{
		for(j=0;j<3;j++)
		leg_com[j]+=coods[i][j]*motor_mass;
		
		tot_mass+=motor_mass;
	}
	
	//////////// BRACKETS
	for(i=0;i<6;i++)
	{
		for(j=0;j<3;j++)
		leg_com[j]+=(coods[i][j]+coods[i+1][j])/2*bracket_mass[i];
		
		tot_mass+=bracket_mass[i];
	}
	
	//////// FEET TP HERE ////////////
		

	for(j=0;j<3;j++)
	leg_com[j]=leg_com[j]/tot_mass;
	//leg==1?printf("RIGHT "):printf("LEFT  ");
	//printf("COM X:%lf\tY:%lf\tZ:%lf\n",leg_com[0],leg_com[1],leg_com[2]);	
	if(leg==0)
	{
		left_com[0]=leg_com[0];
		left_com[1]=leg_com[1];
		left_com[2]=leg_com[2];
	}	
	else if(leg==1)
	{
		right_com[0]=leg_com[0];
		right_com[1]=leg_com[1];
		right_com[2]=leg_com[2];
	}	
	return EXIT_SUCCESS;
}

int generate_com()
{
	generate_leg_com(0);
	generate_leg_com(1);

	// LEFT LEG CONTIBUTION
	
	float l_wt=1499;
	com[0]=(left_com[0]+0)*l_wt;
	com[1]=(left_com[1]+0)*l_wt;
	com[2]=(left_com[2]+0)*l_wt;
	
	// RIGHT LEG CONTRIBUTION	
	
	float r_wt=1499;
	com[0]+=(right_com[0]+0)*r_wt;
	com[1]+=(right_com[1]+0)*r_wt;
	com[2]+=(right_com[2]+0)*r_wt;
	
	// TORSO CONTRIBUTION
	float torso_wt=2734.9;
	com[0]+=121.358*torso_wt;
	com[1]+=0*torso_wt;
	com[2]+=0*torso_wt;

	com[0]=com[0]/(l_wt+r_wt+torso_wt);
	com[1]=com[1]/(l_wt+r_wt+torso_wt);
	com[2]=com[2]/(l_wt+r_wt+torso_wt);
	//printf(" COM X:%lf\tY:%lf\tZ:%lf\t",com[0],com[1],com[2]);
	return EXIT_SUCCESS;
}

int rotate_com(int leg)
{	
	FILE *f;
	
	int i=0,j=0;	
	float com_leg_frame[3];
	float hip_coods[3];
	float roll=0,pitch=0,yaw=0;
	if(leg==0)
	{
		for(i=0;i<3;i++)
		{
			com_leg_frame[i]=com[i]-left_joints[6][i];
			hip_coods[i]=-left_joints[6][i];
			r_leg_coods[i]=right_joints[6][i]-left_joints[6][i];
		}
	}
	else if(leg==1)
	{
		for(i=0;i<3;i++)
		{
			com_leg_frame[i]=com[i]-right_joints[6][i];
			hip_coods[i]=-right_joints[6][i];
			r_leg_coods[i]=left_joints[6][i]-right_joints[6][i];
		
		}
	}
	
	hip[0]=hip_coods[0];
	hip[1]=hip_coods[1]+(-50)*(1-leg)+(50)*leg;
	hip[2]=hip_coods[2];
	

	//// GETTING ANGLES FROM IMU
	//get_imu_data();
	roll=0;//-current_angle[0]*3.14/180;
	pitch=0;//-current_angle[1]*3.14/180;
	yaw=0;//current_angle[2];
	
	//printf("ROLL %f PITCH %f \t",current_angle[0],current_angle[1]);
	//// CONVERTING TO IMU FRAME
	float x=-com_leg_frame[1];
	float y=com_leg_frame[2];
	float z=-com_leg_frame[0];
	/*float roll_matrix[3][3], pitch_matrix[3][3];
	roll_matrix.mat[0][0]=1;	roll_matrix.mat[0][1]=0;	roll_matrix.mat[0][2]=0;
	roll_matrix.mat[1][0]=0;	roll_matrix.mat[1][1]=cos(roll);roll_matrix.mat[1][2]=-sin(roll);
	roll_matrix.mat[2][0]=0;	roll_matrix.mat[2][1]=sin(roll);roll_matrix.mat[2][2]=cos(roll);
	
	pitch_matrix.mat[0][0]=cos(pitch);pitch_matrix.mat[0][1]=0;	pitch_matrix.mat[0][2]=sin(pitch);
	pitch_matrix.mat[1][0]=0;	pitch_matrix.mat[1][1]=1;	pitch_matrix.mat[1][2]=0;
	pitch_matrix.mat[2][0]=-sin(pitch);pitch_matrix.mat[2][1]=0;	pitch_matrix.mat[2][2]=cos(pitch);
	
	float coods[]={-com_leg_frame[1],com_leg_frame[0],-com_leg_frame[0]}
	*/
	float rot_x=cos(pitch)*x+sin(pitch)*z;
	float rot_y=sin(roll)*sin(pitch)*x+cos(roll)*y+sin(roll)*cos(pitch)*z;
	float rot_z=-cos(roll)*sin(pitch)*x+sin(roll)*y+cos(roll)*cos(pitch)*z;
	
	rot_com[0]=-rot_z;
	rot_com[1]=-rot_x+(-50)*(1-leg)+50*leg;
	rot_com[2]= rot_y;

	float hip_x=-hip_coods[1];
	float hip_y=hip_coods[2];
	float hip_z=-hip_coods[0];
	
	float hip_rot_x=cos(pitch)*hip_x+sin(pitch)*hip_z;
	float hip_rot_y=sin(roll)*sin(pitch)*hip_x+cos(roll)*hip_y+sin(roll)*cos(pitch)*hip_z;
	float hip_rot_z=-cos(roll)*sin(pitch)*hip_x+sin(roll)*hip_y+cos(roll)*cos(pitch)*hip_z;
	
	//printf("%f %f %f\n",hip_rot_x,hip_rot_y,hip_rot_z);
	rot_hip[0]=-hip_rot_z;
	rot_hip[1]=-hip_rot_x+(-50)*(1-leg)+50*leg;
	rot_hip[2]=hip_rot_y;


	float rleg_x=-r_leg_coods[1];
	float rleg_y=r_leg_coods[2];
	float rleg_z=-r_leg_coods[0];

	//printf("LEG %f %f %f\n",rleg_x,rleg_y,rleg_z);
	
	float rot_rleg_x=cos(pitch)*rleg_x+sin(pitch)*rleg_z;
	float rot_rleg_y=sin(roll)*sin(pitch)*rleg_x+cos(roll)*rleg_y+sin(roll)*cos(pitch)*rleg_z;
	float rot_rleg_z=-cos(roll)*sin(pitch)*rleg_x+sin(roll)*rleg_y+cos(roll)*cos(pitch)*rleg_z;

	rot_rleg[0]=-rot_rleg_z;
	rot_rleg[1]=-rot_rleg_x+(-50)*(1-leg)+50*leg;
	rot_rleg[2]=rot_rleg_y;
	
	if(rot_rleg[0]<=5)
	{
		//printf("!!!!!TOUCHDOWN!!!!!\n");
		touchdown=1;
	}
	else
	touchdown=0;
	zmp.coods[zmp.nof].x=rot_com[0];
	zmp.coods[zmp.nof].y=rot_com[1];
	zmp.coods[zmp.nof].z=rot_com[2]+COUNTER;
	
	zmp.left[zmp.nof].x=left_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
	zmp.left[zmp.nof].y=left_joints[6][1]-(50+left_joints[6][1])*(1-leg)-(-50+right_joints[6][1])*leg;
	zmp.left[zmp.nof].z=left_joints[6][2]-left_joints[6][2]*(1-leg)-right_joints[6][2]*leg+COUNTER;
	
	zmp.right[zmp.nof].x=right_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
	zmp.right[zmp.nof].y=right_joints[6][1]-(50+left_joints[6][1])*(1-leg)-(-50+right_joints[6][1])*leg;
	zmp.right[zmp.nof].z=right_joints[6][2]-left_joints[6][2]*(1-leg)-right_joints[6][2]*leg+COUNTER;
	
	//printf("ROT X:%lf\tY:%lf\tZ:%lf\n",rot_com[0],rot_com[1],rot_com[2]);
	#if PLOT==1
	f=fopen(comfile,"a");
	if(f!=NULL)
	{
		fprintf(f,"%lf\t%lf\t%lf\n",zmp.coods[zmp.nof].x,zmp.coods[zmp.nof].y,zmp.coods[zmp.nof].z);
		fclose(f);
	}	
	f=fopen(leftfile,"a");
	if(f!=NULL)
	{
		fprintf(f,"%lf\t%lf\t%lf\n",zmp.left[zmp.nof].x,zmp.left[zmp.nof].y,zmp.left[zmp.nof].z);
		fclose(f);
	}
	f=fopen(rightfile,"a");
	if(f!=NULL)
	{
		fprintf(f,"%lf\t%lf\t%lf\n",zmp.right[zmp.nof].x,zmp.right[zmp.nof].y,zmp.right[zmp.nof].z);
		fclose(f);
	}
	zmp.nof++;
	f=fopen(hipfile,"a");
	if(f!=NULL)
	{
		fprintf(f,"%lf\t%lf\t%lf\n",-hip_rot_z,-hip_rot_x+(-50)*(1-leg)+(50)*leg,hip_rot_y+COUNTER);
		fclose(f);
	}
	#endif
	return EXIT_SUCCESS;
	
}
	
int  projection_system(int leg)
{
	/*float cnst[6];
	cnst[0]=1675;	
	cnst[1]=3207;
	cnst[2]=1752;
	cnst[3]=1714;
	cnst[4]=274;
	cnst[5]=1703;
	*/
	gettimeofday (&tv, NULL);
	time_start = (tv.tv_sec *1e+6) + tv.tv_usec;

	int val[6];
	int i=0;
	int mot_no[]={0,1,2,3,4,5};
	int flag=0;	
	for(i=0;i<6;i++)
	{
		//printf("READING MOTOR NO. %d\n",mot_no[i]);
		//if(mot_no[i]==0)//||mot_no[i]==1||mot_no[i]==3)
		{
			val[i]=target_left[i];
			continue;
		}
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			printf("USING TARGET VALUE %d\n",mot_no[i]);
			val[i]=target_left[i];
		}
		else
		{
			val[i]-=offset[mot_no[i]];
			printf("MOT : %d TARGET : %d ACTUAL %d\n",mot_no[i],target_left[i],val[i]);
		}
		
		
	}
	
	forward_kinematics(0,val);
	for(i=0;i<6;i++)
	mot_no[i]+=20;
	
	for(i=0;i<6;i++)
	{
		//printf("READING MOTOR NO. %d\t",mot_no[i]);
		//if(mot_no[i]==25)
		{
			val[i]=target_right[i];
			continue;
		}
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			val[i]=target_right[i];
			printf("USING TARGET VALUE %d\n",mot_no[i]);
		}
		else
		{
			val[i]-=offset[mot_no[i]];
			printf("MOT : %d TARGET : %d ACTUAL %d\n",mot_no[i],target_right[i],val[i]);
		}
	}
	forward_kinematics(1,val);
	generate_com();
	/*if(leg==1)
	printf("LEFT  LEG X : %f Y : %f Z : %f\t",left_joints[6][0],left_joints[6][1],left_joints[6][2]);  
	else 
	printf("RIGHT LEG X : %f Y : %f Z : %f\t",right_joints[6][0],right_joints[6][1],right_joints[6][2]);  
	*/
	rotate_com(1-leg);
	gettimeofday (&tv, NULL);
	time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
	//printf("TIME : %ld\n",time_end-time_start);
	/*if(time_end-time_start>25000)
	printf("||||||||||||||||||||||||||||||||||||||||||||||||||||||________________________|||||||||||||||||||||||||||||||||||||\n");
	if(time_end-time_start>30000)
	printf("____________________|||||||||||||||||||||||||||||||||||||\n");
	*/
}
	
void send_sync()
{ 
	//ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
	iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
}



void close_files()
{
	int ret_motor,ret_imu;
	if(MOTOR_DATA==0)
	{
		if((ret_motor=ftdi_usb_close(&ftdic1)) < 0)
		{
			fprintf(stderr, "unable to close usb2d ftdi device: %d (%s)\n", ret_motor, ftdi_get_error_string(&ftdic1));
    		}
		ftdi_deinit(&ftdic1);
//	free(&ftdic1);
	}
	else if (MOTOR_DATA==1)
	{
		close(fd);
	}
	/*
	if(IMU_DATA==0)
	{
		if((ret_imu=ftdi_usb_close(&imu)) < 0)
		{
			fprintf(stderr, "unable to close usb2d ftdi device: %d (%s)\n", ret_imu, ftdi_get_error_string(&imu));
    		}
		ftdi_deinit(&imu);
	}
	else if(IMU_DATA==1)
	close(imu_t);
	*/
}

uint8_t bid[30];

void initialize_ids(void)  
{
  uint8_t initialize_ids_i;
  for(initialize_ids_i=0;initialize_ids_i<((NOM-4)/2)-1;initialize_ids_i++)
  {
    bid[initialize_ids_i] = initialize_ids_i;
  }
  bid[((NOM-4)/2)-1] = 15;
  bid[((NOM-4)/2)] = 16;
  bid[((NOM-4)/2)+1] = 17;
  bid[((NOM-4)/2)+2] = 18;


  for(initialize_ids_i=((NOM+4)/2)-1;initialize_ids_i<NOM;initialize_ids_i++)   
  {
    bid[initialize_ids_i] = initialize_ids_i+5;
  }
	bid[26]=12;
	bid[27]=32;

}
	
void set_offset()
{

	if(acyut==4)
	{
		//Using iran some later day offset
		offset[0]=15;
		offset[1]=350-25-20-30; 
		offset[2]=275; 
		offset[3]=232-20+10;//+25;  
		offset[4]=25; 
		offset[5]=-15-30+80;  
		offset[6]=346-40;
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0-100; 
		offset[10]=0; 	 
		offset[11]=0; 
		offset[12]=254; 
		offset[13]=0; 
		offset[14]=0; 
		offset[15]=-119; 
		offset[16]=42;
		offset[17]=0; 
		offset[18]=0;
		offset[19]=0; 
		offset[20]=-6;
		offset[21]=334+15-25-20-30; 
		offset[22]=288; 
		offset[23]=248-20+10;//+25; 
		offset[24]=-10; 
		offset[25]=-15+50-80; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]=0; 
		offset[29]=80; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=258;
		
		//Iran video init
		// offset[0]=15;
		// offset[1]=350-25-20; 
		// offset[2]=275;//-40 
		// offset[3]=232-20-25-25-10;    // 142s  
		// offset[4]=0; 
		// offset[5]=-15;//+7; 
		// offset[6]=346-40; 
		// offset[7]=0; 
		// offset[8]=0; 
		// offset[9]=0-100;//-(1023-865);//-150; 
		// offset[10]=0; 	 
		// offset[11]=0; 
		// offset[12]=254; 
		// offset[13]=0; 
		// offset[14]=0; 
		// offset[15]=-119; 
		// offset[16]=42; 
		// offset[17]=0; 
		// offset[18]=0; 
		// offset[19]=0; 
		// offset[20]=-6;
		// offset[21]=334+15-25-20; 
		// offset[22]=288;//-40; 
		// offset[23]=248-25-25-10;//-20; 
		// offset[24]=-10; 
		// offset[25]=-15;//-7; 
		// offset[26]=-306; 
		// offset[27]= 0; 
		// offset[28]=0;// 90*1023/300; 
		// offset[29]=80;//s1023-865; 
		// offset[30]=0;
		// offset[31]=0;
		// offset[32]=258;
	}
	else
	{
		offset[0]=-4;
		offset[1]=120;
		offset[2]=304;
		offset[3]=341;
		offset[4]=0;
		offset[5]=0;
		offset[6]=300;
		offset[7]=0;
		offset[8]=0;
		offset[9]=0;
		offset[10]=0;
		offset[11]=0;
    		offset[12]=254;
		offset[13]=0;
		offset[14]=0;
		offset[15]=-119;
		offset[16]=0;
		offset[17]=0;
		offset[18]=0;
		offset[19]=0;
		offset[20]=-19;
		offset[21]=321;
		offset[22]=304;
		offset[23]=341;
		offset[24]=0;
		offset[25]=0;
		offset[26]=-279;
		offset[27]= 0;
		offset[28]= 0;
		offset[29]=0;
		offset[30]=0;
		offset[31]=0;
		offset[32]=254;
	}
}	

#ifdef TESTING
int bootup_files()
#endif
#ifndef TESTING
int bootup_files_walk()
#endif
{
	initialize_ids();
	int chk_i_ids;
	int ret_motor,ret_imu;
	/*for (chk_i_ids = 0; chk_i_ids < 24; chk_i_ids += 1)
	{
		printf("bid[%d]= %d\n",chk_i_ids,bid[chk_i_ids]);
	}*/
	
	set_offset();
	if(MOTOR_DATA==0)
	{
		if (ftdi_init(&ftdic1) < 0)
    		{	
		        fprintf(stderr, "ftdi_init failed\n");
				ret_motor=-99;
        		//return EXIT_FAILURE;
    		}		
 		//ftdi_set_interface(&ftdic1,0);   
   		if ((ret_motor = ftdi_usb_open_desc (&ftdic1, 0x0403, 0x6001, NULL, serialusb2d)) < 0)
    		{
        		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret_motor, ftdi_get_error_string(&ftdic1));
        		printf("unable to open USB2D\n");
        		//return EXIT_FAILURE;
    		}
    		else
    		{
    			printf("USB2D ftdi successfully open\n");
			ftdi_set_baudrate(&ftdic1,1000000);
    		}	
	}
	else if(MOTOR_DATA==1)
	{
		struct termios options;
		struct timeval start, end;
		long mtime, seconds, useconds;
		if((fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
		{	
			ret_motor=-1;
			printf("Serial Port unavailable\n");
			close_files();
			exit(0);
		}
		else
		printf("Motor Line Opened By TERMIOS\n");
 		tcgetattr(fd, &options);
  		cfsetispeed(&options, B1000000);
  		cfsetospeed(&options, B1000000);
  		options.c_cflag |= (CLOCAL | CREAD);
  		options.c_cflag &= ~PARENB;
  		options.c_cflag &= ~CSTOPB;
     		options.c_cflag &= ~CSIZE;
     		options.c_cflag |= CS8;
     		options.c_oflag &= ~OPOST;
     		options.c_oflag &= ~ONOCR;
     		options.c_oflag |= FF0;
     		options.c_oflag &= ~FF1;
     		options.c_oflag &= ~ONLCR;
	
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    		options.c_cflag &= ~CRTSCTS;
//		tcflush(fd, TCIFLUSH);
  		tcsetattr(fd, TCSANOW, &options);
  		fcntl(fd, F_SETFL, FNDELAY);
	}
	else
	printf("UNKNOWN MOTOR CONNECTION\n");
	
		
	if(IMU_DATA==0)
	{
		if (ftdi_init(&imu) < 0)
    		{
	        	fprintf(stderr, "ftdi_init failed\n");
        		ret_imu=-99;
        		//return EXIT_FAILURE;
    		}	
   		ftdi_set_interface(&imu,2);
  		if ((ret_imu= ftdi_usb_open_desc (&imu, 0x0403, 0x6011, NULL, FT4232)) < 0)
    		{
        		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret_imu, ftdi_get_error_string(&imu));
        		printf("unable to open IMU\n");
       			 //return EXIT_FAILURE;
    		}	
    		else
    		{
    			printf("IMU ftdi successfully open\n");
	   		ftdi_set_baudrate(&imu,57600);
    		}
	}   
    	else if (IMU_DATA==1)
	{
		struct termios imu_options;
		if((imu_t=open("/dev/ttyUSB3", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
		{
			ret_imu=-1;
			printf("IMU Port Unavialable\n");
			close_files();
		//	exit(0);
		}
		tcgetattr(imu_t, &imu_options);
		cfsetispeed(&imu_options, B57600);
		cfsetospeed(&imu_options, B57600);
		imu_options.c_cflag |= (CLOCAL | CREAD);
  		imu_options.c_cflag &= ~PARENB;
  		imu_options.c_cflag &= ~CSTOPB;
     		imu_options.c_cflag &= ~CSIZE;
     		imu_options.c_cflag |= CS8;
     		imu_options.c_oflag |= OPOST;
     		imu_options.c_oflag |= ONOCR;
     		imu_options.c_oflag |= FF0;
     		imu_options.c_oflag &= ~FF1;
     		imu_options.c_oflag &= ~ONLCR;
	
		imu_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    		imu_options.c_cflag &= ~CRTSCTS;
//		tcflush(fd, TCIFLUSH);
  		tcsetattr(imu_t, TCSANOW, &imu_options);
  		fcntl(imu_t, F_SETFL,0);
		
	}
	else
	printf("UNKNOWN IMU CONNECTION\n");
	
	if(ret_motor>=0)//&&ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

}

int checksum(void)
{
  int bchecksum=0x00;
  int checksum_i=0;
  for(checksum_i = 2;checksum_i<=(tx_packet[3]+2);checksum_i++) //except 0xff,checksum
  {
    bchecksum = bchecksum + tx_packet[checksum_i];
	
  }
  bchecksum=bchecksum%256;
  //  bchecksum = bchecksum & 0x11;	
  //  bchecksum = ~bchecksum;
  //  bchecksum=bchecksum & 0x11;
  bchecksum= 255 - bchecksum;

  return bchecksum;

}


int checksum_last(void)
{
  int bchecksum=0x00;
  int checksum_i=0;
  for(checksum_i = 2;checksum_i<=(last_tx_packet[3]+2);checksum_i++) //except 0xff,checksum
  {
    bchecksum = bchecksum + last_tx_packet[checksum_i];
	
  }
  bchecksum=bchecksum%256;
  //  bchecksum = bchecksum & 0x11;	
  //  bchecksum = ~bchecksum;
  //  bchecksum=bchecksum & 0x11;
  bchecksum= 255 - bchecksum;

  return bchecksum;

}


void speed(int a)
{
	byte speed_packet[128];
	speed_packet[0]=0xff;
	speed_packet[1]=0xff;
	speed_packet[2]=0xfe;
	speed_packet[3]=NOM*3+4;
	speed_packet[4]=0x83;
	speed_packet[5]=0x20;
	speed_packet[6]=0x02;
	speed_packet[NOM*3+7]=0xff;
	int sync_write_i,i;
	switch(a)
	{
		case 0:
			for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		    {
		      speed_packet[7+(sync_write_i*3)]= bid[sync_write_i];
		      speed_packet[8+(sync_write_i*3)]=0x20;
		      speed_packet[9+(sync_write_i*3)]=0x00;
		    }
					
		    for(i=2;i<NOM*3+7;i++)
			speed_packet[3*NOM+7]-=speed_packet[i];

		    //if(ftdi_write_data(&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    if(iwrite(MOTOR_DATA,&fd,&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    printf("Speed Not Written\n");
			break;
		case 1:
			for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		    {	
		      speed_packet[7+(sync_write_i*3)]= bid[sync_write_i];
		      speed_packet[8+(sync_write_i*3)]=0x00;
		      speed_packet[9+(sync_write_i*3)]=0x00;
		    }
					
		    for(i=2;i<NOM*3+7;i++)
			speed_packet[3*NOM+7]-=speed_packet[i];

		    if(iwrite(MOTOR_DATA,&fd,&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    printf("Speed Not Written\n");
			break;
	}
	//tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
	//tx_packet[2]=0xfe;
	//tx_packet[3]=LEN_SYNC_WRITE;
	//tx_packet[4]=INST_SYNC_WRITE;
	//tx_packet[5]=0x1e;
	//tx_packet[6]=0x02;
}

//----------------------------------------imu functions-------------------------------------------------------------------------------


void imucal_init()
{
	//printf("entered imu cal init\n");
	float ang[3]={0};
	float acc[3]={0};
	float gyro[3]={0};
	byte im[20];
	byte i;
	byte j;
	byte k=0;
	byte a=3;
	byte flag=0;
	byte imu_no_of_times=0;
	byte imu_noat=0;
	byte flag2=0;
	byte start=0;
	base_angle[0]=base_angle[1]=base_angle[2]=0;
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIOFLUSH);
	while(imu_no_of_times<10)
	{
		imu_noat=0;
		flag2=0;
		while(flag2==0&&imu_noat<10)
		{   
				
			//if(ftdi_read_data(&imu,&im[0],1)>0)
			if(iread(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
			{	
				//printf("%d\t",im[0]);
							
				if(im[0]==0xff)
				{
					//printf("first ff detected....\n");
					start++;
				}
				else
				start=0;
				
				if(start==2)
				{	
					start=0;
					i=0;
					//printf("HERE\n");
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],20)>0 && im[18]==254 && im[19]==254)
					{	
						flag++;
						flag2=1;
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
							base_angle[k]+=ang[k];
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							acc[k]=im[j+6]+im[j+7]*256;
							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;
							base_acc[k]+=(acc[k])*9.8/260;	
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							gyro[k]=im[j+12]+im[j+13]*256;
							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;
							base_gyro[k]+=gyro[k];	
						}
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,base_angle[0],base_angle[1],base_angle[2]);				
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f BASE_ACC 0: %f BASE_ACC 1: %f BASE_ACC 2: %f BASE_GYRO 0: %f BASE_GYRO 1: %f BASE_GYRO 2: %f\n",flag,ang[0],ang[1],ang[2],acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);	
						
					}
				 }
			}
			imu_noat++;
		}
		imu_no_of_times++;
	}
	if(flag>=1)
	{
		for(k=0; k<3; k++)
		{
			base_angle[k]=base_angle[k]/flag;
			base_acc[k]=base_acc[k]/flag;
			base_gyro[k]=base_gyro[k]/flag;
			printf("BASE ANGLE %d : %lf     ",k,base_angle[k]);
			printf("BASE ACC %d : %lf     ",k,base_acc[k]);
			printf("BASE GYRO %d : %lf     ",k,base_gyro[k]);
		}
		printf("IMU INITIALIZED\n");
		imu_init=1;
	}
	else
	{
		printf("IMU NOT RESPONDING\n");
		imu_init=0;
	}
	
	/*for(k=0;k<3;k++)		
	{
		printf("BASE ANGLE : %d is %lf\n",k,base_angle[k]);
	}*/
	sleep(1);
}

void imu_initialize()
{
	imucal_init();
	if(imu_init==1)
	{
		gravity_magnitude=sqrt(pow(base_acc[0],2)+pow(base_acc[1],2)+pow(base_acc[2],2));
		
		gravity[0]=0;//base_acc[0];
		gravity[1]=0;//base_acc[1];
		gravity[2]=base_acc[2];
		printf("GRAVITY : %f\t%f\t%f\n",gravity[0],gravity[1],gravity[2]);		
	}
	
}	

inline void imucal()
{
	byte im[20],i,j,k=0,a=3,fallen=0;
	float ang[3]={0};
	float acc[3]={0};
	float gyro[3]={0};
	byte flag=0,imu_no_of_times=0;
	imu_angle[0]=0;
	imu_angle[1]=0;
	imu_angle[2]=0;
	imu_acc[0]=0;
	imu_acc[1]=0;
	imu_acc[2]=0;
	imu_gyro[0]=0;
	imu_gyro[1]=0;
	imu_gyro[2]=0;
	int start=0,count=0;
	gettimeofday(&tv,NULL); /*gets the current system time into variable tv */
	time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;
		
	//printf("BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",base_angle[0],base_angle[1],base_angle[2]);	
	//if(imu_init!=1)
	//printf("INITIALIZE THE IMU\n");
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
	while(imu_no_of_times<1&& imu_init==1)
	{		
		flag=0;
		while(flag==0)
		{   
			//if(ftdi_read_data(&imu,&im[0],1)>0)
			if(iread(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
			{
				if(im[0]==0xff)
				start++;
				else
				start=0;
				
				if(start==2)
				{
					start=0;
					//printf("PACKET RECIEVED\n");
					i=0;
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],20)>0 && im[18]==254 && im[19]==254)
					{	
						flag=1;	
					}
					
					if(flag==1)
					{	
						count++;
						//printf("count : %d\n",count);
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180-base_angle[k];
							imu_angle[k]=imu_angle[k]+ang[k];
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							acc[k]=im[j+6]+im[j+7]*256;
							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;
							imu_acc[k]=imu_acc[k]+(acc[k]*9.8/260);
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							gyro[k]=im[j+12]+im[j+13]*256;
							gyro[k]=((gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300);
							imu_gyro[k]=imu_gyro[k]+gyro[k]-base_gyro[k];
						}
						
					}	
				}
			}
		}
		imu_no_of_times++;
	}
	
	if(count!=0)
	{
		//angle_moved+=abs(imu_gyro[0])<20000?imu_gyro[0]:0;
		for(k=0;k<3;k++)		
		{
			imu_angle[k]=imu_angle[k]/count;
			imu_acc[k]=(imu_acc[k]/count);
			imu_gyro[k]=imu_gyro[k]/count;
			printf("ANGLE %d : %lf\t",k,imu_angle[k]);
			//printf("ACC %d : %lf\t",k,imu_acc[k]);
			//printf("GYRO %d : %lf\n",k,imu_gyro[k]);
			ang_acc[k]=imu_gyro[k]-prev_gyro[k];
			prev_gyro[k]=imu_gyro[k];
			//printf("ang acc %f\n",ang_acc[k]);
		}
	}
	else
	{
		printf("IMU READING FAILED\n");
		//usleep(16670*0.6);
	}
	ipurge(IMU_DATA,&imu_t,&imu);
	gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
	time_end  = (tv.tv_sec *1e+6) + tv.tv_usec;
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
}

inline void get_imu_data()
{
	int i=0;
	float gyro_threshold=0.01;
	float gyro_scale=1.5;
	float acc_magnitude;
	if(imu_init==0)
	{
		//printf("IMU NOT INITIALIZED\n");
		return ;
	}
	imucal();
	acc_magnitude=sqrt(pow(imu_acc[0],2)+pow(imu_acc[1],2)+pow(imu_acc[2],2));
	//printf("GRAVITY %f\t ACC %f\n",gravity_magnitude,acc_magnitude);
	//printf("GYRO 0 :%f GYRO 1 : %f\n",imu_gyro[0],imu_gyro[1]);
	if(fabs(imu_gyro[0])<=gyro_threshold&&fabs(imu_gyro[1])<=gyro_threshold)//&&abs(imu_gyro[2])<=gyro_threshold)
	{
		if(fabs(acc_magnitude-gravity_magnitude)<1)
		{	
			//// STATIC CONDITION - > RESETTING HELD ANGLE 
			for(i=0;i<2;i++)
			{
				//printf("STATIC COND \n");
				held_angle[i]=imu_angle[i];
				gyro_angle[i]=0;
				//current_angle[i]=held_angle[i]+gyro_angle[i];
			}
		}
		else
		{
			//// LINEAR ACC CONDITION 
			for(i=0;i<2;i++)
			{
				//printf("PURE ACC \n");
				held_angle[i]=held_angle[i];
				//gyro_angle[i]+=imu_gyro[i]*gyro_scale;	
				//current_angle[i]=held_angle[i]+gyro_angle[i];
			}
		}
	}
	else
	{
		for(i=0;i<2;i++)
		{
			//printf("ROTATING\n");
			gyro_angle[i]+=imu_gyro[i]*gyro_scale;
			//printf("GYRO ANGLE %d : %f\n",i,gyro_angle[i]);	
		}
	}
	
	for(i=0;i<2;i++)
	{
/*		current_angle[i]=held_angle[i]+gyro_angle[i];	*/
		current_angle[i]=imu_angle[i];
		printf("ANGLE[%d] = %f\t",i,current_angle[i]);
	}
	printf("\n");
	//float temp=current_angle[1];
	current_angle[1]*=-1;//current_angle[0];
	current_angle[0]*=-1;//temp;
	//imu_acc[2]+=43;
	current_acc[0]=cos(current_angle[0]*D2R)*imu_acc[0]-sin(current_angle[1]*D2R)*imu_acc[2];
	current_acc[1]=sin(current_angle[0]*D2R)*sin(current_angle[1]*D2R)*imu_acc[0]+cos(current_angle[0]*D2R)*imu_acc[1]+sin(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	current_acc[2]=cos(current_angle[0]*D2R)*sin(current_angle[1]*D2R)*imu_acc[0]-sin(current_angle[0]*D2R)*imu_acc[1]+cos(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	
	//for(i=0;i<=2;i++)
	//{
		//printf("ACC[%d] = %f\t IMU_ACC[%d] %f\tGRAVITY[%d] %f\n",i,current_acc[i],i,imu_acc[i],i,gravity[i]);
	//}
	
	current_acc[0]-=base_acc[0];//gravity[0];
	current_acc[1]-=base_acc[1];///gravity[1];
	current_acc[2]-=base_acc[2];//gravity[2];
	/*current_acc[0]=cos(current_angle[1]*D2R)*imu_acc[0]+sin(current_angle[1]*D2R)*sin(current_angle[0]*D2R)*imu_acc[1]-sin(current_angle[1]*D2R)*cos(current_angle[0]*D2R)*imu_acc[2];
	current_acc[1]=cos(current_angle[0]*D2R)*imu_acc[1]+sin(current_angle[0]*D2R)*imu_acc[2];
	current_acc[2]=sin(current_angle[1]*D2R)*imu_acc[0]-sin(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[1]+cos(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	*/
	//for(i=0;i<=2;i++)
	//{
	//	printf("ACC[%d] = %f\t IMU_ACC[%d] %f\tGRAVITY[%d] %f\n",i,current_acc[i],i,imu_acc[i],i,gravity[i]);
	//}
	
	///// ROTATE ACC VECTOR
	///// SUBTRACT GRAVITY
}		

int high_punch=150;
int low_punch=0;

int compliance(int leg,int cp,int cpr)
{
	cpr=32;
	cp=32;
	int mot,revmot,h=0;
	if(leg==0)
	{
		mot=0;
		revmot=20;
	}
	else if(leg==1)
	{
		mot=20;
		revmot=0;
	}
	else
	{
		printf("TAKE LITE\n");
		return EXIT_FAILURE;
	}
	
	byte compliance_packet[]={0xff,0xff,0xfe,0x28,0x83,0x1c,0x02,mot,cp,cp,mot+1,cp,cp,mot+2,cp,cp,mot+12,cp,cp,mot+3,cp,cp,mot+5,cp,cp,revmot,cpr,cpr,revmot+1,cpr,cpr,revmot+2,cpr,cpr,revmot+12,cpr,cpr,revmot+3,cpr,cpr,revmot+5,cpr,cpr,0xff};
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];

	//if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,compliance_packet,compliance_packet[3]+4)<0)
	{
		//printf("Compliance Written\n");
		//else
		printf("ERROR : Compliance\n");
	}	
}


float* generatespline(int gn,float gx[],float gy[])
{
	
	  float ga[gn+1];
	  float gb[gn];
	  float gd[gn];
	
	  float gh[gn];
	  float gal[gn];
	  float gc[gn+1],gl[gn+1],gmu[gn+1],gz[gn+1];
	
	  //1..............................
	 // ga[]=new float[gn+1];
		int gi; 
	  for(gi=0;gi<gn+1;gi++)
	    ga[gi]=gy[gi];
	  //2.................................
	  //gb[]=new float[gn];
	  //gd[]=new float[gn];
	  //3.................................
	  //gh[]=new float[gn];
	  for(gi=0;gi<gn;gi++)
	    gh[gi]=gx[gi+1]-gx[gi];
	  //4...................................
	  //gal[]=new float[gn];
	  for(gi=1;gi<gn;gi++)
	    gal[gi]=3*((((float)ga[gi+1]-(float)ga[gi])/(float)gh[gi])-(((float)ga[gi]-(float)ga[gi-1])/(float)gh[gi-1]));
	  //5......................................
	  //gc[]=new float[gn+1];
	  //gl[]=new float[gn+1];
	  //gmu[]=new float[gn+1];
	  //gz[]=new float[gn+1];
	  //6........................................
	  gl[0]=1;
	  gmu[0]=0;
	  gz[0]=0;
	  //7.........................................
	  for(gi=1;gi<gn;gi++)
	  {
	    gl[gi]=2*(gx[gi+1]-gx[gi-1])-(gh[gi-1]*gmu[gi-1]);
	    gmu[gi]=(float)gh[gi]/(float)gl[gi];
	    gz[gi]=((float)gal[gi]-(float)gh[gi-1]*gz[gi-1])/(float)gl[gi];
	  }
	  //8.........................................
	  gl[gn]=1;
	  gz[gn]=0;
	  gc[gn]=0;
	  //9.............................................
	  int gj;
	  for(gj=gn-1;gj>=0;gj--)
	  {
	    gc[gj]=gz[gj]-gmu[gj]*gc[gj+1];
	    gb[gj]=(((float)ga[gj+1]-(float)ga[gj])/(float)gh[gj])-(((float)gh[gj]*((float)gc[gj+1]+2*(float)gc[gj]))/(float)3);
	    gd[gj]=((float)gc[gj+1]-(float)gc[gj])/(float)(3*gh[gj]);
	  }
	  //10......................................................
	
	
	  int tp=0,gt;
	  for(gt=0;gt<gn;gt++)
	  {
	    printf("eqn");
	    output[tp]=ga[gt];
	    printf("%f    ",ga[gt]);
	    tp++;
	    output[tp]=gb[gt];
	    printf("%f    ",gb[gt]);
	    tp++;
	    output[tp]=gc[gt];
	    printf("%f    ",gc[gt]);
	    tp++;
	    output[tp]=gd[gt];
	    printf("%f    \n",gd[gt]);
	    tp++;
	  }
	  return output;
}



float* generateik(float x, float y, float z,int leg)
{
	//printf("%lf %lf %lf\n",x,y,z);
	int i=0;
	float cons[10];
	float ang[6];
	float theta[4];
	float phi[5];
	float b_len=180;   // Length of bracket 
	float l22=49;		// Distance between bracket ends
	float r2d=180/3.14159265358979323846264338327950288419716939937510;
	float leg_l=pow(x*x+y*y+z*z,0.5);
	
	cons[0]=1675;
	cons[1]=2952+255;
	cons[2]=2008+973;
	cons[3]=1459+255;
	cons[4]=1703;
	
	// CALCULATING LEG LENGTH
	float val=(leg_l-l22)/(2*b_len);
	theta[3]=acos(val); // angle for motor 3 from XZ Acyut plane 
	theta[2]=asin(val);	// angle for motor 2 12 from YZ Acyut plane 
	theta[1]=theta[3];	// angle for motor 1 from XZ Acyut plane 
	
	// CALCULATING LEG POSITION
	
	phi[3]=atan(y/x);
	phi[5]=atan(z/x);
	phi[1]=phi[3];
	phi[0]=phi[5];
	
	// GETTING ANGLES
	//printf("%lf asdjhkdshdk\n",avg_angle[1]);
	ang[0]=phi[0];//+(avg_angle[1]*pow(-1,leg))/r2d;
	ang[1]=-theta[1]+phi[1];//-(avg_angle[0]/r2d);
	ang[2]=-theta[2];
	ang[3]=-theta[3]-phi[3];
	ang[4]=phi[5];//+(avg_angle[1]/r2d)*pow(-1,leg);;
	//printf("%f\t%f\n",phi[0],ang[0]);
	mval[0]=(ang[0]*r2d*4096/300)+cons[0];//+z*1.1;
	mval[1]=(ang[1]*r2d*4096/300)+cons[1];
	mval[2]=(ang[2]*r2d*4096/300)+cons[2];
	mval[3]=(ang[3]*r2d*4096/300)+cons[3];
	mval[4]=(ang[4]*r2d*4096/300)+cons[4];//-z*1.1;

	// GEN PRINTING FOR DEBUGGING
	
	//for(i=0;i<5;i++)
	//{
		//printf("ANG[%d] : %f\t",i,ang[i]*r2d);
	//}
	//printf("\n");
	//for(i=0;i<5;i++)
	//printf("mval[%d] : %f\t",i,mval[i]);
	//printf("\n");
	//printf("THETA 3 : %f\tTHETA 2 : %f\tTHETA 1 : %f\t\n",theta[3]*r2d,theta[2]*r2d,theta[1]*r2d);
	//printf("PHI 3 : %f\tPHI 5 : %f\tPHI 1 : %f\tPHI 0 : %f\t\n",phi[3]*r2d,phi[5]*r2d,phi[1]*r2d,phi[0]*r2d);
	return mval;

}

float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}
float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

float linear(float in,float fi,float t, float tot)
{
	return in*(1-t/tot)+fi*(t/tot);
}

float rotation(int mot,float phi)
{
	 float motval;
	 if(mot==4)
	 motval=742-phi*1023/300;   /// 742 , 274 are initialize values of motors 4 , 24
	 else if(mot==24)
	 motval=274+phi*1023/300;
	 else
	 motval=sqrt(-1);
	 
	 return motval;
}



int torque(int enable)
{
	if(enable!=0)
	enable=1;
	
	int c=0;
	byte torque_packet[NOM*2+8];
	torque_packet[0]=0xff;
	torque_packet[1]=0xff;
	torque_packet[2]=0xfe;
	torque_packet[3]=NOM*2+4;
	torque_packet[4]=0x83;
	torque_packet[5]=0x18;
	torque_packet[6]=0x01;
	torque_packet[7+NOM*2]=0xff-torque_packet[2]-torque_packet[3]-torque_packet[4]-torque_packet[5]-torque_packet[6];
	
	for(c=0;c<NOM;c++)
	{
		torque_packet[7+c*2]=bid[c];
		torque_packet[8+c*2]=enable;
		torque_packet[7+NOM*2]-=(torque_packet[7+c*2]+torque_packet[8+c*2]);
	}
	
	//if(ftdi_write_data(&ftdic1,torque_packet,torque_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,torque_packet,torque_packet[3]+4)>0)
	printf("TORQUE PACKET SENT\n");
	else
	printf("ERROR : TORQUE\n");
	
	
}

void set_walk_offset()
{
	offset[0]+=0;
	offset[1]+=0;
	offset[2]+=0;
	offset[3]+=0;
	offset[4]+=0;
	offset[5]+=0;
	offset[6]+=0;
	offset[7]+=0;
	offset[8]+=0;
	offset[9]+=0;
	offset[10]+=0;
	offset[11]+=0;
	offset[12]+=0;
	offset[13]+=0;
	offset[14]+=0; 
	offset[15]+=0; 
	offset[16]+=0; 
	offset[17]+=0; 
	offset[18]+=0; 
	offset[19]+=0; 
	offset[20]+=0;
	offset[21]+=0;
	offset[22]+=0; 
	offset[23]+=0; 
	offset[24]+=0; 
	offset[25]+=0; 
	offset[26]+=0; 
	offset[27]+=0; 
	offset[28]+=0; 
	offset[29]+=0; 
	offset[30]+=0;
	offset[31]+=0; 
	offset[32]+=0;
}
// Correction 
float err_z=0;
float err_y=0;
float cor_z=0;
float prev_cor_z=0;
float prev_err_z=0;
float future_err_z[40]={0};

float sum_err(float *err, float coeff_a)
{
	int sum=0;
	int i=0;	
	
	for(i=0;i<120;i++)
	sum+=pow(E,-coeff_a*i)*err[i];
	
	return sum;
}
long double fact(int n)
{
	long int i=0;
	long double fact=1;
	for(i=1;i<=n;i++)
	fact*=i;
	return fact;
}

float pid_z(int leg)
{
/*	float pk=1;*/
/*	float pd=0;*/
/*	*/
/*	int i=0;*/
/*	for(i=119;i>0;i--)*/
/*	cor_z[i]=cor_z[i-1];*/

/*	cor_z[0]=pk*err_z+pd*(err_z-prev_err_z);*/
/*	if(abs(cor_z[0])>50)*/
/*	{*/
/*		printf("*******************************************************************\n");*/
/*		cor_z[0]=50*cor_z[0]/abs(cor_z[0]);*/

/*	}*/
/*	prev_err_z=err_z;*/

/*	//printf("CORR : %lf\n",cor_z[0]);*/
/*	*/
	
}

int setupWalk()
{
	walk.time=1;
	walk.exp_time=1;
	walk.fps=60;
	
	walk.left.x.lift.x_start=0.2;
	walk.left.x.lift.x_end=1;
	walk.left.x.lift.lift=40;
	walk.left.x.lift.height=390;
	walk.left.x.lift.freq=2*3.14;
	walk.left.x.lift.displacement=1;	
	walk.left.x.lift.phase=-1.57;
	
	walk.left.x.land.x_land_start=0.9;//1-x_end    
	walk.left.x.land.x_land_increase_end=1;
	walk.left.x.land.x_land_decrease_start=1.5;
	walk.left.x.land.x_land_end=2;
	
	walk.left.x.land.x_land_freq=3.14;
	walk.left.x.land.x_land_displacement=1;
	walk.left.x.land.x_land_phase=-1.57;
	walk.left.x.land.x_land_dec=0;
	
	walk.left.x.load.x_load_start=0.15;
	walk.left.x.load.x_load_increase_end=0.5;
	walk.left.x.load.x_load_decrease_start=0.5;
	walk.left.x.load.x_load_end=0.85;
	walk.left.x.load.x_load_displacement=1;//x_land_displacement;
	walk.left.x.load.x_load_phase=-1.57;//-1.57;
	walk.left.x.load.x_load_freq=3.14;
	walk.left.x.load.x_load_dec=0;//lift/60;
	                  
	walk.left.z_shift.peak_start=0.0;
	walk.left.z_shift.peak_max=0.4;//0.5
	walk.left.z_shift.peak_stay=0.5;//0.75
	walk.left.z_shift.peak_end=1;
	walk.left.z_shift.zmax=45;
	walk.left.z_shift.zfreq=1.57;
	walk.left.z_shift.zphase=0;
	walk.left.z_shift.zdisplacement=0;
	
	walk.left.y.y_start=0.10;
	walk.left.y.y_0=0.30;
	walk.left.y.y_move=0.70;
	walk.left.y.y_end=0.90;
/*	walk.left.y.yr_start=0.10;*/
/*	walk.left.y.yr_end=0.90;*/

	walk.left.z.z_start=0.25;
	walk.left.z.z_0=0.417;
	walk.left.z.z_move=0.5833;
	walk.left.z.z_end=0.75;
	
    	walk.left.misc.seperation=0;
	walk.left.misc.cptime=walk.left.x.lift.x_end;

	walk.left.turn.turn_start=0.2;
	walk.left.turn.turn_end=0.8;

	walk.left.com_trajectory.z_com_start=0;
	walk.left.com_trajectory.z_com_peak=0.25;
	walk.left.com_trajectory.z_com_stay=0.75;
	walk.left.com_trajectory.z_com_end=1;
	walk.left.com_trajectory.z_com_max=32;
	walk.left.com_trajectory.z_static_inc=4;
	
	walk.left.com_trajectory.y_com_start=0;
	walk.left.com_trajectory.y_com_end=1;

	walk.right=walk.left;
	walk.right.z_shift.zmax=32;
	
}

float setWalkLift(float t_frac,x_lift lift)
{
	float x=lift.height;
	
	////X LIFT////		
	if(t_frac>lift.x_start&&t_frac<=lift.x_end)
	x-=lift.lift*(sin(lift.freq*(t_frac-lift.x_start)/(lift.x_end-lift.x_start)+lift.phase)+lift.displacement)/(1+lift.displacement);
	
	return x;
}

float setWalkLand(float t_frac,x_land land)
{
	float x=0;
	///// Soften landing     
	if(t_frac>land.x_land_start&&t_frac<=land.x_land_increase_end)
	x-=land.x_land_dec*(sin(land.x_land_freq*(t_frac-land.x_land_start)/(land.x_land_increase_end-land.x_land_start)+land.x_land_phase)+land.x_land_displacement)/(1+land.x_land_displacement);
	else if(t_frac>land.x_land_increase_end&&t_frac<=land.x_land_decrease_start)
	x-=land.x_land_dec;
	else if(t_frac>land.x_land_decrease_start&&t_frac<=land.x_land_end)
	x-=land.x_land_dec*(sin(land.x_land_freq*(t_frac-land.x_land_decrease_start)/(land.x_land_end-land.x_land_decrease_start)+land.x_land_phase)+land.x_land_displacement)/(1+land.x_land_displacement);
	//x-=x_land_dec*(sin(x_land_freq*(t_frac-x_end+x_land_window)/(2*x_land_window)+x_land_phase)+x_land_displacement)/(1+x_land_displacement);
	
	return x;
}

float setWalkLoad(float t_frac, x_load load)
{

	float xr=0;
	///// Assist weight shift
	if(t_frac>load.x_load_start&&t_frac<=load.x_load_increase_end)
	xr-=load.x_load_dec*(sin(load.x_load_freq*(t_frac-load.x_load_start)/(load.x_load_increase_end-load.x_load_start)+load.x_load_phase)+load.x_load_displacement)/(1+load.x_load_displacement);
	else if(t_frac>load.x_load_increase_end&&t_frac<=load.x_load_decrease_start)
	xr-=load.x_load_dec;
	else if(t_frac>load.x_load_decrease_start&&t_frac<=load.x_load_end)
	xr-=load.x_load_dec*(1-(sin(load.x_load_freq*(t_frac-load.x_load_decrease_start)/(load.x_load_end-load.x_load_decrease_start)+load.x_load_phase)+load.x_load_displacement)/(1+load.x_load_displacement));
		
	return xr;
}

float setWalkShift(float t_frac,walk_zshift zshift)
{
	float z_shift=0;
	/////Z SHIFT//////
	if(t_frac<=zshift.peak_start)
	z_shift=0;
	else if(t_frac>zshift.peak_start&&t_frac<=zshift.peak_max)
	z_shift=zshift.zmax*(sin(zshift.zfreq*(t_frac-zshift.peak_start)/(zshift.peak_max-zshift.peak_start)+zshift.zphase)+zshift.zdisplacement)/(1+zshift.zdisplacement);
	else if(t_frac>zshift.peak_stay&&t_frac<=zshift.peak_end)
	z_shift=zshift.zmax*(sin(zshift.zfreq*(t_frac-zshift.peak_stay)/(zshift.peak_end-zshift.peak_stay)+zshift.zphase+1.57)+zshift.zdisplacement)/(1+zshift.zdisplacement);
	else if(t_frac>zshift.peak_end)
	z_shift=0;
	
	return z_shift;
}

float y_val[2];
	
float* setWalkY(float t_frac, walk_y y, float yin,float yfi, float yrin, float yrfi)
{
	/*////Y MOVEMENT////
	if(t_frac<=y.y_start)
	y_val[0]=yin;  
        else if(t_frac>y.y_start&&t_frac<=y.y_end)
	y_val[0]=scurve2(yin,yfi,t_frac-y.y_start,y.y_end-y.y_start);
	else if(t_frac>y.y_end)
	y_val[0]=yfi;
	
	////Y MOVEMENT////
	if(t_frac<=y.yr_start)
	y_val[1]=yrin;  
        else if(t_frac>y.yr_start&&t_frac<=y.yr_end)
	y_val[1]=scurve2(yrin,yrfi,t_frac-y.yr_start,y.yr_end-y.yr_start);
	else if(t_frac>y.yr_end)
	y_val[1]=yrfi;
	*/
	if(t_frac<=y.y_start)
	{
		y_val[0]=yin;
		y_val[1]=yrin;
	}
	else if(t_frac>y.y_start&& t_frac<=y.y_0)
	{
		y_val[0]=scurve2(yin,yin-yrin,t_frac-y.y_start,y.y_0-y.y_start);
		y_val[1]=scurve2(yrin,0,t_frac-y.y_start,y.y_0-y.y_start);
	}
	else if(t_frac>y.y_0&&t_frac<=y.y_move)
	{
		y_val[0]=scurve2(yin-yrin,yfi-yrfi,t_frac-y.y_0,y.y_move-y.y_0);
		y_val[1]=0;
	}
	else if(t_frac>y.y_move&&t_frac<=y.y_end)
	{
		y_val[0]=scurve2(yfi-yrfi,yfi,t_frac-y.y_move,y.y_end-y.y_move);
		y_val[1]=scurve2(0,yrfi,t_frac-y.y_move,y.y_end-y.y_move);
	}
	else
	{
		y_val[0]=yfi;
		y_val[1]=yrfi;
	}
	
	return y_val;
}

float z_val[2];
	

float* setWalkZ(float t_frac,walk_z z, float zin, float zfi, float zrin,float zrfi)
{
	if(t_frac<=z.z_start)
	{
		z_val[0]=zin;
		z_val[1]=zrin;
	}
	else if(t_frac>z.z_start&&t_frac<=z.z_0)	
	{
		z_val[0]=scurve(zin,zin+zrin,t_frac-z.z_start,z.z_0-z.z_start);
		z_val[1]=scurve(zrin,0,t_frac-z.z_start,z.z_0-z.z_start);
	}
	else if(t_frac>z.z_0&&t_frac<=z.z_move)
	{
		z_val[0]=scurve(zin+zrin,zfi+zrfi,t_frac-z.z_0,z.z_move-z.z_0);
		z_val[1]=scurve(0,0,t_frac-z.z_0,z.z_move-z.z_0);
	}
	else if(t_frac>z.z_move&&t_frac<=z.z_end)
	{
		z_val[0]=scurve(zrfi+zfi,zfi,t_frac-z.z_move,z.z_end-z.z_move);
		z_val[1]=scurve(0,zrfi,t_frac-z.z_move,z.z_end-z.z_move);
	}

	return z_val;

}		
 
float phi_val[2];

float* setTurnPhi(float t_frac,walk_turn turn, float thetain,float thetafi,float thetarin,float thetarfi)
{
			
	if(t_frac<=turn.turn_start)
	{
		phi_val[0]=thetain;
		phi_val[1]=thetarin;
	}
	else if(t_frac>turn.turn_start&&t_frac<=turn.turn_end)
	{
		phi_val[0]=scurve(thetain,thetafi,t_frac-turn.turn_start,turn.turn_end-turn.turn_start);
		phi_val[1]=scurve(thetarin,thetarfi,t_frac-turn.turn_start,turn.turn_end-turn.turn_start);
	}
	else if(t_frac>turn.turn_end)
	{
		phi_val[0]=thetafi;
		phi_val[1]=thetarfi;
	}
		
	return phi_val;
}

float com_val[2];


	
float* setCOM(float t_frac, walk_com com, float yrin, float yrfi)
{
	if(t_frac<=com.z_com_start)
	com_val[0]=0;
	else if(t_frac>com.z_com_start&&t_frac<=com.z_com_peak)
	com_val[0]=com.z_com_max*(sin(3.14*(t_frac-com.z_com_start)/(com.z_com_peak-com.z_com_start)-1.57)+1)/2;//0*(1-(t_frac-z_com_start)/(z_com_peak-z_com_start))+z_com_max*((t_frac-z_com_start)/(z_com_peak-z_com_start));
	else if(t_frac>com.z_com_peak&&t_frac<=com.z_com_stay)
	com_val[0]=com.z_com_max;//+z_static_inc*(sin(2*3.14*(t_frac-z_com_peak)/(z_com_stay-z_com_peak)-1.57)+1)/2;
	else if(t_frac>=com.z_com_stay&&t_frac<=com.z_com_end)
	com_val[0]=com.z_com_max-com.z_com_max*(sin(3.14*(t_frac-com.z_com_stay)/(com.z_com_end-com.z_com_stay)-1.57)+1)/2;
	else
	com_val[0]=0;
	
	if(t_frac<=com.y_com_start)
	com_val[1]=yrin;
	else if(t_frac>com.y_com_start&&t_frac<=com.y_com_end)
	com_val[1]=yrin-(yrin-yrfi)*(t_frac-com.y_com_start)/(com.y_com_end-com.y_com_start);
	else
	com_val[1]=yrfi;
	
	return com_val;
	
}


int walk3(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi) //walk charli
{
	setupWalk();
	
	FILE *f;
	
	float time=walk.time;
	float exp_time=time;
	float fps=walk.fps;
	
	x_lift lift;
	x_land land;
	x_load load;
	walk_zshift zshift;
	walk_y y_control;
	walk_z z_control;
	walk_turn turn;
	walk_misc misc;
	walk_com com;
	if(leg==0)
	{
		lift =walk.left.x.lift;
		land =walk.left.x.land;
		load =walk.left.x.load;
		zshift =walk.left.z_shift;
		y_control =walk.left.y;
		z_control =walk.left.z;
		turn =walk.left.turn;
		misc=walk.left.misc;
		com=walk.left.com_trajectory;
	}
	else if(leg==1)
	{
		lift =walk.right.x.lift;
		land =walk.right.x.land;
		load =walk.right.x.load;
		zshift =walk.right.z_shift;
		y_control =walk.right.y;
		z_control =walk.right.z;
		turn =walk.right.turn;
		misc=walk.right.misc;
		com=walk.right.com_trajectory;
	}
			

	
	/////// COM Trajectory //////
	//Z Movement
	float z_com_start=walk.left.com_trajectory.z_com_start*(1-leg)+walk.right.com_trajectory.z_com_start*(leg);
	float z_com_peak=walk.left.com_trajectory.z_com_peak*(1-leg)+walk.right.com_trajectory.z_com_peak*(leg);
	float z_com_stay=walk.left.com_trajectory.z_com_stay*(1-leg)+walk.right.com_trajectory.z_com_stay*(leg);
	float z_com_end=walk.left.com_trajectory.z_com_end*(1-leg)+walk.right.com_trajectory.z_com_end*(leg);
	float z_com_max=walk.left.com_trajectory.z_com_max*(1-leg)+walk.right.com_trajectory.z_com_max*(leg);
	float z_static_inc=walk.left.com_trajectory.z_static_inc*(1-leg)+walk.right.com_trajectory.z_static_inc*(leg);
	
	//Y Movement
	float y_com_start=walk.left.com_trajectory.y_com_start*(1-leg)+walk.right.com_trajectory.y_com_start*(leg);
	float y_com_end=walk.left.com_trajectory.y_com_end*(1-leg)+walk.right.com_trajectory.y_com_end*(leg);

	
	/////// Live Variables /////
	int frame=0;
	float t_frac=0,future_frac=0,t=0;



	float x,xr,y,z,yr,zr,z_shift,phi,phir;
	float yreal,yrreal,zreal,zrreal;
	int z_flag=0;
	float acc_time;
	
	float exp_velo_z,exp_acc_z;
	
	/////// Packet Generation Variables /////
	int adchksum=7+NOM*3,i=0;
	float *fval,*retval;
	float val[5],valr[5];
	int mot=0*(1-leg)+20*leg;
	int revmot=20*(1-leg)+0*leg;
	int value=0;
        
	compliance(leg,7,7);	
	for(frame=0;frame<time*fps;frame++,t+=1.0/fps)
	{
		t_frac=t/time;
		future_frac=(t+1.0/fps)/time;
		
		x=setWalkLift(t_frac,lift)+setWalkLand(t_frac,land);
		xr=lift.height+setWalkLoad(t_frac,load);
			
		
		
		z_shift=setWalkShift(t_frac,zshift);
		
		retval=setWalkY(t_frac,y_control,yin,yfi,yrin,yrfi);
		y=retval[0];
		yr=retval[1];
		
		retval=setWalkZ(t_frac,z_control,zin,zfi,zrin,zrfi);
		z=retval[0];
		zr=retval[1];
		
		retval=setTurnPhi(t_frac,turn,thetain,thetafi,thetarin,thetarfi);
		phi=retval[0];
		phir=retval[1];
				
		retval=setCOM(t_frac,com,yrin,yrfi);
		com_z=z_shift;//retval[0];
		com_y=retval[1];
		
		
		f=fopen(expect_com,"a");
		if(f!=NULL)
		{
			fprintf(f,"0\t%lf\t%lf\n",pow(-1,leg)*com_z,com_y+18+COM_INC);
			fclose(f);
		}
	
		/////Compliance////
		if(compare_float(misc.cptime,t_frac)==1)
		compliance(leg,10,7);
		
		//zr+=err_z;
		//z-=err_z;
		//z+=z_shift;
		//zr-=z_shift;
		
		if(leg==0)
		{
			zreal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yreal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrreal=zr*cos(phir*3.14/180)-yr*sin(phir*3.14/180);
			yrreal=zr*sin(phir*3.14/180)+yr*cos(phir*3.14/180);
		}
		else if(leg==1)
		{
			zreal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yreal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrreal=zr*cos(-phir*3.14/180)+yr*sin(phir*3.14/180);
			yrreal=-zr*sin(-phir*3.14/180)+yr*cos(phir*3.14/180);
			
		}
			
		zreal+=z_shift;
		zrreal-=z_shift;
		/*if(abs(err_z)<25)
		{
			zrreal+=err_z*0.2;
			zreal-=err_z*0.2;
		}*/
		//zrreal+=0.5*sum_err(cor_z,1);
		//zreal-=0.5*sum_err(cor_z,1);
		fval=generateik(x,yreal,zreal+misc.seperation,leg);
		for(i=0;i<5;i++)
		val[i]=fval[i];
		fval=generateik(xr,yrreal,zrreal+misc.seperation,1-leg);
		for(i=0;i<5;i++)
		valr[i]=fval[i];
		
		///// PROJECTION SYSTEM 
		target_left[0]=val[0]*(1-leg)+valr[0]*leg;
		target_left[1]=val[1]*(1-leg)+valr[1]*leg;	
		target_left[2]=val[2]*(1-leg)+valr[2]*leg;
		target_left[3]=val[3]*(1-leg)+valr[3]*leg;
		target_left[4]=rotation(4,phi*(1-leg)+phir*leg);
		target_left[5]=val[4]*(1-leg)+valr[4]*leg;
		
		target_right[0]=valr[0]*(1-leg)+val[0]*leg;
		target_right[1]=valr[1]*(1-leg)+val[1]*leg;
		target_right[2]=valr[2]*(1-leg)+val[2]*leg;
		target_right[3]=valr[3]*(1-leg)+val[3]*leg;
		target_right[4]=rotation(24,phir*(1-leg)+phi*leg);
		target_right[5]=valr[4]*(1-leg)+val[4]*leg;
		
		projection_system(leg);
	

		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
				value=val[0]+offset[tx_packet[7+i*3]]-pow(-1,leg);//*change[1]*4096/300*7/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+1)
			{
				value=val[1]+offset[tx_packet[7+i*3]];//+change[0]*4096/300*3/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+2||tx_packet[7+i*3]==mot+12)
			{
				value=val[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+3)
			{
				value=val[3]+offset[tx_packet[7+i*3]];//-change[0]*4096/300*5/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+4)
			{
				value=rotation(mot+4,phi)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1024)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+5)
			{
				value=val[4]+offset[tx_packet[7+i*3]]+pow(-1,leg);//*change[1]*4096/300*12/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot)
			{
				value=valr[0]+offset[tx_packet[7+i*3]]+pow(-1,leg);//*change[1]*4096/300*7/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+1)
			{
				value=valr[1]+offset[tx_packet[7+i*3]];//+change[0]*4095/300*3/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+2||tx_packet[7+i*3]==revmot+12)
			{
				value=valr[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+3)
			{
				value=valr[3]+offset[tx_packet[7+i*3]];//-change[0]*4096/300*5/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+4)
			{
				value=rotation(revmot+4,phir)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1024)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+5)
			{
				value=valr[4]+offset[tx_packet[7+i*3]]-pow(-1,leg);//*change[1]*4096/300*12/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}	
		}
		tx_packet[adchksum]=255;

		for(i=2;i<adchksum;i++)
		tx_packet[adchksum]-=tx_packet[i];

		if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)<0)
		{
			//printf("ERROR : SENDING PACKET\n");
		}
		usleep(16670*TIMESCALE);
		//usleep(1000000.0/fps);
	}
	printf("************TOTAL MOVE TIME : %f************",(float)frame/60);//int cor_i=0;	
	usleep(16670*15);
	//for(;cor_i<120;cor_i++)
	//cor_z[cor_i]*=-1;
	//com_z*=-1;
	//past_com_z*=-1;
	//future_com_z*=-1;
	//printf("******************************************************************************************\n");
	// COUNTER+=STEP*2;
	COM_INC+=(yrin-yrfi);
	return EXIT_SUCCESS;
}


int pingtest()
{

	//printf("pinging....\n");
	int failure[NOM][2];
	byte txp[15]={0xff,0Xff,0,0X02,0X01,0},rxp[4],chkff,count=2,exit=0,chksum,noat2=0;
	int er=0,ping_test_k=0,i,j,z,igm=0;
	//ftdi_usb_purge_buffers(&ftdic1);
	ipurge(MOTOR_DATA,&fd,&ftdic1);
	for (i = 0; i < NOM; i++)
	{
		count=2;
		//printf("in loop\n");
		//printf("i =  %d\n",i);
		//printf("\n");
		txp[2]=bid[i];
		failure[i][0]=bid[i];
		failure[i][1]=0;
		//printf("bid = %d , i =%d\n",bid[i],i);
		igm=0;
		exit=0;
		chksum=0x03+txp[2];
		chksum=~chksum;
		txp[5]=chksum;
		printf("Checking Motor ID : %d\n",txp[2]);
		for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
		{
			if(txp[2]==ignore_ids[ping_test_k])
			{
				igm=1;
				printf("*****Motor Pinging Ineffectual %d*****\n",ignore_ids[ping_test_k]);
				er+=ignore_ids[ping_test_k];
				break;
			}
		}
		for(noat=0;noat<(MOTOR_DATA==0?4:4000);noat++)
		{
			//printf("NOAT %d \n",noat);
			//ftdi_write_data(&ftdic1,txp,txp[3]+4);
			iwrite(MOTOR_DATA,&fd,&ftdic1,txp,txp[3]+4);
			noat2=0;
			while(noat2<5)
			{
				noat2++;
				//printf("COUNT:%d \n",noat2);
				//if((ftdi_read_data(&ftdic1,&chkff,1)>0)&&chkff==0xff)
				if((iread(MOTOR_DATA,&fd,&ftdic1,&chkff,1)>0)&&chkff==0xff)
				count--;
				else
				count=2;
				//printf("pack: %d\n",chkff);
				
				if(count==0)
				{
					//ftdi_read_data(&ftdic1,rxp,4);
					iread(MOTOR_DATA,&fd,&ftdic1,rxp,4);
					count=2;
					printf("DATA READ : %d %d %d %d\n",rxp[0],rxp[1],rxp[2],rxp[3]);
					chksum=0;
					for(z=0;z<3;z++)
					chksum+=rxp[z];
					chksum=~chksum;
					if(chksum!=rxp[3])
					{
						printf("ERROR: Packet checksum error\n");
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}
					else if(rxp[0]!=txp[2])
					{
						printf("ERROR: Error Pinging Motor\n");
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}		
					else if(rxp[2]!=0)
					{
						printf("ERROR: Motor Error :%d\n",rxp[2]);
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}
					else
					{
						printf("SUCCESS PINGING MOTOR : %d\n",rxp[0]);
						if(igm==0)
						er+=rxp[0];
						exit=1;
						failure[i][1]=1;
						
					}
					//printf("Er:%d\n",er);
					break;
				}
				
			}
			//printf("ER %d\n",er);
			if(exit==1)
			break;
		}
		printf("\n\n");
	}
	printf("\n\nMOTORS NOT RESPONDING\n");
	for(i=0;i<NOM;i++)
	{
		//printf("i=%d",i);
		if(failure[i][1]==0)
		{
			printf("%d",failure[i][0]);
			for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
			{
				if(failure[i][0]==ignore_ids[ping_test_k])
				printf(" <- IGNORED");
			}
			printf("\n");
		}
	}
	return ((er==440)?1:0);
	

}

void initialize_acyut()
{
	tx_packet[0]=0xff;
	int adchksum=7+NOM*3;
	tx_packet[1]=0xff;
	tx_packet[2]=0xfe;
	tx_packet[3]=NOM*3+4;
	tx_packet[4]=INST_SYNC_WRITE;
	tx_packet[5]=0x1e;
	tx_packet[adchksum]=0xff;
	tx_packet[6]=0x02;
	int i_init,h=0;
	for(i_init=0;i_init<NOM;i_init++)
	{
		tx_packet[7+i_init*3]=bid[i_init];
	}
	float val[5];
	float *fval;
	int value;
	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	for(i_init=0;i_init<NOM;i_init++)
	{
		if(tx_packet[7+i_init*3]==0||tx_packet[7+i_init*3]==20)
		{
			value=val[0]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==1||tx_packet[7+i_init*3]==21)
		{
			value=val[1]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==2||tx_packet[7+i_init*3]==12||tx_packet[7+i_init*3]==22||tx_packet[7+i_init*3]==32)
		{
			value=val[2]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==3||tx_packet[7+i_init*3]==23)
		{
			value=val[3]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==5||tx_packet[7+i_init*3]==25)
		{
			value=val[4]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==4||tx_packet[7+i_init*3]==24)
		{
			value=rotation(tx_packet[7+i_init*3],0)+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==6||tx_packet[7+i_init*3]==26)
		{
			value=512+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==7)
		{
			value=698+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==27)
		{
			value=325+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==8)
		{
			value=500+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==28)
		{
			value=512+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==9)
		{
			value=150+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==29)
		{
			value=865+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==10)
		{
			value=823+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==30)
		{
			value=200+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==15)
		{
			value=1863+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==16)
		{
			value=2048+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==17)
		{
			value=546+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==18)
		{
			value=588+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		
		
	}
	//for(i_init=0;i_init<=adchksum;i_init++)
	//printf("%d\t",tx_packet[i_init]);
	for(i_init=2;i_init<adchksum;i_init++)
	{
		tx_packet[adchksum]-=tx_packet[i_init];
	}
	speed(0);	
	//if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)>0)
	printf("INTIALIZED\n");
	sleep(5);
	speed(1);	
	for(i_init=0;i_init<128;i_init++)
	{
		last_tx_packet[i_init]=tx_packet[i_init];
	}

}

#ifdef TESTING
int main()
{
	bootup_files();
	char ch;
	if(!pingtest())
	{
		printf("Bypass ping? (y/n)\n");
		scanf("%c",&ch);
		if(ch=='n')
		return EXIT_FAILURE;
	}
	
	char STR[4];
	//itoa(COUNT,STR,10);
	
	
	
	// byte read;
	//torque(0);
	//pingtest();
	
	int flag=1;
	// COUNTER=0;
	//ftdi_set_latency_timer(&imu,1);
	//ftdi_read_data_set_chunksize(&imu,4096);
	initialize_acyut();
	imu_initialize();
	walk3(1-flag,0,STEP,0,-STEP,0,0,0,0,0,0,0,0);
	usleep(16670*10);
	
	while(!kbhit2())
	{
		//printf("%d\n",read_position(5));
		walk3(flag,-STEP,STEP,STEP,-STEP,0,0,0,0,0,0,0,0);
		flag=1-flag;
		usleep(16670*10);
		//projection_system(flag);
		
	}
	walk3(flag,-STEP,0,STEP,0,0,0,0,0,0,0,0,0);
	usleep(16670*30);
	compliance(0,32,32);
	
	f=fopen("WALK","w");
	if(f!=NULL)
	{
		fwrite(&zmp,sizeof(zmp),1,f);
		fclose(f);
	}
	close_files();


}
#endif


//what the function does
//takes r and theta 
//
//if |theta|>15 turns accordingly else(ie. |theta|,15)
////check r and walk one step//walk(-prev_step to 0)
		//turn right to walktheta
//return
#define MINTHETA 15//degrees
#define STEPLEN 20//mm
#define STEPANGLE 20//degrees
#define LEFT 0
#define RIGHT 1
float prev_step;
int leg_flag=0;

void callWalk(float walkr,float walktheta)
{
	//theta is big enough to turn
	//positive theta==right,negative==left 
	if(walktheta>MINTHETA)//positive angle
	{
		if(prev_step!=0)
		{
		walk3(leg_flag,-prev_step,0,prev_step,0,0,0,0,0,0,0,0,0);
		leg_flag=1-leg_flag;
		}
		//turn right to walktheta
		if(walktheta>STEPANGLE)//turn STEPANGLE
		{
			walk3(RIGHT,0,0,0,0,0,0,0,0,0,STEPANGLE,0,0);
			walk3(LEFT,0,0,0,0,0,0,0,0,0,0,STEPANGLE,0);
		}
		else//walktheta<STEPANGLE
		{
			walk3(RIGHT,0,0,0,0,0,0,0,0,0,walktheta,0,0);
			walk3(LEFT,0,0,0,0,0,0,0,0,0,0,walktheta,0);
		}//turn walktheta
	}	
	else if(walktheta<(-MINTHETA))//negative theta 
	{
		
		if(prev_step!=0)
		{
		walk3(leg_flag,-prev_step,0,prev_step,0,0,0,0,0,0,0,0,0);
		leg_flag=1-leg_flag;
		}
		//turn left to -walktheta
		if(fabs(walktheta)>STEPANGLE)//turn STEPANGLE
		{
			walk3(LEFT,0,0,0,0,0,0,0,0,0,STEPANGLE,0,0);
			walk3(RIGHT,0,0,0,0,0,0,0,0,0,0,STEPANGLE,0);
		}
		else//walktheta<STEPANGLE
		{
			walk3(LEFT,0,0,0,0,0,0,0,0,0,-walktheta,0,0);
			walk3(RIGHT,0,0,0,0,0,0,0,0,0,0,-walktheta,0);
		}//turn walktheta
	}
	
	else//-MINTHETA<theta<MINTHETA
	{
		if(walkr>STEPLEN)
		{
			//walk-STEPLEN
			walk3(leg_flag,-prev_step,STEPLEN,prev_step,-STEPLEN,0,0,0,0,0,0,0,0);
			leg_flag=1-leg_flag;
			prev_step=STEPLEN;
			
		}
		else
		{
			//walk-r
			walk3(leg_flag,-prev_step,walkr,prev_step,-walkr,0,0,0,0,0,0,0,0);
			leg_flag=1-leg_flag;
			prev_step=walkr;
		}
		
		
	}
}

