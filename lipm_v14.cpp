#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <queue>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <ftdi.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#define MASS (5.733)
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
#define sgn( a ) ( ((a) > 0) ? (1) : ( ((a) < 0) ? -1 : 0 ) )
#define D2R (3.14/180)
#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A7003N1d"  //"A700eSSZ" //"A900fDpz"//"A900fDhp" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8 //A700eST2
#define serialimu "A8004Yt8" //"A8008c3o" 
long int COUNTER=0;
long int COM_INC=0;
int COUNT;
int walk_count=0;
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
float mval[5];
	
///////////IMU DECLARATIONS//////////////////////////
/// GEN DECLARATIONS
int imu_init=0;
int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length);
int iread(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length);
int ipurge(int choose,int *termios_pointer, struct ftdi_context *ftdi_pointer);

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


struct ftdi_context imu;

 
FILE *f;
FILE *f1;
int fd1,fb,imu_t;
struct ftdi_context ftdic1,ftdic2;  //ftdic1=usb2dyn   ftdic2=bluetooth

byte tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
byte last_tx_packet[128]={0};

byte* gui_packet;
int noat=0;
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
}coordinates;

typedef struct
{
	long int nof;
	coordinates coods[6000];
	coordinates left[6000];
	coordinates right[6000];
}zmp_file;

zmp_file zmp;

///////////////////////////
//////////////////// WALKING DEFINITIONS///////////////////////////

float Y_END=0,YR_END=0,Z_END=0,ZR_END=0,THETA_END=0,THETAR_END=0;
float com_y;
float com_z,future_com_z;
int dsp=1;
int touchdown=1;
int pauseSeq=0;
float r_leg_coods[3];


	
////////////////////////////////////////////////////////////////////
///////////////////// FORWARD KINEMATICS, INVERSE KINEMATICS , COM GENERATION , ZMP CALCULATION DEFINITIONS
float left_joints[7][3],right_joints[7][3];
float left_com[3]={0},right_com[3]={0};
float com[3]={0},rot_com[3]={0},rot_hip[3]={0},rot_rleg[3],hip[3]={0};
int offset[33]={0};
int motor_val[33]={0};
int target_left[6],target_right[6];
float STEP=0;
	
/////////////////////////////////////////////////////////////////////
float* generateik(float x,float y,float z,int i );
float scurve(float in,float fi,float t, float tot);
void initialize_acyut();
void imucal();
void get_imu_data();
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

void changemode(int dir)   // do not display characters in terminal
{
  static struct termios oldt, newt;

  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}




int get_lb( int word )
{
  unsigned short temp;
  temp = word & 0xff;
  return (int)temp;
}

int get_hb( int word )
{
  unsigned short temp;
  temp = word & 0xff00;
  temp = temp >> 8;
  return (int)temp;
}
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
		//i=6;
		//printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
	
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
		//i=6;	
		//printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
	
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
	float torso_wt=2734.9+300;
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
	
	//printf(" %f %f %f\n",r_leg_coods[0],r_leg_coods[1],r_leg_coods[2]);
	hip[0]=hip_coods[0];
	hip[1]=hip_coods[1]+(-50)*(1-leg)+(50)*leg;
	hip[2]=hip_coods[2];
	//printf("\n\n%f %f %f\n",hip[0],hip[1],hip[2]);
	

	//// GETTING ANGLES FROM IMU
	get_imu_data();
	roll=current_angle[0]*3.14/180;
	pitch=-current_angle[1]*3.14/180;
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
	
	//printf("RLEG %f %f %f\n",rot_rleg[0],rot_rleg[1],rot_rleg[2]);
	if(rot_rleg[0]<=10)
	{
		//printf("!!!!!TOUCHDOWN!!!!!\n");
		touchdown=rot_rleg[0];
	}
	else
	touchdown=10;
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
		close(fd1);
	}

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
	//for(initialize_ids_i=0;initialize_ids_i<30;initialize_ids_i++)
	//printf("%d\n",bid[initialize_ids_i]);
}
	
void set_offset()
{
	if(acyut==4)
	{
		offset[0]=0+15;
		offset[1]=235-50 + 25; 
		offset[2]=325+30; 
		offset[3]=180+20-20-20;//232-20-25-25-10;    // 142s  
		offset[4]=-10; 
		offset[5]=-15+10;//+7; 
		offset[6]=346-40; 
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0;//-(1023-865);//-150; 
		offset[10]=0; 	 
		offset[11]=0; 
		offset[12]=274+30+30; 
		offset[13]=0; 
		offset[14]=0; 
		offset[15]=-119; 
		offset[16]=42; 
		offset[17]=0; 
		offset[18]=-150; 
		offset[19]=0; 
		offset[20]=24-15;
		offset[21]=224-50 + 25; 
		offset[22]=343+30+10; 
		offset[23]=383+10-20-20;//-20; 
		offset[24]=-2; 
		offset[25]=-15+7+10-20;//-7; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]=0;// 90*1023/300; 
		offset[29]=0;//s1023-865; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=273+30;
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

int bootup_files_walk()
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
		if((fd1= open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
		{	
			ret_motor=-1;
			printf("Serial Port unavailable\n");
			close_files();
			exit(0);
		}
		else
		printf("Motor Line Opened By TERMIOS\n");
 		tcgetattr(fd1, &options);
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
//		tcflush(fd1, TCIFLUSH);
  		tcsetattr(fd1, TCSANOW, &options);
  		fcntl(fd1, F_SETFL, FNDELAY);
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
   		//ftdi_set_interface(&imu,2);
  		if ((ret_imu= ftdi_usb_open_desc (&imu, 0x0403, 0x6001, NULL, serialimu)) < 0)
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
			exit(0);
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
//		tcflush(fd1, TCIFLUSH);
  		tcsetattr(imu_t, TCSANOW, &imu_options);
  		fcntl(imu_t, F_SETFL,0);
		
	}
	else
	printf("UNKNOWN IMU CONNECTION\n");
	
	if(ret_motor>=0&&ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

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
		    if(iwrite(MOTOR_DATA,&fd1,&ftdic1,speed_packet,speed_packet[3]+4)<0)
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

		    if(iwrite(MOTOR_DATA,&fd1,&ftdic1,speed_packet,speed_packet[3]+4)<0)
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
			//printf("ANGLE %d : %lf\t",k,imu_angle[k]);
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
		current_angle[i]=held_angle[i]+gyro_angle[i];	
		//current_angle[i]=imu_angle[i];
		//printf("ANGLE[%d] = %f\t",i,current_angle[i]);
	}
//	printf("\n");
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


int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	/*int i=0;
	
	f=fopen("check","a");
	if(f!=NULL)
	{
		for(i=0;i<length;i++)
		{
			printf("%x\t",packet[i]);
			printf("\n");
			fprintf(f,"%02x",packet[i]);
			//fprintf(f,"\n");
		}
		fclose(f);
	}
	*/
	switch(choose)
	{
		case 0: return ftdi_write_data(ftdi_pointer,packet,length);
			break;
		case 1 :return write(*termios_pointer,packet,length);
			break;
		default : return -99999;
	}
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

int high_punch=150;
int low_punch=0;

int compliance(int leg,int cp,int cpr)
{
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
	if(iwrite(MOTOR_DATA,&fd1,&ftdic1,compliance_packet,compliance_packet[3]+4)<0)
	//printf("Compliance Written\n");
	//else
	{
		//printf("ERROR : Compliance\n");
	}
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
	
	if(leg_l>2*b_len+l22)
	return mval;
	
	cons[0]=1675;
	cons[1]=2952+255;
	cons[2]=2008+973;
	cons[3]=1459+255;
	cons[4]=1703;
	
	// CALCULATING LEG LENGTH
	float val=(leg_l-l22)/(2*b_len);
	if(val<=1&&val>=-1)
	{
		theta[3]=acos(val); // angle for motor 3 from XZ Acyut plane 
		theta[2]=asin(val);	// angle for motor 2 12 from YZ Acyut plane 
		theta[1]=theta[3];	// angle for motor 1 from XZ Acyut plane 
	}
	else
		return mval;
	// CALCULATING LEG POSITION
	if(y<=x)
		phi[3]=atan(y/x);
	else
		return mval;
	if(z<=x)
		phi[5]=atan(z/x);
	else
		return mval;	
	phi[1]=phi[3];
	phi[0]=phi[5];
	
	// GETTING ANGLES
	//printf("%lf asdjhkdshdk\n",avg_angle[1]);
	ang[0]=phi[0];//+(current_angle[1]*pow(-1,leg))*0.2/r2d;
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
	if(iwrite(MOTOR_DATA,&fd1,&ftdic1,torque_packet,torque_packet[3]+4)>0)
	printf("TORQUE PACKET SENT\n");
	else
	printf("ERROR : TORQUE\n");
	
	
}

int generatePacket()
{
	int packet_i=0;
	int adchksum=NOM*3+7;
	tx_packet[0]=0xff;
	tx_packet[1]=0xff;
	tx_packet[2]=0xfe;
	tx_packet[3]=NOM*3+4;
	tx_packet[4]=INST_SYNC_WRITE;
	tx_packet[5]=0x1e;
	tx_packet[6]=0x02;
	tx_packet[adchksum]=0xff;
	for(packet_i=0;packet_i<NOM;packet_i++)
	{
		tx_packet[3*packet_i+7]=bid[packet_i];
		tx_packet[3*packet_i+8]=get_lb(motor_val[bid[packet_i]]);
		tx_packet[3*packet_i+9]=get_hb(motor_val[bid[packet_i]]);
		//printf(" %d %d\n",bid[packet_i],motor_val[bid[packet_i]]);
	}
	
	for(packet_i=2;packet_i<adchksum;packet_i++)
	tx_packet[adchksum]-=tx_packet[packet_i];
	
	return 1;
	
}

	
	
int pingtest()
{
	int failure[NOM][2];
	byte txp[15]={0xff,0Xff,0,0X02,0X01,0},rxp[4],chkff,count=2,exit=0,chksum,noat2=0;
	int er=0,ping_test_k=0,i,j,z,igm=0;
	ipurge(MOTOR_DATA,&fd1,&ftdic1);
	for (i = 0; i < NOM; i++)
	{
		count=2;
		txp[2]=bid[i];
		failure[i][0]=bid[i];
		failure[i][1]=0;
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
			iwrite(MOTOR_DATA,&fd1,&ftdic1,txp,txp[3]+4);
			noat2=0;
			while(noat2<5)
			{
				noat2++;
				if((iread(MOTOR_DATA,&fd1,&ftdic1,&chkff,1)>0)&&chkff==0xff)
				count--;
				else
				count=2;
				
				if(count==0)
				{
					iread(MOTOR_DATA,&fd1,&ftdic1,rxp,4);
					count=2;
					//printf("DATA READ : %d %d %d %d\n",rxp[0],rxp[1],rxp[2],rxp[3]);
					chksum=0;
					for(z=0;z<3;z++)
					chksum+=rxp[z];
					chksum=~chksum;
					if(chksum!=rxp[3])
					{
						printf("ERROR: Packet checksum error\n");
						if(igm==1)
						{
							exit=1;
						}
					}
					else if(rxp[0]!=txp[2])
					{
						printf("ERROR: Error Pinging Motor\n");
						if(igm==1)
						{
							exit=1;
						}
					}		
					else if(rxp[2]!=0)
					{
						printf("ERROR: Motor Error :%d\n",rxp[2]);
						if(igm==1)
						{
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
					break;
				}
				
			}
			if(exit==1)
			break;
		}
		printf("\n\n");
	}
	printf("\n\nMOTORS NOT RESPONDING\n");
	for(i=0;i<NOM;i++)
	{
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
	int h=0;
	float val[5];
	float *fval;
	int value;

	//fval=generateik(390,30,6,0);
	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	motor_val[0]=val[0]+offset[0];
	motor_val[1]=val[1]+offset[1];
	motor_val[2]=val[2]+offset[2];
	motor_val[3]=val[3]+offset[3];
	motor_val[4]=rotation(4,0)+offset[4];
	motor_val[5]=val[4]+offset[5];
	motor_val[6]=512+offset[6];
	motor_val[7]=698+offset[7];
	motor_val[8]=512+offset[8];
	motor_val[9]=150+offset[9];
	motor_val[10]=823+offset[10];
	motor_val[12]=val[2]+offset[12];
	
	//printf("%lf \n",val[4]);
	
	motor_val[15]=1863+offset[15];
	motor_val[16]=2048+offset[16];
	motor_val[17]=546+offset[17];
	motor_val[18]=588+offset[18];
	
	//fval=generateik(390,-30,6,0);
	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	motor_val[20]=val[0]+offset[20];
	motor_val[21]=val[1]+offset[21];
	motor_val[22]=val[2]+offset[22];
	motor_val[23]=val[3]+offset[23];
	motor_val[24]=rotation(24,0)+offset[24];
	motor_val[25]=val[4]+offset[25];
	motor_val[26]=512+offset[26];
	motor_val[27]=325+offset[27];
	motor_val[28]=512+offset[28];
	motor_val[29]=865+offset[29];
	motor_val[30]=200+offset[30];
	motor_val[32]=val[2]+offset[32];
	
	speed(0);
	if(generatePacket()==1 && iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4)>0)
	printf("INITIALIZED\n");
	sleep(5);
	speed(1);
	
	
}
/*
void dribble(int leg, float time)
{
	time =0.6;
	int fps = 80;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	
	float height = 390;
	float lift   = 25;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = 0.15;
	float stopX  = time-0.15;
	float xPhase =-3.14/2;
	
	float feetSeperation = 20;
	float zmax   = 30;
		
	float x,xr,y,yr,z,zr;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	
	for(walkTime = 0.0; walkTime<=time; walkTime +=timeInc)
	{
		if(walkTime>startX && walkTime < stopX)
		x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
		else 
		x=height;
	
		xr = height;
		
		z  = feetSeperation;
		
		/* 
		*Basic LIPM Implementation Z Direction
		*-- Good results 
		*-- Simply converting all the given time to reach a particular Zmax value. Leads to input velocities which do not match with the current
		*-- Bot does some amount of shuffling 
		*-- A time of 0.8 secs was used and seperation of 20 lead to good results 
		*-- Modified IK Values to prevent movenent of 5 motors.The values must be changed if the seperation is altered and are equal to init val
		///
		
		///zr = feetSeperation -zmax * (cosh(-time/2)-cosh(walkTime - time/2))/(cosh(-time/2) -1);  
		
		y  = 0;
		yr = 0;
		
		printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		fval=generateik(x,y,z,leg);
		
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,0)+offset[20*leg+walk_i];
				continue;
			}
			else if(walk_i==5)
			{
				motor_val[20*leg+walk_i]=1743+offset[20*leg+walk_i];
				target_i++;
				continue;
			}
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
			
		}
		
		fval=generateik(xr,yr,zr,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,0)+offset[20*(1-leg)+walk_i];
				continue;
			}
			else if(walk_i==5)
			{
				motor_val[20*(1-leg)+walk_i]=1743+offset[20*(1-leg)+walk_i];
				target_i++;
				continue;
			}
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		if(generatePacket()==1)
		iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4);
		
		usleep(sleep);	
	}
	

	

}
*/	
enum SupportPhase{DSP,SSP};
float lYin  = -5;
float slYin = 5;
float vYin  = 40;
float vYfi  = 40;
float vTTar = 40;
float lZin = 10 ;
float slZin = 10;
float vZin  = -140;
float vZfi  = 140;

enum state{Y,YR,YV,Z,ZR,ZV,R,RR};

float endState[8];

float* dribble(int leg, float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin, float veloZin, float veloZfi, float legRotin, float legRotfi, float supLegRotin, float supLegRotfi)
{
//	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 0;
	float hipLength = 100;
	
	float Tc = sqrt(526.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	float D_dsp1Time = 0.05;
	float D_dsp2Time = 0.05;
	
//	printf("Tc\t\t%lf\n",Tc);
//	printf("legZin\t\t%lf\n",legZin);
//	printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	float zMax = 20;
	
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
	
//	printf("zMax\t\t%lf\n",zMax);
//	printf("sspZPhs\t\t%lf\n",sspZPhs);
//	printf("sspZTime\t%lf\n",sspZTime);
//	printf("sspZin\t\t%lf\n",sspZin);
//	printf("sspZSupin\t%lf\n",sspZSupin);
//	printf("sspZfi\t\t%lf\n",sspZfi);
//	printf("sspZSupfi\t%lf\n",sspZSupfi);
//	printf("dsp1Time\t%lf\n",dsp1Time);
//	printf("stepTime\t%lf\n",stepTime);
//	printf("LegZfi\t\t%lf\n",legZfi);
//	printf("supLegZfi\t%lf\n",supLegZfi);
	
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
	
//	printf("\n\n\n");
//	printf("legYin\t\t%lf\n",legYin);
//	printf("supLegYin\t%lf\n",supLegYin);
//	printf("sspYin\t\t%lf\n",sspYin);
//	printf("sspYSupin\t%lf\n",sspYSupin);
//	printf("sspYAmp\t\t%lf\n",sspYAmp);
//	printf("sspYPhs\t\t%lf\n",sspYPhs);
//	printf("sspYfi\t\t%lf\n",sspYfi);
//	printf("sspYSupfi\t%lf\n",sspYSupfi);
//	printf("legYfi\t\t%lf\n",legYfi);
//	printf("suplegYfi\t%lf\n",supLegYfi);
//	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
//	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
//	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
//	printf("P_sspZTime\t%lf\n",P_sspZTime);
//	printf("P_sspYin\t\t%lf\n",P_sspYin);
//	printf("P_legYin\t\t%lf\n",P_legYin);
	
	
	float height = 390;
	float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time;
	float stopX  = stepTime-dsp2Time;
	float xPhase =-3.14/2;
	
	
	float x,xr,y,yr,z,zr,phi,phiR;
	float yReal,yrReal,zReal,zrReal;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	compliance(leg,10,10);
	
	
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
			x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			xr = height;
			y  = linear(-sspYin,-sspYfi, walkTime-dsp1Time,sspTime);
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
			x  = height;
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
		//printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		///printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		
		if(leg==0)
		{
			zReal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yReal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrReal=zr*cos(phiR*3.14/180)-yr*sin(phiR*3.14/180);
			yrReal=zr*sin(phiR*3.14/180)+yr*cos(phiR*3.14/180);
		}
		else if(leg==1)
		{
			zReal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yReal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrReal=zr*cos(-phiR*3.14/180)+yr*sin(phiR*3.14/180);
			yrReal=-zr*sin(-phiR*3.14/180)+yr*cos(phiR*3.14/180);
			
		}
		
		
		
		fval=generateik(x,yReal,zReal+feetSeperation,leg);
			
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
		 	{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,phi)+offset[20*leg+walk_i];
				continue;
			}
			//else if(walk_i==5)
			//{
			//	if(state==DSP) 
			//	{
			//		sspMotVal = fval[target_i];
			//	 	motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//	}
			//	else if(state==SSP)
			//	
			//	motor_val[20*leg+walk_i]=sspMotVal+offset[20*leg+walk_i];
			//	
			//	target_i++;
			//	continue;
			//}
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
	
		fval=generateik(xr,yrReal,zrReal+feetSeperation,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,phiR)+offset[20*(1-leg)+walk_i];
				continue;
			}
			//else if(walk_i==5)
			//{
			//	if(state==DSP)
			//	{
			//		motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//		sspRMotVal = fval[target_i];
			//	}
			//	else if(state==SSP)
			//	
			//	motor_val[20*leg+walk_i]=sspMotVal+offset[20*leg+walk_i];
			//	
			//	
			//	target_i++;
			//	continue;
			//}
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		if(generatePacket()==1)
		iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4);
		
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



//float veloZin = -120;
//float veloZfi = 120;
/*void dribble(int leg, float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin)
{
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 0;
	float hipLength = 100;
	
	
	float Zmax = 30;
	float sspTime = 0.5;
	float dsp1Time = 0.05;
	float dsp2Time = 0.05;
	
	float sspLegZin    = legZin - veloZin*dsp1Time;
	float sspSupLegZin = supLegZin  + veloZin*dsp1Time;
	
	
	float Kz = 1;
	float K  = -1/sqrt(1 - pow(Zmax/sspSupLegZin,2));   // Location specifier
	float Mt = asinh((-sqrt((pow(Kz,2)-1)*pow(K,2)+1) - Kz*K)/(pow(K,2)-1));
	
	float Tc = sspTime/Mt;
	float Tz = K*Tc;
	float veloZin = K*sspSupLegYin;
	
	
	float veloRat = veloYfi/veloYin; 
	float Ty      = Tc * (veloRat - sqrt(1+pow(sinh(Mt),2)))/sinh(Mt);
	printf("Ty\t%lf\n",Ty);
	
	
	float stepTime = dsp1Time + sspTime + dsp2Time;
	float Tyd =-dsp1Time+Ty;
	
	float sspYAmp  = veloYin*sqrt(pow(Tc,2)-pow(Ty,2));
	float sspYPhs  = asinh(Ty/sqrt(pow(Tc,2)-pow(Ty,2)));
	float sspZAmp  = -veloZin * sqrt(pow(Tz,2)-pow(Tc,2));
	float sspZPhs  = - asinh(Tc/sqrt(pow(Tz,2)-pow(Tc,2)));
	
	float sspLegYin    = legYin + veloYin*dsp1Time + hipLength/2;
	float sspSupLegYin = supLegYin - veloYin*dsp1Time + hipLength/2;
	
	float sspLegYfi    = (-Tyd+dsp2Time)*veloYfi;
//	float sspSupLegYin = supLegYin - veloYin*dsp2Time;
	float sspSupLegYfi    = -sspYAmp * sinh(sspTime/Tc + sspYPhs);
	//float sspLegYfi = -sspSupLegYfi + veloYin*(dsp1Time+dsp2Time);
	//printf("sspLegYin\t%f\t",sspLegYin);
	//printf("sspSupLegYin\t%f\t",sspSupLegYin);
	//printf("sspLegYfi\t%f\t",sspLegYfi);
	//printf("sspSupLegYfi\t%f\n",sspSupLegYfi);
	
	
	float sspLegZfi    = veloZfi*(-Tzd+dsp2Time) - hipLength/2;
	float sspSupLegZfi = Kz*sspSupLegZin;//sspZAmp * cosh((sspTime-1.0/fps)/Tc + sspZPhs) - hipLength/2;
	float veloZfi2 	   = sspZAmp * sinh((sspTime)/Tc + sspZPhs)/Tc;
	
	printf("Velo Z2 %lf\tZ1\t%lf\n",veloZfi2,veloZfi);
	//printf("sspLegZin\t%lf\tsspLegZfi\t%lf\n",sspLegZin,sspLegZfi);	
	printf("sspSupLegZfi\t%lf\tsspSupLegZfi (exp)\t%lf\n",sspSupLegZfi,Kz*sspSupLegZin);	

	//printf("sspTime\t%lf\tdsp1Time\t%lf\tdsp2Time\t%lf\n",sspTime,dsp1Time,dsp2Time);
	//printf("Ty\t%lf\tVeloz\t%f\nm\t%lf]t",Ty,veloZfi2,m);
	//printf("sspLegYin\t%f\t",sspLegYin);
	//printf("AMP\t%lf\tPHASE\t%lf\tsspTIME\t%lf\tTIME\t%lf\td             spTIME\t%lf\tSYfi%lf\n",sspYAmp,sspYPhs,sspTime,dsp1Time,0.0,0.0);
	//printf("Vr\t%lf\tTy\t%lf\tsspTIME\t%lf\tdspTime\t%lf\tstepTIME\t%lf\tZfi\t%lf\n",veloRat,Ty,sspTime,dsp1Time,stepTime,sspZfi);
	//printf("sspZAmp\t%lf\tsspZPhs\t%lf\tsspZfi\t%lf\tZmax\t%lf\n",sspZAmp,sspZPhs,sspZfi,Zmax);
	//printf("Tz\t%lf\tTy\t%lf\tC%lf\tC!%lf\tC!!%lf\n",Tz,Ty,Tc*Tc/Ty,Tc,veloRat);
	//printf("Tz\t%lf\tTy\t%lf\n",sspTime2,sspTime);
	
	float height = 390;
	float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time;
	float stopX  = stepTime-dsp2Time;
	float xPhase =-3.14/2;
	
	float x,xr,y,yr,z,zr;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	printf("MOVE TIME %lf\n",stepTime);
	compliance(leg,10,10);
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

	//sspTime = ((float)((int)(sspTime*100))/100.0);
	//dsp1Time = ((float)((int)(dsp1Time*100))/100.0);
	//dsp2Time = ((float)((int)(dsp2Time*100))/100.0);
	//stepTime = ((float)((int)(stepTime*100))/100.0);
	
	///printf("SSPTime\t%lf\tTc\t%lf\tTy\t%lf\n",sspTime,Tc,Ty);
	float Zmax = sspZAmp - hipLength/2;
	printf("Zmax\t\%lf\n",Zmax);
	
	
	for(walkTime = 0.0/fps; walkTime<stepTime-0.0/fps; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = 0;//legYin - veloYin*walkTime;
			yr = 0;//supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime ;
			zr = supLegZin + veloZin*walkTime;
			printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			x  = height;// - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			xr = height;
			y  = 0;//scurve(sspLegYin,sspLegYfi, walkTime-dsp1Time,sspTime);
			yr = 0;//-sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspLegZin,sspLegZfi, walkTime-dsp1Time,sspTime);
			zr =  sspZAmp * cosh((walkTime-dsp1Time+1.0/fps)/Tc + sspZPhs) -hipLength/2;
			state = SSP;
			printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;
			xr = height;
			y  = 0;//sspLegYfi - veloYin*(walkTime-dsp1Time-sspTime);
			yr = 0;//sspSupLegYfi - veloYfi*(walkTime-dsp1Time-sspTime);
			z  = sspLegZfi - veloZfi*(walkTime-dsp1Time-sspTime);
			zr = sspSupLegZfi + veloZfi*(walkTime-dsp1Time-sspTime+1.0/fps);
			printf("DSP2\t");
		}
		else 
		{
			printf("************* SOMETHING WRONG*******************\n");
		}
		printf("X\t%lf\tXR\t%lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\n",x,xr,y,yr,z,zr);
		///printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		fval=generateik(x,y,z+feetSeperation,leg);
		
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
		 	{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,0)+offset[20*leg+walk_i];
				continue;
			}
			/*else if(walk_i==5)
			{
				if(state==DSP) 
				{
					sspMotVal = fval[target_i];
				 	motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
				}
				else if(state==SSP)
				
				motor_val[20*leg+walk_i]=sspMotVal+offset[20*leg+walk_i];
				
				target_i++;
				continue;
			}/////
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
	
		fval=generateik(xr,yr,zr+feetSeperation,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,0)+offset[20*(1-leg)+walk_i];
				continue;
			}
			/*else if(walk_i==5)
			{
				if(state==DSP)
				{
					motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
					sspRMotVal = fval[target_i];
				}
				else if(state==SSP)
				
				motor_val[20*leg+walk_i]=sspMotVal+offset[20*leg+walk_i];
				
				
				target_i++;
				continue;
			}////
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		if(generatePacket()==1)
		iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4);
		
		usleep(sleep);
		
	}
	YY=y;
	ZZ=z;
	ZZRR=zr;
	YYRR=yr;
	//exit(0);
}

*/
#define DRIBBLE_TIME 1

void lipm_generatorSSP(int leg, float legYin, float suplegYin, float veloYin, float sspTime)
{
	int fps=80;
	int sleep=(int)((float)1000000.0/(float)fps);
	
	float gravConstant = sqrt(9.81/(0.576*1));   // gravitational Acceleration/Pendulum Length
	printf("SSP_TIME   %lf\n",sspTime);
 	printf("Grav Cosnt %f\n",gravConstant); 

	
	float supLegYfi = -(-suplegYin*cosh(sspTime*gravConstant)+ veloYin/gravConstant*sinh(sspTime*gravConstant));
	
	float veloYfi   = veloYin*cosh(sspTime*gravConstant)+(-suplegYin*gravConstant)*sinh(sspTime*gravConstant);
	
	printf("RLEG Yi %lf\tYf %lf\n",suplegYin,supLegYfi);
	printf("STEP L %lf\t Yv %lf\t Ya %lf\t\n",-supLegYfi+suplegYin,veloYin,(-supLegYfi+suplegYin)/sspTime);
	printf("LEG  Y %lf\n",0.0);
	
	/*float amplitudeSY = sqrt(fabs(pow(veloYin/gravConstant,2)-pow(-suplegYin,2)));
	float phaseSY     = amplitudeSY!=0 ? asinh(-suplegYin/amplitudeSY) : 0;
	float sspTime     = amplitudeSY!=0 ? (acosh(veloYfi/(amplitudeSY*gravConstant))-phaseSY)/gravConstant : DRIBBLE_TIME;
	
	printf("Amplitude %lf\tPhase %lf\t Time %lf\n %lf \n",amplitudeSY,phaseSY,sspTime, acosh (veloYin/(amplitudeSY*gravConstant)));
	
	legYfi=sqrt(pow(legYin,2)+pow(sspTime,2)*(pow(veloYfi,2)-pow(veloYin,2)));	
	
	
	float startFracY = 0.1;
	float endFracY   = 0.1;
	
	
	float timeTest = -(suplegYin*gravConstant)/veloYin;
	
	timeTest = 2/(timeTest-1/timeTest);
	
	printf("Time Start %f \n",timeTest);
		
	
	float wtShift = 40;
	float startZ  = -acosh(wtShift);
	float stopZ   = acosh(wtShift);
	
	
	*/
	
	float height = 390;
	float lift   = 20;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = 0;
	float stopX  = sspTime;
	float xPhase =-3.14/2;
	
	
	float walkTime=0;
	float x=0,y=0,z=0,xr=0,yr=0,zr=0;
	float yr1=0;
	
	float *fval;
	int walk_i=0,target_i=0;
	
	long frameTime = 0;
	long initialTime = 0;
	gettimeofday(&tv,NULL);
	initialTime = tv.tv_sec*1000000 +  tv.tv_usec;
	//sspTime=0.5;
	
	for(walkTime=0;walkTime<=sspTime+1.0/fps;walkTime+=1.0/fps)
	{
		
		x  = height-(lift)*(sin(xfreq*(walkTime-startX)/(stopX-startX)+xPhase)+displacement)/(1+displacement);
		xr = height;
		
		//y  = sinh(linear(startY,stopY,walkTime,sspTime));
		//yr = sinh(linear(startYR,stopYR,walkTime,sspTime));
		//yr  = amplitudeSY*sinh(gravConstant*walkTime + phaseSY);
		yr = -(-suplegYin*cosh(walkTime*gravConstant)+(veloYin/gravConstant)*sinh(gravConstant*walkTime));
	//             -(-suplegYin*cosh(sspTime *gravConstant)+(veloYin/gravConstant)*sinh(sspTime*gravConstant));
	
		printf("Y %f\n",yr);
		
		//y  = linear(legYin,legYfi,walkTime-startFracY*sspTime,sspTime-(startFracY+endFracY)*sspTime);
		
		//z  = (wtShift-cosh(linear(startZ,stopZ,walkTime,sspTime)));
		//zr = -(wtShift-cosh(linear(startZ,stopZ,walkTime,sspTime)));
		
		//printf("TIME %f\tX %f\tY %f\tZ %f\tXR %f\tYR %f\tZR %f\n",walkTime,x,y,z,xr,yr,zr);

		fval=generateik(x,y*1000,z,leg);
		
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,0)+offset[20*leg+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
			
		}
		
		fval=generateik(xr,yr*1000,zr,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,0)+offset[20*(1-leg)+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		if(generatePacket()==1)
		iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4);
		gettimeofday(&tv,NULL);
	
		usleep(sleep);
		
	}
	gettimeofday(&tv,NULL);
	
	printf("MOVE TIME %ld \n",tv.tv_sec*1000000+tv.tv_usec-initialTime);
	
	
	
}



void lipm_generatorDSP(int leg, float veloY, float legYin, float suplegYin, float dspTime)
{
	int fps = 120;
	int sleep = 1000000.0/(float)fps;
	float height = 390;
	float walkTime = 0.0;
	
	float x,xr,y,yr,z,zr;
	
	int walk_i = 0, target_i=0;
	float *fval;
	
	for(walkTime = 0; walkTime < dspTime; walkTime += 1.0/fps)
	{
		x  = height;
		xr = height;
		
		z  = 0;
		zr = 0;
		
		y  = legYin + veloY*walkTime;
		yr = suplegYin + veloY*walkTime;
		
		printf("TIME %f\tX %f\tY %f\tZ %f\tXR %f\tYR %f\tZR %f\n",walkTime,x,y,z,xr,yr,zr);

		fval=generateik(x,y*1000,z,leg);
		
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,0)+offset[20*leg+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
			
		}
		
		fval=generateik(xr,yr*1000,zr,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,0)+offset[20*(1-leg)+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		if(generatePacket()==1)
		iwrite(MOTOR_DATA,&fd1,&ftdic1,tx_packet,tx_packet[3]+4);
		gettimeofday(&tv,NULL);
	
		usleep(sleep);
		
		
	
	}
	

}


/*void endwalk(int flag,float legYin, float supLegYin,float legZin, float supLegZin)
{
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 0;
	float hipLength = 100;
	float Tc = sqrt(476.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	float zMax = 35;
	
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
	
	float height = 390;
	float lift   = 30;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time;
	float stopX  = stepTime-dsp2Time;
	float xPhase =-3.14/2;
}*/	
int flag_behaviour=0;
float globalWalkR[2];

/*
int main_walk()
{
	bootup_files_walk();
	char ch;
	if(!pingtest())
	{
		printf("Bypass ping? (y/n)\n");
		scanf("%c",&ch);
		if(ch=='n')
		return EXIT_FAILURE;
	}
	
	#if PLOT==1
	FILE *f;
	f=fopen("Graphs/COUNT","r");
	if(f!=NULL)
	{
		fscanf(f,"%d",&COUNT);
		printf("COUNT : %d\n",COUNT);		
		fclose(f);
	}
	char STR[4];
	//itoa(COUNT,STR,10);
	sprintf(STR,"%d",COUNT);
	printf("%d\n",COUNT);
	COUNT++;
	printf("%d\n",COUNT);
	strcat(comfile,"Graphs/Com/COM");
	strcat(comfile,STR);
	strcat(leftfile,"Graphs/LeftLeg/LEFT");
	strcat(leftfile,STR);
	strcat(rightfile,"Graphs/RightLeg/RIGHT");
	strcat(rightfile,STR);
	expect_com[0]='\0';
	strcat(expect_com,"Graphs/ExpectedCOM/Expected");
	strcat(expect_com,STR);
	strcat(hipfile,"Graphs/Hip/Hip");
	strcat(hipfile,STR);
	strcat(normalrctn,"Graphs/NormalF/Normal");
	strcat(normalrctn,STR);
	
	printf("%s\n%s\n%s\n%s\n%s\n%s\n",comfile,leftfile,rightfile,hipfile,expect_com,normalrctn);
	
	f=fopen(comfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(leftfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(rightfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(expect_com,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(hipfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(normalrctn,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\n");
		fclose(f);
	}
	f=fopen("Graphs/COUNT","w");
	if(f!=NULL)
	{
		fprintf(f,"%d",COUNT);
		fclose(f);
	}
	#endif
	COUNTER=0;
	
	initialize_acyut();
	int flag=0;
	float *state;
	
	
	compliance(0,10,10);
	call_walk();
		state = dribble(flag,lYin,slYin,vYin,vYfi,lZin,slZin,vZin,vZfi,0,0,0,0);
	//	state = dribble(flag,lYin,slYin,25,25,lZin,slZin,vZin,vZfi,0,0,0,0);
		
		//lipm_generatorSSP(flag, 0, 0.010, -0.01 ,0.75);
		flag=1-flag;
		while(!kbhit2())
		{
			state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
			state = dribble(1-flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
	//		usleep(16670*2);	
			flag=flag;
		//	lipm_generatorSSP(flag, -30, 30, 30, -30, 10);
		//	flag=1-flag;
		//		walk5(flag,-STEP,STEP,STEP,-STEP,0,0,0,0,0,0,0,0);
		//	
		}
		compliance(0,32,32);
	
	#if PLOT==1
	f=fopen("WALK","w");
	if(f!=NULL)
	{
		fwrite(&zmp,sizeof(zmp),1,f);
		fclose(f);
	}
	close_files();
	f=fopen("script","w");
	if(f!=NULL)
	{	
		fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1  with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,leftfile,rightfile,expect_com,hipfile);
		//fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,expect_com);
		fclose(f);
		//system("gnuplot < \"script\" -persist");
	}
	f=fopen("script2","w");
	if(f!=NULL)
	{
		fprintf(f,"plot \"%s\" using 2 with lines,\"%s\" using 2 with lines, \"%s\" using 2 with lines\n",comfile,hipfile,expect_com);
		fclose(f);
		system("gnuplot < \"script2\" -persist");
	}
	f=fopen("script3","w");
	if(f!=NULL)
	{
		fprintf(f,"plot \"%s\" using 3 with lines,\"%s\" using 3 with lines, \"%s\" using 3 with lines\n",comfile,hipfile,expect_com);
		fclose(f);
		system("gnuplot < \"script3\" -persist");
	}
	f=fopen("script4","w");
	if(f!=NULL)
	{
		fprintf(f,"plot \"%s\" using 1 with lines\n",normalrctn);
		fclose(f);
		//system("gnuplot < \"script3\" -persist");
	}
	#endif		
}
*/
