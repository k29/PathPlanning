#include <stdio.h>
#include <unistd.h>
#include <ftdi.h>
#include <inttypes.h>
#include <stdlib.h>
#include <termios.h>

#define usb_serial "A4007sgE"//"A7003N1d"//"A4007sgE"//"A700eSSZ"//
#define id1 1
#define id2 2 
#define broadcast_id 0xfe



//0=fwd ,1=backward,3=left ,2 = right

static struct termios oldt;

void restore_terminal_settings(void)
{
    tcsetattr(0, TCSANOW, &oldt);  /* Apply saved settings */
}

void disable_waiting_for_enter(void)
{
    struct termios newt;

    /* Make terminal read 1 char at a time */
    tcgetattr(0, &oldt);  /* Save terminal settings */
    newt = oldt;  /* Init new settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
    atexit(restore_terminal_settings); /* Make sure settings will be restored when program ends  */
}

struct ftdi_context ftdi;
typedef uint8_t byte;
byte packet[18];
byte packet_recieved[18];
int packet_length;
int len_to_read;



void ftdi_bootup();
void ftdi_close();
int checksum(int size);
int ping(int id);
int read_packet(byte *packet, int length);
int write_packet(byte *packet, int length);
int sync_write_data_motor(int mode, int speed);
int write_data_motor(int mode, int speed,int id);

int testbot_bootup()
{
//	
	// char ch;
	ftdi_bootup();
	
// 	printf("motor1 ping.\n");	
// 	ping(id1);
// 	printf("motor2 ping.\n");
// 	ping(id2);
// 	printf("\n\n");	
// 	printf("running.\n");
// //	write_data_motor(0,500,id2);
// 	disable_waiting_for_enter();	
// 	while(1)
// 	{	
// 		ch = getchar();
// 		if(ch=='w')
// 		{	sync_write_data_motor(0, 500);
// 			ch='\0';
// 		}
// 		else if(ch=='s')
// 		{	sync_write_data_motor(1, 500);
// 			ch='\0';
// 		}
// 		else if(ch=='a')
// 		{	sync_write_data_motor(2, 500);
// 			ch='\0';
// 		}
// 		else if(ch=='d')
// 		{	sync_write_data_motor(3, 500);
// 			ch='\0';
// 		}
// 		else
// 		{	sync_write_data_motor(0, 0);
		
// 		}
// 	}
// 	ftdi_close();

// 	return 0;
}

void ftdi_bootup()
{

	if(ftdi_init(&ftdi)!=0)
	{
		printf("[testbot]ftdi - could not be opened.\n");
	}
	else
		printf("[testbot]booting up.\n");

	
//	int ftdi_usb_open_desc	(	struct ftdi_context * 	ftdi,int vendor,int product,const char * description,const char * serial )		

	if(ftdi_usb_open_desc(&ftdi,0x0403, 0x6001,NULL/*description*/,usb_serial)!=0)
	{
		printf("[testbot]ftdi error.\n");
	}
	else
	{
		printf("[testbot]ftdi opened.\n");
		ftdi_set_baudrate(&ftdi,1000000);
	}

}

void ftdi_close()
{

	if(ftdi_usb_close(&ftdi)!=0)
	{
		printf("\nusb could not be closed.\n");

	}
	else
		printf("\nusb closed.\n");
	
	 ftdi_deinit(&ftdi);	
	
}



int checksum(int size)
{
	int result=0;
	int i=2;
	while(i<size)
	{		
		result+=packet[i];
		i++;	
	}
//	printf("result=%d\n",result);
//	if(result>255)
//	{
//		result=result%100;	
//	}
	
	result=~result;	
	
//	printf("result2=%d\n",result);
	return result;
}

int sync_write_data_motor(int mode, int speed)
{	
	
	byte mot1_lowdata,mot1_highdata,mot2_lowdata,mot2_highdata;
	if(mode==0)	//fwd	
	{	mot1_lowdata=speed&0xff;
		mot1_highdata=speed>>8;
		
		mot2_lowdata=(speed+1024)&0xff;
		mot2_highdata=(speed+1024)>>8;
			
	}
	
	else if(mode==1)	//backwd	
	{	mot1_lowdata=(speed+1024)&0xff;
		mot1_highdata=(speed+1024)>>8;
		
		mot2_lowdata=speed&0xff;
		mot2_highdata=speed>>8;
	}
	
	else if(mode==2)	//right	
	{	mot1_lowdata=speed&0xff;
		mot1_highdata=speed>>8;
		
		mot2_lowdata=speed&0xff;
		mot2_highdata=speed>>8;
	}
	else if(mode==3)	//left	
	{	mot1_lowdata=(speed+1024)&0xff;
		mot1_highdata=(speed+1024)>>8;
		
		mot2_lowdata=(speed+1024)&0xff;
		mot2_highdata=(speed+1024)>>8;
	}


	
	packet[0]=0xff;		//header
	packet[1]=0xff;		//header
	packet[2]=broadcast_id;		//id
	packet[3]=10;		//length
	packet[4]=0x83;		//instruction
	packet[5]=0x20;		//Start address to write Data
	packet[6]=0x02;		//Length of Data to write
	packet[7]=id1;		//First ID of RX-28
	packet[8]=mot1_lowdata;		//First data of the first RX-28
	packet[9]=mot1_highdata;		//Second data of the first RX-28
	packet[10]=id2;		//ID of the second RX-28
	packet[11]=mot2_lowdata;		//First data of the second RX-28
	packet[12]=mot2_highdata;		//Second data of the second RX-28
	
	packet[13]=checksum(13);		//checksum
	
	packet_length=14;	
	write_packet(packet,packet_length);



}

void testBotWalk1(int cmd,int speed)
{
		printf("[testbotcallwalk]cmd=%d with speed=%d\n",cmd,speed);

	sync_write_data_motor(cmd,speed);
}
int write_data_motor(int mode, int speed,int id)
{	
	
	byte mot1_lowdata,mot1_highdata,mot2_lowdata,mot2_highdata;
	if(mode==0)	//fwd	
	{	mot1_lowdata=speed&0xff;
		mot1_highdata=speed>>8;
		
		mot2_lowdata=speed&0xff;
		mot2_highdata=speed>>8;
			
	}



/*
	packet[0]=0xff;		//header
	packet[1]=0xff;		//header
	packet[2]=1;//id;		//id
	packet[3]=0x04;		//length
	packet[4]=0x03;		//instruction
	packet[5]=0x18;		//Start address to write Data
//	packet[6]=0x02;		//Length of Data to write
//	packet[7]=id1;		//First ID of RX-28
	packet[6]=1;//mot1_lowdata;		//First data of the first RX-28
//	packet[7]=0;//mot1_highdata;		//Second data of the first RX-28
//	packet[10]=id2;		//ID of the second RX-28
//	packet[11]=mot2_lowdata;		//First data of the second RX-28
//	packet[12]=mot2_highdata;		//Second data of the second RX-28
	
	packet[7]=checksum(7);		//checksum
	
	packet_length=8;	
	write_packet(packet,packet_length);

	len_to_read=9;
	read_packet(packet_recieved,len_to_read);











*/



	packet[0]=0xff;		//header
	packet[1]=0xff;		//header
	packet[2]=2;//id;		//id
	packet[3]=0x05;		//length
	packet[4]=0x03;		//instruction
	packet[5]=0x20;		//Start address to write Data
//	packet[6]=0x02;		//Length of Data to write
//	packet[7]=id1;		//First ID of RX-28
	packet[6]=0;//mot1_lowdata;		//First data of the first RX-28
	packet[7]=0;//mot1_highdata;		//Second data of the first RX-28
//	packet[10]=id2;		//ID of the second RX-28
//	packet[11]=mot2_lowdata;		//First data of the second RX-28
//	packet[12]=mot2_highdata;		//Second data of the second RX-28
	
	packet[8]= checksum(8);		//checksum
	
	packet_length=9;	
	write_packet(packet,packet_length);

	len_to_read=9;
	read_packet(packet_recieved,len_to_read);


}

int ping(int id)
{

	packet[0]=0xff;		//header
	packet[1]=0xff;		//header
	packet[2]=id;		//id
	packet[3]=0x02;		//length
	packet[4]=0x01;		//instruction
	packet[5]=checksum(5);		//checksum
	

	packet_length=6;	
	write_packet(packet,packet_length);

	len_to_read=6;
	read_packet(packet_recieved,len_to_read);

}

int read_packet(byte *packet, int length)
{	int attempts=0;
	int i,a;
	printf("DATA recieved:\n");
 	while(attempts<100)
	{	attempts++;
		if((a=ftdi_read_data(&ftdi,packet_recieved,length))>0)			
		{
			printf("bytes rd(no error):%d\n",a);
			for(i=0;i<length;i++)	
			{	if(i==4)			
					printf("(err)");		
								
				printf("%x\t",packet_recieved[i]);
			}		
			printf("\n");	 
			ftdi_usb_purge_buffers(&ftdi);			
			break;
		}
		else 
		{	printf("NO DAta\n");
			printf("bytes rd(error):%d\n",a);		
		}	
	}


}

int write_packet(byte *packet, int length)
{	int a=0,i;	
	printf("[testbot]Writing data...\n");
	//for(i=0;i<length;i++)			
	//	printf("%x\t",packet[i]);
	
	//printf("\n");
	a=ftdi_write_data(&ftdi,packet,length);	

	if(a>0)	
		printf("[testbot]bytes written(no error):%d\n",a);
	else 
		printf("[testbot]write error.\n");

}
