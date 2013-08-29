#ifndef WIFITRANS_H
#define WIFITRANS_H

#include "commondefs.h"
#define NETWORK "192.168.1.255"
#define PORT 9876
#define FPS_WIFI 2.0

pthread_mutex_t mutex_wifitransmission=PTHREAD_MUTEX_INITIALIZER;
//Communication Function 
//compile with behaviour

Packet wtpack;


void convertTransmitData()
{
    pthread_mutex_lock(&mutex_wifitransmission);
    wtpack.behav_pack.flags=globalflags;

    wtpack.loc_pack.selfX=loc.selfX;
    //printf("PACK________selfx=%lf\n",wtpack.loc_pack.selfX);
    wtpack.loc_pack.selfY=loc.selfY;
    wtpack.loc_pack.selfAngle=loc.selfAngle;

    fd.updatePacket(wtpack.feat_pack);
    #ifdef DEBUGBEHAV
    pathobj.updatePathPacket(wtpack.path_pack,true);
    #endif
    #ifndef DEBUGBEHAV
    //printf("called update path packet nodebug\n");
    pathobj.updatePathPacket(wtpack.path_pack);
    #endif
    pthread_mutex_unlock(&mutex_wifitransmission);
}
void* main_wifitransmission(void*)
{
    //printf("[wificomm]Wifi thread created\n\n\n\n");
	struct sockaddr_in addr;
   	int fild;
	int broadcast = 1;
   	Packet localpack;
	
    if ((fild = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    if (setsockopt(fild, SOL_SOCKET, SO_BROADCAST, &broadcast,sizeof broadcast) == -1)
	{
		perror("setsockopt (SO_BROADCAST)");
		exit(1);
	}

    /* set up destination address */
    memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(NETWORK);
    addr.sin_port=htons(PORT);
    	

        
    while(1)
    {   
        pthread_mutex_lock(&mutex_wifitransmission);
        localpack=wtpack;
        pthread_mutex_unlock(&mutex_wifitransmission);
        /*BROADCASTING THE STRUCTURE*/   
        if (sendto(fild, &localpack, sizeof(Packet), 0,(struct sockaddr *) &addr, sizeof(addr)) < 0)
        {
            perror("sendto");
            //exit(1);
        }
        usleep((int)(1000000.0/FPS_WIFI));
    }
}//communication function ends
#endif