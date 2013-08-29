#include "acyut_gpio.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{	
	short switch2=0;
	int arg;
	if(argc>1)
		arg=atoi(argv[1]);
	switch2=read_switch(arg);
	switch(switch2)
	{
		 case 0:printf("0");
		 break;
		 case 1:printf("1");
		 break;
	}
}