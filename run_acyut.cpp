#include "acyut_gpio.h"
#include <stdio.h>
#include <stdlib.h>

int main()
{
	int switch2;
	while(1)
	{
		switch2 = read_switch(2);
		led(4,1);
		//printf("Reading Switch2\n");
		if(switch2==1)
		{	
			//printf("Switch2 is on\nRunning Behaviour\n");
			led(3,1);
			led(4,1);
			system("./simple_behaviour");
			led(3,0);
		}
		else
		{
			//printf("Switch2 is OFF\n");
			led(4,0);
			exit(0);
		}
	}
}