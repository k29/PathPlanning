#include "acyut_gpio.h"

pthread_mutex_t mutex_switch=PTHREAD_MUTEX_INITIALIZER;


int switch1;

void* switchupdate( void*)
{
    while(1)
    {
        pthread_mutex_lock(&mutex_switch);
        switch1 = read_switch(1);
        pthread_mutex_unlock(&mutex_switch);
        sleep(1);
    }
}
