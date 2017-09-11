#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "rp.h"
#include "colour.h"
#include "imu.h"
#include "binary.h"
#include "uart.h"

void splash(void);
void help(void);
void monitor_imu(void);
void initRP(void);

extern heartbeat beat;

//global flags
int is_experiment_active = false;
int is_debug_mode = false;

int main(int argc, char *argv[])
{
	pthread_t imu_thread;
	int opt;	
	
	//retrieve command-line options
    while ((opt = getopt(argc, argv, "dh")) != -1)
    {
        switch (opt)
        {
			case 'd':
				is_debug_mode = true;
				break; 
			case 'h':
				help();
				break;
			case '?':
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
    }	
	
	splash();
	initRP();	
	initUART();		
	initIMU();
	getFirmwareVersion();	
	
	checkHealth(50);
	
	while(beat.gps_fail)
	{
		checkHealth(50);
	}
	
	resetEKF();	
	zeroGyros();
	setMagReference();
	setHomePosition();	
	
	is_experiment_active = true;
	
	if (pthread_create(&imu_thread, NULL, (void*)monitor_imu, NULL))
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Error launching imu thread.\n");
	}

	//sleep for 5 seconds to emulate other work
	for (int i = 0; i < 10; i++)
	{
		usleep(1e6);
	}
	
	//other work complete
	is_experiment_active = false;

	//join all threads
	pthread_join(imu_thread, NULL);

	dnitUART();
	rp_Release();
	
	return EXIT_SUCCESS;
}


void monitor_imu(void)
{
	FILE *imuFile;
	
	if (!(imuFile = fopen("imu.bin", "wb"))) 
	{
		printf("imu file open failed\n");
		exit(EXIT_FAILURE);
	}
	
	printf("IMU active.\n");
	
	//while experiment is active
	while (is_experiment_active)
	{	
		saveUART(200);
	}
	
	fclose(imuFile);
}


void splash(void)
{
	system("clear\n");
	printf("UM7-RP\n");
	printf("------\n");	
	
	if (is_debug_mode)
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("Debug mode enabled.\n");
	}
	else
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Debug mode disabled.\n");
	}
}


void help(void)
{
	splash();
	printf(" -h: display this help screen\n");
	printf(" -d: enable debug mode\n");
	exit(EXIT_SUCCESS);	
}

void initRP(void)
{
	//initialize RP API
	if (rp_Init() != RP_OK) 
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Red Pitaya API initialization failed!\n");
		exit(EXIT_FAILURE);
	}
}

