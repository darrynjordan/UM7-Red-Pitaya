#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "rp.h"
#include "colour.h"
#include "imu.h"
#include "binary.h"
#include "uart.h"

void splash(void);
void help(void);
void imu_worker(void);
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
	
	if (pthread_create(&imu_thread, NULL, (void*)imu_worker, NULL))
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Error launching imu thread.\n");
	}
	else
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("Experiment active.\n");
	}
	
	//start experiment
	is_experiment_active = true;

	//loop to emulate other work
	for (int i = 0; i < 5; i++)
	{
		getHeartbeat(25);
		showHeartbeat();
		usleep(1e6);
	}
	
	//stop experiment
	is_experiment_active = false;

	//join all threads
	pthread_join(imu_thread, NULL);

	dnitUART();
	rp_Release();
	
	return EXIT_SUCCESS;
}


void imu_worker(void)
{
	FILE *imuFile;
	
	if (!(imuFile = fopen("imu.bin", "wb"))) 
	{
		printf("imu file open failed\n");
		exit(EXIT_FAILURE);
	}
	
	//while experiment is active
	while (is_experiment_active)
	{	
		fwrite(getUART(200), sizeof(uint8_t), 200, imuFile);
		usleep(0.1e6);
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

