#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <termios.h>

#include "colour.h"
#include "imu.h"
#include "binary.h"
#include "uart.h"

void splash(void);
void help(void);
void imu_worker(void);
void parse_options(int argc, char *argv[]);

extern heartbeat beat;
extern uint8_t* uart_buffer;

//global flags
int is_experiment_active = 0;
int is_debug_mode = 0;
int is_reset = 0;

int main(int argc, char *argv[])
{
	parse_options(argc, argv);
	splash();
	//initUART(B115200);
	initIMU(is_debug_mode, is_reset);
	
	if (is_debug_mode)
	{
		printConfiguration();
	}

	pthread_t imu_thread;

	if (pthread_create(&imu_thread, NULL, (void*)imu_worker, NULL))
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Error launching imu thread.\n");
	}
	else
	{
		//start experiment
		is_experiment_active = 1;
		cprint("[OK] ", BRIGHT, GREEN);
		printf("Experiment active.\n");
	}

	printf("\n\n\n\n\n");
	//loop to emulate other work
	while (beat.sats_used < 10)
	{
		getHeartbeat();
		printf("\033[6A\n");
		printHeartbeat();		
	}

	//stop experiment
	is_experiment_active = 0;

	//join all threads
	pthread_join(imu_thread, NULL);

	dnitUART();
	
	if (is_debug_mode)
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("Enter host password to transfer files:\n");
		
		//copy experiment folder from red pitaya to host computer
		char command[100];
		sprintf(command, "scp imu.bin darryn@10.42.0.1:/home/darryn/Dropbox/Datasets/Temp");		
		system(command);
	}

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
		fwrite(uart_buffer, sizeof(uint8_t), getUART(), imuFile);
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


void parse_options(int argc, char *argv[])
{
	int opt;

	//retrieve command-line options
    while ((opt = getopt(argc, argv, "dhr")) != -1)
    {
        switch (opt)
        {
			case 'd':
				is_debug_mode = 1;
				break;
			case 'h':
				help();
				break;
			case 'r':
				is_reset = 1;
				break;
			case '?':
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
    }
}




