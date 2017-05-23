#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "rp.h"
#include "colour.h"
#include "imu.h"

void splash(void);
void help(void);
void parse_uart(void);

//global UM7 receive packet
extern UM7_packet global_packet;

//global experiment active flag
int is_experiment_active = false;

int main(int argc, char *argv[])
{
	pthread_t imu_thread;
	int opt;
	int is_debug_mode;
	
	//retrieve command-line options
    while ((opt = getopt(argc, argv, "dh")) != -1)
    {
        switch (opt)
        {
			case 'd':
				is_debug_mode = 1;
				break; 
			case 'h':
				help();
				break;
			case '?':
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
    }	
	
	splash();
	
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

	//initialize RP API
	if (rp_Init() != RP_OK) 
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Red Pitaya API initialization failed!\n");
		exit(EXIT_FAILURE);
	}

	//initialise IMU and configure update rates
	initIMU();
	
	is_experiment_active = true;
	
	if (pthread_create(&imu_thread, NULL, (void*)parse_uart, NULL))
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

	rp_Release();
	
	return EXIT_SUCCESS;
}


void parse_uart(void)
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
		//process UART buffer and check for valid packets
		if (rxPacket(100) == 0)
		{		
			//process valid packet data
			if ((global_packet.address == DREG_ALL_PROC) && (global_packet.packet_type &= PT_IS_BATCH))
			{		
				for (int i = 0; i < 12; i++)
				{
					float data = bit32ToFloat(bit8ArrayToBit32(&global_packet.data[4*i]));
					fwrite(&data, sizeof(float), 1, imuFile);
				}					
			}
		}	
	}
	
	fclose(imuFile);
}


void splash(void)
{
	system("clear\n");
	printf("UM7-RP\n");
	printf("------\n");	
}


void help(void)
{
	splash();
	printf(" -h: display this help screen\n");
	printf(" -d: enable debug mode\n");
	exit(EXIT_SUCCESS);	
}

/*
system("clear\n");
printf("GYRO_X: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[0])));
printf("GYRO_Y: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[4])));
printf("GYRO_Z: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[8])));
printf("GYRO_TIME: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[12])));
printf("\n");				
printf("ACCL_X: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[16])));
printf("ACCL_Y: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[20])));
printf("ACCL_Z: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[24])));
printf("ACCL_TIME: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[28])));
printf("\n");				
printf("MAGN_X: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[32])));
printf("MAGN_Y: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[36])));
printf("MAGN_Z: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[40])));
printf("MAGN_TIME: \t%f\n", bit32ToFloat(bit8ArrayToBit32(&global_packet.data[44])));
*/
