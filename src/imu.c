#include "imu.h"

struct sp_port *port;
struct sp_port_config *port_config;

uint8_t* byte_buffer;
uint8_t zero_buffer[4] = {0, 0, 0, 0};

packet global_packet;
heartbeat beat;

// Parse the serial data obtained through the UART interface and fit to a general packet structure
uint8_t parseUART(int address, uint8_t* rx_data, uint8_t rx_length)
{
	uint8_t index;
	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes
	if (rx_length < 7)
	{
		//buffer length too short to contain a valid packet
		return 0;		
	}
	
	// Try to find the 'snp' start sequence for the packet
	for (index = 0; index < (rx_length - 2); index++)
    {
		// Check for 'snp'. If found, immediately exit the loop
		if (rx_data[index] == 's' && rx_data[index+1] == 'n' && rx_data[index+2] == 'p')
		{
			//found valid SNP
			uint8_t packet_index = index;
	
			// Check to see if the variable 'packet_index' is equal to (rx_length - 2). If it is, then the above
			// loop executed to completion and never found a packet header.
			if (packet_index == (rx_length - 2))
			{
				return 0;
			}
			
			// If we get here, a packet header was found. Now check to see if we have enough room
			// left in the buffer to contain a full packet. Note that at this point, the variable 'packet_index'
			// contains the location of the 's' character in the buffer (the first byte in the header)
			if ((rx_length - packet_index) < 7)
			{
				return 0;
			}
			
			// We've found a packet header, and there is enough space left in the buffer for at least
			// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
			// the actual length of this packet
			uint8_t PT = rx_data[packet_index + 3];

			// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
			// We have to do this because the individual bits in the PT byte specify the contents of the
			// packet.
			uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
			uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
			uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

			// Now finally figure out the actual packet length
			uint8_t data_length = 0;
			if (packet_has_data)
			{
				if (packet_is_batch)
				{
					// Packet has data and is a batch. This means it contains 'batch_length' registers, each
					// of which has a length of 4 bytes
					data_length = 4*batch_length;
					//printf("Packet is batch, length = %i\n", (int)(data_length));
				}
				else // Packet has data but is not a batch. This means it contains one register (4 bytes)   
				{
					data_length = 4;
				}
			}
			else // Packet has no data
			{
				data_length = 0;
			}
			
			// At this point, we know exactly how long the packet is. Now we can check to make sure
			// we have enough data for the full packet.
			if( (rx_length - packet_index) < (data_length + 5) )
			{
				//printf("Not enough data for full packet!\n");
				return 0;
			}
			
			// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
			// out the data and make sure the checksum is good.
			// Start by extracting all the data
			global_packet.address = rx_data[packet_index + 4];	
			global_packet.packet_type = PT;
			
			if (global_packet.address != address)
			{
				//printf("Wrong packet address, looking again!\n");
				//packet address does not match search address
				//increase the loop index and look for a new valid packet
				continue;
			}
			//printf("Found one!\n");

			// Get the data bytes and compute the checksum all in one step
			global_packet.n_data_bytes = data_length;
			uint16_t computed_checksum = 's' + 'n' + 'p' + global_packet.packet_type + global_packet.address;	
			
			for( int k = 0; k < data_length; k++ )
			{
				// Copy the data into the packet structure's data array
				global_packet.data[k] = rx_data[packet_index + 5 + k];
				// Add the new byte to the checksum
				computed_checksum += global_packet.data[k];		
			}    
		   
			// Now see if our computed checksum matches the received checksum
			// First extract the checksum from the packet
			uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);

			received_checksum |= rx_data[packet_index + 6 + data_length];
			// Now check to see if they don't match
			if (received_checksum != computed_checksum)
			{
				//checksum is bad
				//increase the loop index and look for a new valid packet
				return 0;
			}
			
			//printf("checksum good!\n");
			global_packet.checksum = computed_checksum;
			// At this point, we've received a full packet with a good checksum. It is already
			// fully parsed and copied to the packet structure, so return 0 to indicate that a packet was
			// processed.
			return 1;			
		}
    }    
    
    return 0;	
}


void initIMU(int is_debug_mode, int is_reset)
{
	byte_buffer = (uint8_t*)malloc(UART_BYTE_BUFFER*sizeof(uint8_t));	
	
	if (is_reset)
	{
		writeCommand(RESET_TO_FACTORY);
	}
	
	if (is_debug_mode)
	{
		if (writeCommand(GET_FW_REVISION))
		{		
			char FWrev[5];
			FWrev[0] = global_packet.data[0];
			FWrev[1] = global_packet.data[1];
			FWrev[2] = global_packet.data[2];
			FWrev[3] = global_packet.data[3];
			FWrev[4] = '\0'; //Null-terminate string

			cprint("[**] ", BRIGHT, CYAN);
			printf("Firmware Version: %s\n", FWrev);
		}
		
		cprint("[**] ", BRIGHT, CYAN);
		printf("Reseting IMU registers.\n");
		
		//baud rate of the UM7 main serial port = 115200 (5)
		//baud rate of the UM7 auxiliary serial port = 57600 (4)

		uint8_t com_settings[4]  = {(4 << 0) + (5 << 4), 0, 0, 0};	
		uint8_t health_rate[4] 	 = {0, 1, 0, 0};
		uint8_t chr_nmea_rate[4] = {(1 << 0) + (1 << 4), 0, (1 << 0) + (1 << 4), 0};
		uint8_t all_proc_rate[4] = {0, 0, 0, 0};
		uint8_t position_rate[4] = {0, 0, 0, 0};
		uint8_t misc_settings[4] = {0, 0, 1, 1};
		
		writeRegister(CREG_COM_SETTINGS, 4, com_settings);		// baud rates, auto transmission		
		writeRegister(CREG_COM_RATES1, 4, zero_buffer);			// raw gyro, accel and mag rate	
		writeRegister(CREG_COM_RATES2, 4, zero_buffer);			// temp rate and all raw data rate		
		writeRegister(CREG_COM_RATES3, 4, zero_buffer);			// proc accel, gyro, mag rate		
		writeRegister(CREG_COM_RATES4, 4, all_proc_rate);		// all proc data rate	
		writeRegister(CREG_COM_RATES5, 4, position_rate);		// quart, euler, position, velocity rate
		writeRegister(CREG_COM_RATES6, 4, health_rate);			// heartbeat rate
		writeRegister(CREG_COM_RATES7, 4, chr_nmea_rate);		// CHR NMEA-style packets
		writeRegister(CREG_MISC_SETTINGS, 4, misc_settings);	// miscellaneous filter and sensor control options
	
		//writeCommand(FLASH_COMMIT);
		writeCommand(ZERO_GYROS);
		writeCommand(SET_MAG_REFERENCE);
		writeCommand(SET_HOME_POSITION);
		writeCommand(RESET_EKF);
	
		//let gps lock before setting reference points 
		/*while (beat.sats_used < 3)
		{
			getHeartbeat();
			printHeartbeat();
		}*/
	}
		
	printHome();
}


int txPacket(packet* tx_packet)
{  
	int msg_len = tx_packet->n_data_bytes + 7;

	uint8_t tx_buffer[msg_len + 1];
	//Add header to buffer
	tx_buffer[0] = 's';
	tx_buffer[1] = 'n';
	tx_buffer[2] = 'p';
	tx_buffer[3] = tx_packet->packet_type;
	tx_buffer[4] = tx_packet->address;
	
	//Calculate checksum and add data to buffer
	uint16_t checksum = 's' + 'n' + 'p' + tx_buffer[3] + tx_buffer[4];
	
	int i;
	
	for (i = 0; i < tx_packet->n_data_bytes; i++)
	{
		tx_buffer[5 + i] = tx_packet->data[i];
		checksum += tx_packet->data[i];
	}
	
	tx_buffer[5 + i] = checksum >> 8;
	tx_buffer[6 + i] = checksum & 0xff;
	tx_buffer[msg_len++] = 0x0a; //new line numerical value
	
	if (sp_nonblocking_write(port, (const void*)tx_buffer, msg_len) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "UART write error.\n");
		return 0;
	}

	return 1;
}

//searches for the first valid paket within 'size' samples of the UART buffer
//that matches a specified address in a number of attempts
int rxPacket(int address, int attempts)
{
	for (int i = 0; i < attempts; i++)
	{
		if (parseUART(address, byte_buffer, getUART()) == 1) 
		{
			//found valid packet matching address -> global packet
			return 1; 
		}
	}

	//no valid packet found in all attempts matching address
	return 0; 			
}


int writeRegister(uint8_t address, uint8_t n_data_bytes, uint8_t *data)
{
	packet tx_packet;	

	tx_packet.address = address;
	tx_packet.packet_type = 0;
	tx_packet.n_data_bytes = n_data_bytes;	
	
	if (n_data_bytes != 0)
	{
		tx_packet.packet_type |= PT_HAS_DATA; // packet contains data			
	}	
	
	for (int i = 0; i < n_data_bytes; i++)
	{
		tx_packet.data[i] = data[i]; // populate packet data
	}			
	
	txPacket(&tx_packet);
	
	int i = 0;
	
	do 
	{
		txPacket(&tx_packet);
		
		usleep(0.1e6);
		
		if (i++ == TX_PACKET_ATTEMPTS)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("No response from ");
			
			switch (tx_packet.address)
			{
				case CREG_COM_SETTINGS: 
					printf("CREG_COM_SETTINGS.\n");
					break;
				case CREG_COM_RATES1: 
					printf("CREG_COM_RATES1.\n");
					break;	
				case CREG_COM_RATES2:
					printf("CREG_COM_RATES2.\n");
					break;	
				case CREG_COM_RATES3:
					printf("CREG_COM_RATES3.\n");
					break;	
				case CREG_COM_RATES4:
					printf("CREG_COM_RATES4.\n");
					break;
				case CREG_COM_RATES5:	
					printf("CREG_COM_RATES5.\n");
					break;
				case CREG_COM_RATES6:
					printf("CREG_COM_RATES6.\n");
					break;
				case CREG_COM_RATES7:
					printf("CREG_COM_RATES7.\n");
					break;
				case CREG_MISC_SETTINGS:
					printf("CREG_MISC_SETTINGS.\n");
					break;
				case DREG_HEALTH:
					printf("DREG_HEALTH.\n");
					break;
				case RESET_EKF:
					printf("RESET_EKF.\n");
					break;
				case RESET_TO_FACTORY:
					printf("RESET_TO_FACTORY.\n");
					break;
				case GET_FW_REVISION:
					printf("GET_FW_REVISION.\n");
					break;
				default:
					printf("UM7_R%i.\n", tx_packet.address);
					break;
			}
	
			return 0;
		}
	} while(rxPacket(address, 1) != 1);
	
	return 1;
}


int writeCommand(int command)
{
	if (writeRegister(command, 0, zero_buffer))
	{
		if (global_packet.packet_type & PT_CF)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("%i Error.\n", command);
			exit(EXIT_FAILURE);
		}
		else
		{
			cprint("[OK] ", BRIGHT, GREEN);			
			
			switch (command)
			{
				case GET_FW_REVISION: 
					printf("Received firmware version.\n");
					break;
				case FLASH_COMMIT: 
					printf("Flash committed.\n");
					break;	
				case RESET_TO_FACTORY:
					printf("Reset to factory settings.\n");
					break;	
				case ZERO_GYROS:
					printf("Gyros zero.\n");
					break;	
				case SET_HOME_POSITION:
					printf("GPS home position set.\n");
					break;
				case SET_MAG_REFERENCE:	
					printf("Mag reference set.\n");
					break;
				case RESET_EKF:
					printf("Extended Kalman filter reset.\n");
					break;
			}	
			
			return 1;		
		}				
	}	
	
	return 0;
}


void getHeartbeat(void)
{
	//wait until valid health packet is received
	while(rxPacket(DREG_HEALTH, 1) != 1)
	{
		usleep(1e3);
	}
	
	uint32_t health_reg = bit8ArrayToBit32(global_packet.data);
	beat.sats_used = 0;
	beat.sats_view = 0;
	
	beat.gps_fail = checkBit(health_reg, 0);
	beat.mag_fail = checkBit(health_reg, 1);
	beat.gyro_fail = checkBit(health_reg, 2);
	beat.acc_fail = checkBit(health_reg, 3);
	beat.acc_norm = checkBit(health_reg, 4);
	beat.mag_norm = checkBit(health_reg, 5);
	beat.uart_fail = checkBit(health_reg, 8);
	
	for (int i = 0; i < 6; i++)
	{
		if (health_reg & (uint32_t)(1 << (10 + i)))
		{
			beat.sats_view += pow(2, i);
		}
		
		if (health_reg & (uint32_t)(1 << (26 + i)))
		{
			beat.sats_used += pow(2, i);
		}
	}
}


void printHeartbeat(void)
{
	/*if (beat.gps_fail) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("No GPS data for 2 seconds.\n");
	}
	
	if (beat.mag_fail) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Mag failed to init on startup.\n");
	}		
	
	if (beat.gyro_fail)
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Gyro failed to init on startup.\n");
	}	
 
	if (beat.acc_fail) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Acc failed to init on startup.\n");
	}		
	
	if (beat.acc_norm) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Acc norm exceeded - aggressive acceleration detected.\n");
	}	
	
	if (beat.mag_norm) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Mag norm exceeded - bad calibration.\n");
	}
	
	if (beat.uart_fail)
	{
		cprint("[**] ", BRIGHT, RED);
		printf("UART overflow - reduce broadcast rates.\n");
	}		
	
	cprint("[**] ", BRIGHT, CYAN);
	printf("Satellites in view: %i\n", beat.sats_view);	

	cprint("[**] ", BRIGHT, CYAN);
	printf("Satellites in use: %i\n", beat.sats_used);*/

	int imu_sum = (beat.mag_fail + beat.mag_fail + beat.gyro_fail + beat.acc_fail + beat.acc_norm + beat.mag_norm);
	
	char ok[20];
	char no[20];
	
	ctext(ok, "OK", BRIGHT, GREEN);
	ctext(no, "NO", BRIGHT, RED);
	
	char* gps_status  = (beat.gps_fail)  ? no : ok;
	char* uart_status = (beat.uart_fail) ? no : ok;
	char* imu_status  = (imu_sum)        ? no : ok;
	
	printf("---------------------------\n");
	printf("| GPS | IMU | UART | SATS |\n");
	printf("---------------------------\n");
	printf("| %s  | %s  | %s   | %i/%2i |\n", gps_status, imu_status, uart_status, beat.sats_used, beat.sats_view);	
	printf("---------------------------\n");
	
}


void printHome(void)
{
	if (writeRegister(CREG_HOME_NORTH, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("Latitude: \t%f\n", bit8ArrayToFloat(global_packet.data));
	}
		
	if (writeRegister(CREG_HOME_EAST, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("Longitude: %f\n", bit8ArrayToFloat(global_packet.data));
	}
	
	if (writeRegister(CREG_HOME_UP, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("Altitude: \t%f\n", bit8ArrayToFloat(global_packet.data));
	}
}


void printRegister(uint8_t address)
{
	if (writeRegister(address, 0, zero_buffer))
	{
		printf("UM7_R%i: ", global_packet.address);
		for (int i = 0; i < 4; i++)
			printf(" %i", global_packet.data[i]);
		printf("\n");
	}
}


void printConfiguration(void)
{
	printf("\n");
	
	if (writeRegister(CREG_COM_SETTINGS, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_SETTINGS (%i):\n", global_packet.address);
		printf("baud_rate: \t%i\n", (global_packet.data[0] & 0b11110000) >> 4);
		printf("gps_baud: \t%i\n", (global_packet.data[0] & 0b00001111) >> 0);
		printf("gps_auto: \t%i\n", checkBit(global_packet.data[2], 0));
		printf("sat_auto: \t%i\n", checkBit(global_packet.data[3], 4));
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES1, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES1 (%i):\n", global_packet.address);
		printf("raw_acc_rate: \t%i\n", 	global_packet.data[0]);
		printf("raw_gyro_rate: \t%i\n", global_packet.data[1]);
		printf("raw_mag_rate: \t%i\n", 	global_packet.data[2]);
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES2, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES2 (%i):\n", global_packet.address);
		printf("temp_rate: \t%i\n", 	global_packet.data[0]);
		printf("all_raw_rate: \t%i\n", 	global_packet.data[3]);
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES3, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES3 (%i):\n", global_packet.address);
		printf("proc_acc_rate: \t%i\n", global_packet.data[0]);
		printf("proc_gyro_rate: %i\n", 	global_packet.data[1]);
		printf("proc_mag_rate: \t%i\n", global_packet.data[2]);
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES4, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES4 (%i):\n", global_packet.address);
		printf("all_proc_rate: \t%i\n", global_packet.data[3]);
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES5, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES5 (%i):\n", global_packet.address);
		printf("quat_rate: \t%i\n", 	global_packet.data[0]);
		printf("euler_rate: \t%i\n", 	global_packet.data[1]);
		printf("position_rate: \t%i\n", global_packet.data[2]);
		printf("velocity_rate: \t%i\n", global_packet.data[3]);
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES6, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES6 (%i):\n", global_packet.address);
		printf("pose_rate: \t%i\n", global_packet.data[0]);
		printf("health_rate: \t%i\n", (global_packet.data[1] & 0b00001111));
		printf("\n");
	}
	
	if (writeRegister(CREG_COM_RATES7, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_COM_RATES7 (%i):\n", global_packet.address);
		printf("health_rate: \t%i\n", 	(global_packet.data[0] & 0b11110000) >> 4);
		printf("pose_rate: \t%i\n", 	(global_packet.data[0] & 0b00001111) >> 0);
		printf("attitude_rate: \t%i\n", (global_packet.data[1] & 0b11110000) >> 4);
		printf("sensor_rate: \t%i\n", 	(global_packet.data[1] & 0b00001111) >> 0);
		printf("rates_rate: \t%i\n", 	(global_packet.data[2] & 0b11110000) >> 4);
		printf("gps_pose_rate: \t%i\n", (global_packet.data[2] & 0b00001111) >> 0);
		printf("quat_rate: \t%i\n", 	(global_packet.data[3] & 0b11110000) >> 4);
		printf("\n");
	}
	
	if (writeRegister(CREG_MISC_SETTINGS, 0, zero_buffer))
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("CREG_MISC_SETTINGS (%i):\n", global_packet.address);
		printf("pps: \t\t%s\n", 		checkBit(global_packet.data[2], 0) ? "enabled" : "disabled");
		printf("gyro_bias: \t%s\n", 	checkBit(global_packet.data[3], 2) ? "enabled" : "disabled");
		printf("quaternion: \t%s\n", 	checkBit(global_packet.data[3], 1) ? "enabled" : "disabled");
		printf("mag_state: \t%s\n", 	checkBit(global_packet.data[3], 0) ? "enabled" : "disabled");
		printf("\n");
	}
}



void list_ports(void)
{
	int i;
	struct sp_port **ports;

	sp_list_ports(&ports);

	for (i = 0; ports[i]; i++)
	{
		printf("%s\n", sp_get_port_name(ports[i]));
	}
	sp_free_port_list(ports);
}


int getUART(void)
{
	int bytes_read = 0;
	int bytes_waiting = sp_input_waiting(port);
	
	if (bytes_waiting > 0) 
	{
		//printf("Bytes waiting %i\n", bytes_waiting);	

		memset(byte_buffer, 0, bytes_waiting*sizeof(uint8_t));
		
		bytes_read = sp_nonblocking_read(port, byte_buffer, bytes_waiting);
		
		if (bytes_read < 0)
		{
			printf("Error reading from UART.\n");
			exit(EXIT_FAILURE);
		}
	}
	return bytes_read;
}


void initUART(void)
{
	if (sp_get_port_by_name(UART_PORT, &port) == SP_OK) 
	{		
		if (sp_open(port, SP_MODE_READ_WRITE) == SP_OK)
		{
			cprint("[OK] ", BRIGHT, GREEN);
			printf("Opened serial port: %s.\n", sp_get_port_name(port));
			
			sp_new_config(&port_config);
			sp_set_config_baudrate(port_config, UART_BAUD_RATE);
			sp_set_config_bits(port_config, UART_BITS);
			sp_set_config_parity(port_config, SP_PARITY_NONE);
			sp_set_config_stopbits(port_config, UART_STOPBITS);
			sp_set_config_rts(port_config, SP_RTS_OFF);
			sp_set_config_cts(port_config, SP_CTS_IGNORE);
			sp_set_config_dtr(port_config, SP_DTR_OFF);
			sp_set_config_dsr(port_config, SP_DSR_IGNORE);
			sp_set_config_xon_xoff(port_config, SP_XONXOFF_DISABLED);
			sp_set_config_flowcontrol(port_config, SP_FLOWCONTROL_NONE);
			
			if (sp_set_config(port, port_config) == SP_OK)
			{
				cprint("[OK] ", BRIGHT, GREEN);
				printf("Serial port configured.\n");
			}
			else
			{
				printf("Could not configure the specified serial port.\n");
				exit(EXIT_FAILURE);
			}
		}
		else
		{
			printf("Could not open the specified serial port.\n");
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		printf("Could not obtain a pointer to a new sp_port structure representing the named port.\n");
		printf("Try one of the following:\n");
		list_ports();
		exit(EXIT_FAILURE);
	}	
}


void dnitUART(void)
{
	sp_close(port);
}








