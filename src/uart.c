#include "uart.h"

// UART file discriptor
int uart_fd = -1; 

// UART buffer
uint8_t* uart_buffer;

uint8_t* getUART(int size)
{
	// don't block serial read 
	fcntl(uart_fd, F_SETFL, FNDELAY); 
	
	uart_buffer = (uint8_t*)malloc(size*sizeof(uint8_t));
  
	while(1)
	{
		if (uart_fd == -1)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("UART has not been initialized.\n");
		}
		
		// perform uart read
		int rx_length = read(uart_fd, (void*)uart_buffer, size);
		
		if (rx_length == -1)
		{
			//printf("No UART data available yet, check again.\n");
			if(errno == EAGAIN)
			{
				// an operation that would block was attempted on an object that has non-blocking mode selected.
				//printf("UART read blocked, try again.\n");
				continue;
			} 
			else
			{
				printf("Error reading from UART.\n");
			}
		  
		}
		else if (rx_length == size)
		{	
			return uart_buffer;
		}
	}  
	
	tcflush(uart_fd, TCIFLUSH); 
}


void initUART(void)
{
	dnitUART();
	
	uart_fd = open("/dev/ttyPS1", O_RDWR | O_NOCTTY | O_NDELAY);

	if(uart_fd == -1)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "Failed to init UART.\n");
		exit(EXIT_FAILURE);
	}
	
	struct termios settings;
	tcgetattr(uart_fd, &settings);

	/*  CONFIGURE THE UART
	*  The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):

	*	Baud rate:- B1200, B2400, B4800, B9600, B19200,
	*	B38400, B57600, B115200, B230400, B460800, B500000,
	*	B576000, B921600, B1000000, B1152000, B1500000,
	*	B2000000, B2500000, B3000000, B3500000, B4000000
	*	CSIZE:- CS5, CS6, CS7, CS8
	*	CLOCAL - Ignore modem status lines
	* 	CREAD - Enable receiver
	*	IGNPAR = Ignore characters with parity errors
	*	ICRNL - Map CR to NL on input (Use for ASCII comms
	*	where you want to auto correct end of line characters
	*	- don't us e for bianry comms!)
	*	PARENB - Parity enable
	*	PARODD - Odd parity (else even) */

	/* Set baud rate to 115200 */
	speed_t baud_rate = B115200;

	/* Baud rate functions
	* cfsetospeed - Set output speed
	* cfsetispeed - Set input speed
	* cfsetspeed  - Set both output and input speed */

	cfsetspeed(&settings, baud_rate);

	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
	settings.c_lflag = ICANON; /* canonical mode */
	settings.c_oflag &= ~OPOST; /* raw output */

	/* Setting attributes */
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &settings);
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("UART Initialised.\n");
}


int dnitUART(void)
{
	tcflush(uart_fd, TCIFLUSH);
	close(uart_fd);

	return 1;
}


void saveUART(int size)
{
	/*fwrite(getUARTbuffer(200), sizeof(uint8_t), 200, imuFile);		
	usleep(0.01e6);*/
}


