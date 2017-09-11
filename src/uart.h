#ifndef UM7_UART_H
#define UM7_UART_H

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>		
#include <fcntl.h>			
#include <termios.h>		
#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include "colour.h"

void initUART(void);
int dnitUART(void);
uint8_t* getUART(int size);
void saveUART(int size);

#endif
