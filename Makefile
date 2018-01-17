CC=arm-linux-gnueabihf-gcc
#HAD TO CHANGE AWAY FROM GNUEABI

#Default location for h files is ./source
CFLAGS= -std=gnu99 -Wall -Werror -I./src -L lib -lm -lpthread

#h files used go here
DEPS= colour.h imu.h binary.h uart.h

#c files used go here (with .o extension)
OBJ = src/main.o src/colour.o src/imu.o src/binary.o src/uart.o

#name of generated binaries
BIN = um7rp

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFlags)

um7rp: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o src/*.o
