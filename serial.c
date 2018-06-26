#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <sys/select.h>

#define BUFFER_SIZE 100

char incomingBuffer[BUFFER_SIZE];
int fd1;
int fd2;

void * ReaderThread(void)
{
	int total=0;
	struct timeval timeOut;
	fd_set fdSet;
	int maxFdp=0;

	printf("Waiting for data on serial ports\r\n");
	
	while(1)
	{
		timeOut.tv_sec = 0;
		timeOut.tv_usec = 1000000; // Every 100 ms 
		FD_ZERO(&fdSet);
		FD_SET(fd1, &fdSet);
		FD_SET(fd2, &fdSet);
		maxFdp = fd2+1;

		int result = select(maxFdp, &fdSet, (fd_set*)0, (fd_set*)0, &timeOut);

		if (result < 0) {
			printf("A select error occured.. exiting\r\n");
			return;
		}
		else if (result > 0)
		{
			//printf("Data is available on a port\r\n");
			int n=0;
			if (FD_ISSET(fd1, &fdSet)) {
				n = read(fd1, incomingBuffer, BUFFER_SIZE);
				printf("Data is available on a port  ttyS2 \r\n");
			} else if (FD_ISSET(fd2, &fdSet)) {
				n = read(fd2, incomingBuffer, BUFFER_SIZE);
				printf("Data is available on a port  ttyS3 \r\n");			   
			}
			else {
				printf("Unexpected fd from select... exiting\r\n");
				return;
			}
			
			incomingBuffer[n] = '\0';
			printf("Received: %s\r\n", incomingBuffer);
			total += n;
		}
		// else (result==0) just a time out so go round again
	}
}

int OpenPort(const char* port)
{
     printf("Opening serial %s\r\n", port);
     int fd=open(port, O_RDWR | O_NOCTTY | O_NDELAY);
     if (fd == -1)
     {
          printf("Error opening serial port %s... exiting", port);
          return -1;
     }                       
     fcntl(fd, F_SETFL, 0); /* Reads will be blocking */
     struct termios options;
     tcgetattr(fd, &options);
     (void)cfsetispeed(&options, B57600); /* (void) is to stop warning in cygwin */
     (void)cfsetospeed(&options, B57600); 
     options.c_cflag &= ~CSIZE;
     options.c_cflag |= CS8;  /* 8 bits */
     options.c_cflag &= ~CSTOPB; /* 1 stop bit */
     options.c_cflag &= ~PARENB; /* no parity */
     options.c_cflag &= ~PARODD;
     options.c_cflag &= ~CRTSCTS; /* HW flow control off */
     options.c_lflag =0; /* RAW input */
     options.c_iflag = 0;            /* SW flow control off, no parity checks etc */
     options.c_oflag &= ~OPOST; /* RAW output */
     options.c_cc[VTIME]=10; /* 1 sec */
     options.c_cc[VMIN]=BUFFER_SIZE;
     options.c_cflag |= (CLOCAL | CREAD);
     tcsetattr(fd, TCSAFLUSH, &options);

     return fd;
}

int main(int argc, char** argv)
{
     fd1 = OpenPort("/dev/ttyS2");
     fd2 = OpenPort("/dev/ttyS3");

     /* Start thread to wait for incoming message on ttyS1 */
     ReaderThread();
     sleep(1);
     while (1) {}
     close(fd1);

     return 0;
}