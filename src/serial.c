#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
/* According to earlier standards */
#include <sys/time.h>
#include "serial.h"
#include <stdlib.h>

#define NULL	((void *)0)

void serial_Init(Serial_t* Serial,
					int (*Serial_ComOpen)(char *dev, Serial_t *serial_t),
					void (*Serial_Condifgure)(struct Serial_t* serial, int baudrate, int databits, int stopbits, char parity, int flags),
					int (*Serial_ComRead)(int serial_fd, char *buf, int size, int TimeOut),
					int (*Serial_ComWrite)(int serial_fd, char *buf, int size),
					int (*Serial_ComClose)(int serial_fd)
				)
{
	Serial->baudrate = 0;
	Serial->databits = 0;
	Serial->stopbits = 0;
	Serial->parity = 0;
	Serial->flags = 0;

	Serial->ComOpen = Serial_ComOpen;
	Serial->ComRead = Serial_ComRead;
	Serial->ComWrite = Serial_ComWrite;
	Serial->ComClose = Serial_ComClose;
}

Serial_t* Serial_Create(void)
{
	Serial_t* serial = (Serial_t*)malloc(sizeof(Serial_t));
	if(serial == NULL) {
		printf("<%s-%d>liangjf: malloc serial error\r\n", __func__,__LINE__);
		return NULL;
	}
	serial_Init(serial, Serial_ComOpen, Serial_Condifgure, Serial_ComRead, Serial_ComWrite, Serial_ComClose);
	return serial;
}

void Serial_destroy(Serial_t* me)
{
	if(me != NULL) {
		//TODO	clean the zi yuan
	}
	free(me);
}

void Serial_Condifgure(struct Serial_t* serial, int baudrate, int databits, int stopbits, char parity, int flags)
{
	serial->baudrate = baudrate;
	serial->databits = databits;
	serial->stopbits = stopbits;
	serial->parity = parity;
	serial->flags = flags;
}

int Serial_ComOpen(char *dev, struct Serial_t *serial_t)
{
	int fd = open(dev, O_RDWR | serial_t->flags);
	if (fd < 3) { /* 0, 1, 2 are reserved and -1 means error */
		printf("<%s-%d>liangjf: Set_Serial error\r\n", __func__,__LINE__);
		return -1;
	}

	int i, err = 0;
	struct termios port_attr;
	int baudrate_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400};
	int name_arr[] = {115200,  19200,  9600,  4800,  2400};

	tcgetattr(fd, &port_attr);

	for (i = 0; i < sizeof (baudrate_arr) / sizeof (int); i++) {
		if (serial_t->baudrate == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&port_attr, baudrate_arr[i]);
			cfsetospeed(&port_attr, baudrate_arr[i]);
			err = tcsetattr(fd, TCSANOW, &port_attr);
			if (err) {
				printf("<%s-%d>liangjf: setting serial port error\r\n", __func__,__LINE__);
				return -2;
			}
			tcflush(fd, TCIOFLUSH);
			break;
		}
		if (i == sizeof (baudrate_arr) / sizeof (int)) {
			printf("<%s-%d>liangjf: %s find no baudrate matched: %d\r\n", __func__,__LINE__, dev, serial_t->baudrate);
			return -3;
		}
	}

	//for some reasons we have to clear c_iflag, c_oflag, c_lflag, c_line so
	//that the serial port will receive full data from input
	port_attr.c_iflag = 0;
	port_attr.c_cflag |= (CLOCAL | CREAD); //this line is important too
	port_attr.c_cflag &= ~CSIZE;
	switch (serial_t->databits) {
		case 6:
			port_attr.c_cflag |= CS6;
			break;
		case 7:
			port_attr.c_cflag |= CS7;
			break;
		case 8:
			port_attr.c_cflag |= CS8;
			break;
		default:
			printf("<%s-%d>liangjf: Unsupported data size\r\n", __func__,__LINE__);
			return -4;
	}

	switch (serial_t->parity) {
		case 'n':
		case 'N':
			port_attr.c_cflag &= ~PARENB; /* Clear parity enable */
			//port_attr.c_iflag &= ~INPCK; /* Enable parity checking */
			break;
		case 'o':
		case 'O':
			port_attr.c_cflag |= (PARODD | PARENB);
			port_attr.c_iflag |= INPCK; /* Disnable parity checking */
			break;
		case 'e':
		case 'E':
			port_attr.c_cflag |= PARENB; /* Enable parity */
			port_attr.c_cflag &= ~PARODD;
			port_attr.c_iflag |= INPCK; /* Disnable parity checking */
			break;
		case 'S':
		case 's': /*as no parity*/
			port_attr.c_cflag &= ~PARENB;
			port_attr.c_cflag &= ~CSTOPB;
			break;
		default:
			printf("<%s-%d>liangjf: Unsupported parity\r\n", __func__,__LINE__);
			return -5;
	}

	switch (serial_t->stopbits) {
		case 1:
			port_attr.c_cflag &= ~CSTOPB;
			break;
		case 2:
			port_attr.c_cflag |= CSTOPB;
			break;
		default:
			printf("<%s-%d>liangjf: Unsupported stop bits\r\n", __func__,__LINE__);
			return -5;
	}

	port_attr.c_cc[VTIME] = 0; /* timeout 15 seconds*/
	port_attr.c_cc[VMIN] = 0; /* Update the port_attr and do it NOW */

	/* set the terminal to raw mode */
	port_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
	port_attr.c_oflag &= ~OPOST; /*Output*/

//	port_attr.c_cflag |= CRTSCTS; //设置有RTS/CTS硬件流控
	port_attr.c_cflag &= ~CRTSCTS; //设置无RTS/CTS硬件流控

	if (tcsetattr(fd, TCSANOW, &port_attr) != 0) {
		printf("<%s-%d>liangjf: Setup serial port %s\r\n", __func__,__LINE__, dev);
		return -6;
	}

	tcflush(fd, TCIFLUSH);

	return fd;
}

int Serial_ComRead(int serial_fd, char *buf, int size, int TimeOut)
{
	int ret = -1;
	fd_set s_fd;
	int max_fd = serial_fd + 1;
	struct timeval time;
	int sec = TimeOut/1000;
	int min = TimeOut%1000;

	time.tv_sec = sec;
	time.tv_usec = min*1000;

	FD_ZERO(&s_fd);
	FD_SET(serial_fd, &s_fd);

	switch( select(max_fd, &s_fd, &s_fd, NULL, &time) ){
		case -1:
			printf("<%s-%d>liangjf: select_read error\r\n", __func__,__LINE__);
		break;
		case 0:
			printf("<%s-%d>liangjf: select_timeout to read\n", __func__,__LINE__);
		break;
		default:
			if( FD_ISSET(serial_fd, &s_fd) ) {
				ret = read(serial_fd, buf, size);
				if( ret < 0 ) {
					printf("<%s-%d>liangjf: Comread error\n", __func__,__LINE__);
				}
			}
		break;
	}
	return ret;
}

/*
 * 注意：返回的数字比请求的数字小，不一定是出错。比如写磁盘时，磁盘已满仍被写
 * */
int Serial_ComWrite(int serial_fd, char *buf, int size)
{
	int w_len = 0;

	w_len = write(serial_fd, buf, size);
	if( w_len < 0 ) {
		printf("<%s-%d>liangjf: ComWrite error\n", __func__,__LINE__);
		return -1;
	}

	return w_len;
}


int Serial_ComClose(int serial_fd)
{
	int ret = close(serial_fd);
	if( ret < 0 ) {
		printf("<%s-%d>liangjf: ComClose %d error\n", __func__, __LINE__, serial_fd);
		return -1;
	}
	return ret;
}











