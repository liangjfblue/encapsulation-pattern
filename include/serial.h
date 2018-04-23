#ifndef __SERIAL_H__
#define __SERIAL_H__
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
/* According to earlier standards */
#include <sys/time.h>

typedef struct Serial_t {
	int baudrate;	//波特率: 	115200,  19200,  9600,  4800,  2400
	int databits;	//数据位:	7 8
	int stopbits;	//停止位:
	char parity;	//校验位:    o/O--    e/E--	s/S--
	int flags;		//方式标志位: O_NOCTTY / O_NONBLOCK / O_NOFOLLOW / O_SYNC

	int (*ComOpen)(char *dev, struct Serial_t *serial_t);
	void (*Serial_Condifgure)(struct Serial_t* serial, int baudrate, int databits, int stopbits, char parity, int flags);
	int (*ComRead)(int serial_fd, char *buf, int size, int TimeOut);
	int (*ComWrite)(int serial_fd, char *buf, int size);
	int (*ComClose)(int serial_fd);
} Serial_t;

void serial_Init(Serial_t* Serial,
					int (*Serial_ComOpen)(char *dev, Serial_t *serial_t),
					void (*Serial_Condifgure)(struct Serial_t* serial, int baudrate, int databits, int stopbits, char parity, int flags),
					int (*Serial_ComRead)(int serial_fd, char *buf, int size, int TimeOut),
					int (*Serial_ComWrite)(int serial_fd, char *buf, int size),
					int (*Serial_ComClose)(int serial_fd)
				);

/*
 * fun name: SerialProxy_Create
 * function: create a serial device
 * params:   void
 * return:   Serial_t*
 * note:	 none
 * */
Serial_t* Serial_Create(void);

/*
 * fun name: SerialProxy_destroy
 * function: destroy a serial device
 * params:   Serial_t* me -> the serial device pointer
 * return:   void
 * note:	 none
 * */
void Serial_destroy(Serial_t* me);

/*
 * fun name: Serial_Condifgure
 * function: condigure the serial
 * params:   Serial_t* serial -> the serial port sets the structure
 *			 int baudrate -> the serial baudrate
 *			 int databits -> the serial databits
			 int stopbits -> the serial stopbits
			 char parity -> the serial parity
			 int flags -> the serial flags
 * return:   null
 * note:	 none
 * */
void Serial_Condifgure(Serial_t* serial, int baudrate, int databits, int stopbits, char parity, int flags);

/*
 * fun name: Serial_ComOpen
 * function: open and set the serial
 * params:   char *dev -> the serial port's path
 * 			 Serial_t *serial_t -> the serial port sets the structure
 * return:   ok-fd( fd>0 ) 	error-negative number( <0 )
 * note:	 none
 * */
int Serial_ComOpen(char *dev, Serial_t *serial_t);

/*
 * fun name: Serial_ComRead
 * function: open and set the serial
 * params:   char serial_fd -> the serial port's path
 * 			 char *buf -> the buf to read data
 * 			 int size -> the buf's size
 * 			 int OutTime -> wait for timeout
 * return:   ok-buf size 	error-negative number( <0 )
 * note:	 none
 * */
int Serial_ComRead(int serial_fd, char *buf, int size, int TimeOut);

/*
 * fun name: Serial_ComWrite
 * function: open and set the serial
 * params:   char serial_fd -> the serial port's path
 * 			 char *buf -> the buf to write data
 * 			 int size -> the buf's size
 * return:   ok-buf size 	error-negative number( <0 )
 * note:	 返回的数字比请求的数字小，不一定是出错。比如写磁盘时，磁盘已满仍被写
 * */
int Serial_ComWrite(int serial_fd, char *buf, int size);

/*
 * fun name: Serial_ComClose
 * function: close the serial
 * params:   int serial_fd -> the serial port's fd
 * return:   ok-0 	error-negative number( <0 )
 * note:	 none
 * */
int Serial_ComClose(int serial_fd);

#endif



