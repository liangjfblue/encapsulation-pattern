#include <stdio.h>
#include <string.h>
#include "serial.h"

#define SERIAL_PATH 	"/dev/ttyO0"

int main(int argc, char *argv[])
{
	int ret = -1;
	int serial_fd = -1;
	char r_buf[1024];
	Serial_t *serial = Serial_Create();

	serial->Serial_Condifgure(serial, 115200, 8, 1, 'n', O_NONBLOCK);

	serial_fd = serial->ComOpen(SERIAL_PATH, serial);

	while(1){
		memset(r_buf, 0 ,1024);
		ret = serial->ComRead(serial_fd, r_buf, sizeof(r_buf), 100);
		if(ret < 0) {
			printf("<%s-%d>liangjf: ComRead error [%s]\n", __func__, __LINE__, strerror(ret));
			return -1;
		} else if(ret > 0){
			printf("<%s-%d>liangjf: ComRead ok, recv: [%d]data\n", __func__, __LINE__, ret);
		}
	}

	Serial_destroy(serial);

	return 0;
}
