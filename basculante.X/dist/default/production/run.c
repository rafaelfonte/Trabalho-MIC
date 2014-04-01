#include <sys/io.h>
#include <stdio.h>

#define port 0x378
#define SLEEP_TIME 10

int main (int argc, char* argv[]) {

	if ((iopl(3)) || (ioperm(port,5,1)))
		return -1;
			
	outb(0x24, port);

	return 0;
}
