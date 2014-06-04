/*
	Copyright (c) 2009, Scott Ellis
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		  notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.
		* Neither the name of the <organization> nor the
		  names of its contributors may be used to endorse or promote products
		  derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY Scott Ellis ''AS IS'' AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL Scott Ellis BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef TARGET_RASPBERRY_PI
    #include <linux/i2c-dev.h>
#endif

/* some local functions */
static int i2c_open_device();
static int i2c_set_slave_address(int fh, uint8_t address);

/*
 *  Return a file handle if successful.
 *  Return a value less then zero on failure.
 */
int i2c_start_transaction(uint8_t slave_address, const char* i2c_bus)
{
	int fh, result;

#ifdef TARGET_LINUX
	fh = i2c_open_device(i2c_bus);

	if (fh < 0)
        return -1;

	result = i2c_set_slave_address(fh, slave_address);

	if (result < 0) {
		close(fh);
		return result;
	}
#else
    fh = 1;
#endif
    
	return fh;
}

/*
 *  Just closes the file handle returned by i2c_start_transaction.
 */
int i2c_end_transaction(int fh)
{
#ifdef TARGET_RASPBERRY_PI
	if (fh > 0) 
		close(fh);
#endif
    
	return 1;
}

int i2c_open_device(const char* i2c_bus)
{
	int fh = -1;
#ifdef TARGET_RASPBERRY_PI
	fh = open(i2c_bus, O_RDWR);

	if (fh < 0) {
		fprintf(stderr, "Error: Could not open file %s: %s\n",
				i2c_bus, strerror(errno));
	}
#else
    fh = 1;
#endif
	return fh;
}

int i2c_set_slave_address(int fh, uint8_t address)
{
	if (fh < 0) 
		return -1;

#ifdef TARGET_RASPBERRY_PI
	if (ioctl(fh, I2C_SLAVE, address) < 0) {
		if (errno == EBUSY) 
			fprintf(stderr, "Device %d is busy!\n", address);
		else  
			fprintf(stderr, "Could not set slave address to 0x%02x: %s\n",
				address, strerror(errno));
		

		return -1;
	}
#endif

	return 1;
}

