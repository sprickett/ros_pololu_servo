
/*Copyright (c) 2013 Jacques Menuet

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include <polstro/PolstroSerialInterfacePOSIX.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include <errno.h>  

namespace Polstro
{
/* The port name can be:
	- Windows : "\\\\.\\USBSER000", "\\\\.\\COM6", etc...
	- Linux : "/dev/ttyACM0"
	- Mac OS X : "/dev/cu.usbmodem00034567"
*/
SerialInterfacePOSIX::SerialInterfacePOSIX( const std::string& portName )
	:	SerialInterface(),
		mFileDescriptor(-1)
{
	mFileDescriptor = openPort( portName );
}

SerialInterfacePOSIX::~SerialInterfacePOSIX()
{
	if ( isOpen() )
	{
		// Before destroying the interface, we "go home"
		goHomeCP();

		close( mFileDescriptor );
	}
	mFileDescriptor = -1;
}

bool SerialInterfacePOSIX::isOpen() const
{
	return mFileDescriptor!=-1;
}

int SerialInterfacePOSIX::openPort( const std::string& portName )
{
	int fd = open( portName.c_str(), O_RDWR | O_NOCTTY );
	if (fd == -1)
	{
		perror(portName.c_str());
		return -1;
	}

	// Flush away any bytes previously read or written.
  	int result = tcflush(fd, TCIOFLUSH);
  	if (result)
  	{
    		perror("tcflush failed");  // just a warning, not a fatal error
  	}

	// Get the current configuration of the serial port.
  	struct termios options;
	result = tcgetattr(fd, &options);
  	
	// Turn off any options that might interfere with our ability to send and
        // receive raw binary bytes.
        options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
        options.c_oflag &= ~(ONLCR | OCRNL);
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 
        // Set up timeouts: Calls to read() will return as soon as there is
        // at least one byte available or when 100 ms has passed.
//        options.c_cc[VTIME] = 1;
//        options.c_cc[VMIN] = 2;

	tcsetattr(fd, TCSANOW, &options);
  	if (result)
  	{
    		perror("tcgetattr failed");
    		close(fd);
    		return -1;
  	}
 
  	
	return fd;
}

bool SerialInterfacePOSIX::writeBytes( const unsigned char* data, unsigned int numBytesToWrite )
{
	if ( !isOpen() )
		return false;

	// See http://linux.die.net/man/2/write
	ssize_t ret = write( mFileDescriptor, data, numBytesToWrite );
	if ( ret==-1 )
	{
		printf("Error writing. errno=%d\n", errno );
		return false;
	}
	else if ( ret!=numBytesToWrite )
	{
		printf("Error writing. Wrote %ld bytes instead of %d\n", ret, numBytesToWrite );
		return false;
	}

	return true;
}

bool SerialInterfacePOSIX::readBytes( unsigned char* data, unsigned int numBytesToRead )
{
	if ( !isOpen() )
		return false;
	//sleep(5);
	// See http://linux.die.net/man/2/read
	/*
	ssize_t ret = read( mFileDescriptor, data, numBytesToRead );
	
	if ( ret==-1 )
	{
		printf("Error reading. errno=%d\n", errno );
		return false;
	}
	else if ( ret!=numBytesToRead )
	{
		printf("%02X:%02X:%02X:%02X ", data[0], data[1], data[2], data[3]);	
		printf("Error reading. Read %ld bytes instead of %d.  \n", ret, numBytesToRead);
				
		return false;
	}
	printf("%02X:%02X:%02X:%02X %02X:%02X:%02X:%02X \n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	return true;
	*/

	size_t received = 0;
 	while (received < numBytesToRead)
  	{
    		ssize_t ret = read(mFileDescriptor, data + received, numBytesToRead - received);
		if (ret < 0)
    		{
			std::cerr << "Error reading" << std::endl;
                	return false;
    		}
    		if (ret == 0)
    		{
			std::cerr << "Error Read timeout" << std::endl;
      			return false;
    		}
    		received += ret;
  	}
	
  	return true;
}

};
