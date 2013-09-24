//============================================================================
// Name        : LxFTDI.cpp
// Author      : Wouter Caarls,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2013 LGPL
// Description : serial communicatin class linux FTDI
//============================================================================

#include <stdio.h>
#include <sys/time.h>

#include <threemxl/LxFTDI.h>

/*	constructor */
LxFTDI::LxFTDI()
{
}

/* open port */
bool LxFTDI::port_open(const std::string& portname, LxSerial::PortType port_type)
{
        int ret = dev.open(portname);
        if (ret < 0)
                std::cerr << "Error opening FTDI device " << portname << ": " << dev.error_string() << std::endl;

	return !ret;
}

bool LxFTDI::set_speed_int(const int baudrate)
{
	return !dev.set_baud_rate(baudrate);
}

// close the serial port
bool	LxFTDI::port_close()
{
	return !dev.close();
}

int	LxFTDI::port_read(unsigned char* buffer, int numBytes)
{
	int nBytesRead=0;

	while (nBytesRead < numBytes)
	{
		int n = dev.read(buffer+nBytesRead, numBytes);
		if (n < 0)
			return n;
		else
			nBytesRead += n;
	}

	return nBytesRead;
}

int 	LxFTDI::port_read(unsigned char* buffer, int numBytes, int seconds, int microseconds)
{
	struct timeval timeout, now;
	gettimeofday(&timeout, NULL);
	timeout.tv_sec += seconds;
	timeout.tv_usec += microseconds;
	if (timeout.tv_usec > 1000000)
	{
		timeout.tv_sec += 1;
		timeout.tv_usec -= 1000000;
	}

	int nBytesRead=0;
	do
	{
		int n = dev.read(buffer+nBytesRead, numBytes-nBytesRead);
		if (n < 0)
			return n;
		else
			nBytesRead += n;
			
                
		gettimeofday(&now, NULL);
	} while (nBytesRead < numBytes &&
		 (now.tv_sec < timeout.tv_sec || (now.tv_sec == timeout.tv_sec && now.tv_usec < timeout.tv_usec)));

	return nBytesRead;
}

int	LxFTDI::port_write(unsigned char* buffer, int numBytes)
{
	return dev.write(buffer, numBytes);
}

void LxFTDI::flush_buffer()
{
	dev.flush(Ftdi::Context::Input | Ftdi::Context::Output);
}

LxFTDI::~LxFTDI()
{
	dev.close();
}
