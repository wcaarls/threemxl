//============================================================================
// Name        : LxSerial.cpp
// Author      : Wouter Caarls,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2013 LGPL
// Description : serial communicatin class linux FTDI
//============================================================================

#ifndef LXFTDI_H_
#define LXFTDI_H_
//#define __DBG__

#include <ftdi.hpp>
#include <threemxl/platform/hardware/serial/LxSerial.h>

class LxFTDI : public LxSerial
{
	protected:
		Ftdi::Context		dev;
		std::string		description;

	public:
				LxFTDI();
		virtual		~LxFTDI();
		virtual bool	port_open(const std::string& portname, LxSerial::PortType port_type);	// open serial port. If overridden, make sure you set s_port_name!!
		virtual bool	is_port_open() { return dev.is_open(); }
		std::string&	get_port_name() { description = dev.description(); return description; }
		virtual bool	set_speed(LxSerial::PortSpeed baudrate ) { fprintf(stderr, "Unsupported function. Use set_speed_int.\n"); return false;}
		virtual bool	set_speed_int(const int baudrate);	// Set speed by integer value directly - UNPROTECTED!
		void		set_clear_echo(bool clear) { }
		virtual bool	port_close();
		virtual int	port_read(unsigned char* buffer, int numBytes);
		virtual int 	port_read(unsigned char* buffer, int numBytes, int seconds, int microseconds);
		virtual int	port_write(unsigned char* buffer, int numBytes);
		virtual void	flush_buffer();													// flush input and output buffers
};

#endif /*LXFTDI_H_*/
