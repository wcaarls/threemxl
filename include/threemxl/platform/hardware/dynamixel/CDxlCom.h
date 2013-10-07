// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXELCOM_H_INCLUDED__
#define __DYNAMIXELCOM_H_INCLUDED__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

#include <threemxl/platform/hardware/serial/LxSerial.h>
#ifdef XENOMAI
#include <threemxl/platform/hardware/serial/LxRtSerial.h>
#include <native/timer.h>
#endif //XENOMAI


#include "CDxlPacket.hpp"
#include <threemxl/platform/io/logging/Log2.h>

#define SEND_RETRY_FACTOR 1		///< the number of times we want to retry a send if it fails
#define RECEIVE_RETRY_FACTOR 0	///< the number of times we want to retry a receive if it fails

#define MAX_NUM_DYNAMIXELS				254	///< In any case, no more than 254 dynamixels may reside on one bus (IDs 0-253)

// error return codes Dynamixel Class: they are all < 0
#define DXL_SUCCESS						0
#define DXL_ERROR						-1
#define DXL_PKT_RECV_ERROR				-9001
#define DXL_PKT_RECV_CHECKSUM_ERR		-9002
#define DXL_PKT_RECV_LENGTH_ERR			-9003
#define DXL_PKT_RECV_ID_ERR				-9004
#define DXL_PKT_RECV_TIMEOUT			(-ETIMEDOUT)
#define DXL_ALREADY_INITIALIZED			-9006
#define DXL_NOT_INITIALIZED				-9007
#define DXL_NO_SERIAL_PORT				-9008
#define	DXL_INVALID_PARAMETER			-9009

#define DXL_PKT_SEND_ERROR				-9101
#define DXL_PKT_SEND_LENGTH_ERR			-9102

#define DXL_CHECKSUM_ERROR				0x10
// standard Dynamixel status errorcode bits
// these are now stored at  M3XL_DONE  = 0x87
#define M3XL_NO_ERROR						0b00000000
#define M3XL_INSTRUCTION_ERROR				0b01000000
#define M3XL_OVERLOAD_ERROR					0b00100000
#define M3XL_CHECKSUM_ERROR					0b00010000
#define M3XL_RANGE_ERROR					0b00001000
#define M3XL_OVERHEATING_ERROR				0b00000100
#define M3XL_ANGLE_LIMIT_ERROR				0b00000010
#define M3XL_INPUT_VOLTAGE_ERROR			0b00000001

#define M3XL_STATUS_EEPROM_ERROR			0x80
#define M3XL_STATUS_NOT_INITIALIZED			0x81
#define M3XL_STATUS_EM_STOP_ERROR			0x82
#define	M3XL_STATUS_INIT_TIME_OUT_ERROR		0x83
#define	M3XL_STATUS_MAX_POS_ERROR			0x84
#define	M3XL_STATUS_MAX_TORQUE_ERROR		0x85
#define	M3XL_STATUS_MAX_CURRENT_ERROR		0x86
#define	M3XL_STATUS_MOTOR_STUCK_ERROR		0x87
#define	M3XL_STATUS_JOINT_STUCK_ERROR		0x88
#define	M3XL_STATUS_PROTOCOL_TIME_OUT_ERROR	0x89

// dxlPkgRecvWait // TODO: Tune these parameters
#define DXL_PKT_RECV_WAIT_TIME_USEC		100000 // 100ms
#define DXL_PKT_RECV_WAIT_TIME_SEC 		0

// Instruction constants
#define INST_PING					0x01
#define INST_READ					0x02
#define INST_WRITE					0x03
#define INST_REG_WRITE				0x04
#define INST_ACTION					0x05
#define INST_RESET					0x06
#define INST_DIGITAL_RESET			0x07
#define INST_SYSTEM_READ			0x0C
#define INST_SYSTEM_WRITE			0x0D
#define INST_SYNC_WRITE				0x83
#define INST_SYNC_REG_WRITE			0x84


class CDxlPacketHandler;

/// Generic base class for CDxlGeneric and CDxlGroup
/** Both single dynamixels/3mxls and groups of dynamixels/3mxls have to
  * send packets, use the serial port and do proper error handling. */
class CDxlCom
{
	protected:
		CLog2		mLog;
		bool		mInitialized;
		int			mLastError;

		LxSerial*	mSerialPort;
		CDxlPacketHandler*	mPacketHandler;
		
		/// Packet commands. Synchronous (blocking) interface
		int			sendPacket(CDxlPacket *packet, bool replyExpected=true);

		/// Packet commands. Synchronous (blocking) interface
		int			receivePacketWait(CDxlStatusPacket *packet, int seconds=DXL_PKT_RECV_WAIT_TIME_SEC, int microseconds=DXL_PKT_RECV_WAIT_TIME_USEC);
		//int			receivePacket(CDxlStatusPacket *packet); //not used at the moment

	public:
		CDxlCom();

		bool		isInitialized()	{return mInitialized;}

		/// Set packet handler to something else than direct serial port
		/** Becomes owner */
		int			setPacketHandler(CDxlPacketHandler *packetHandler);
		
		int			initPacketHandler();

		/** \returns Human-readable error code */
		static const char*	translateErrorCode(int errorCode);

		int			getLastError()			{return mLastError;}
};

#endif // __DYNAMIXELCOM_H_INCLUDED__
