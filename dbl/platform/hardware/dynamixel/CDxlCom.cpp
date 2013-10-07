// Dynamixel control code - C++ file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#include <threemxl/platform/hardware/dynamixel/CDxlCom.h>
#include <threemxl/platform/hardware/dynamixel/CDxlSerialPacketHandler.h>

//***********************************************************//
//********************* CDxlCom *********************//
//***********************************************************//

CDxlCom::CDxlCom() : mLog("CDxlCom")
{
//	mLog.setLevel(llCrawl);
//	mLog.enableConsoleOutput("false");
	mInitialized	= false;
	mLastError		= 0;
	mSerialPort		= NULL;
	mPacketHandler = new CDxlSerialPacketHandler(mSerialPort);
}

int CDxlCom::setPacketHandler(CDxlPacketHandler *packetHandler)
{
	if (packetHandler == NULL)
		return DXL_INVALID_PARAMETER;
		
	delete mPacketHandler;
	mPacketHandler = packetHandler;
	
	return DXL_SUCCESS;
}

int CDxlCom::initPacketHandler()
{
	if (mPacketHandler == NULL)
		return DXL_INVALID_PARAMETER;
		
	return mPacketHandler->init();
}

const char*	CDxlCom::translateErrorCode(int errorCode)
{
	switch (errorCode)
	{
	case DXL_SUCCESS: return "DXL_SUCCESS"; break;
	case DXL_PKT_RECV_ERROR: return "DXL_PKT_RECV_ERROR"; break;
	case DXL_PKT_RECV_CHECKSUM_ERR: return "DXL_PKT_RECV_CHECKSUM_ERR"; break;
	case DXL_PKT_RECV_LENGTH_ERR: return "DXL_PKT_RECV_LENGTH_ERR"; break;
	case DXL_PKT_RECV_ID_ERR: return "DXL_PKT_RECV_ID_ERR"; break;
	case DXL_PKT_RECV_TIMEOUT: return "DXL_PKT_RECV_TIMEOUT" ; break;
	case DXL_ALREADY_INITIALIZED: return "DXL_ALREADY_INITIALIZED"; break;
	case DXL_NOT_INITIALIZED: return "DXL_NOT_INITIALIZED"; break;
	case DXL_NO_SERIAL_PORT: return "DXL_NO_SERIAL_PORT"; break;
	case DXL_INVALID_PARAMETER: return "DXL_INVALID_PARAMETER"; break;
	case DXL_PKT_SEND_ERROR: return "DXL_PKT_SEND_ERROR"; break;
	case DXL_PKT_SEND_LENGTH_ERR: return "DXL_PKT_SEND_LENGTH_ERR"; break;
	case M3XL_INSTRUCTION_ERROR				: return "M3XL_INSTRUCTION_ERROR			 ";break;
	case M3XL_OVERLOAD_ERROR				: return "M3XL_OVERLOAD_ERROR				 ";break;
	case M3XL_CHECKSUM_ERROR				: return "M3XL_CHECKSUM_ERROR				 ";break;
	case M3XL_RANGE_ERROR					: return "M3XL_RANGE_ERROR					 ";break;
	case M3XL_OVERHEATING_ERROR				: return "M3XL_OVERHEATING_ERROR			 ";break;
	case M3XL_ANGLE_LIMIT_ERROR				: return "M3XL_ANGLE_LIMIT_ERROR			 ";break;
	case M3XL_INPUT_VOLTAGE_ERROR			: return "M3XL_INPUT_VOLTAGE_ERROR			 ";break;
	case M3XL_STATUS_EEPROM_ERROR			: return "M3XL_STATUS_EEPROM_ERROR			 ";break;
	case M3XL_STATUS_NOT_INITIALIZED		: return "M3XL_STATUS_NOT_INITIALIZED		 ";break;
	case M3XL_STATUS_EM_STOP_ERROR			: return "M3XL_STATUS_EM_STOP_ERROR			 ";break;
	case M3XL_STATUS_INIT_TIME_OUT_ERROR	: return "M3XL_STATUS_INIT_TIME_OUT_ERROR	 ";break;
	case M3XL_STATUS_MAX_POS_ERROR			: return "M3XL_STATUS_MAX_POS_ERROR			 ";break;
	case M3XL_STATUS_MAX_TORQUE_ERROR		: return "M3XL_STATUS_MAX_TORQUE_ERROR		 ";break;
	case M3XL_STATUS_MAX_CURRENT_ERROR		: return "M3XL_STATUS_MAX_CURRENT_ERROR		 ";break;
	case M3XL_STATUS_MOTOR_STUCK_ERROR		: return "M3XL_STATUS_MOTOR_STUCK_ERROR		 ";break;
	case M3XL_STATUS_JOINT_STUCK_ERROR		: return "M3XL_STATUS_JOINT_STUCK_ERROR		 ";break;
	case M3XL_STATUS_PROTOCOL_TIME_OUT_ERROR: return "M3XL_STATUS_PROTOCOL_TIME_OUT_ERROR";break;

	default: return "UNKOWN ERROR";
	}
}

int CDxlCom::sendPacket(CDxlPacket *packet, bool replyExpected)		// Returns true if no. of bytes sent equals packet size
{
	int result = mPacketHandler->sendPacket(packet, replyExpected);

//	std::cout << "Sent packet " << packet->getPktString() << std::endl;
	
	if (result != DXL_SUCCESS)
		mLastError = mPacketHandler->getLastError();
	return result;
}

//int CDxlCom::receivePacket(CDxlStatusPacket *packet)
//{
//	int numBytesRead = mSerialPort->port_read(packet->data(), packet->length());
//	if (numBytesRead < 0 )
//	{
//		mLastError = numBytesRead;
//		return DXL_PKT_RECV_ERROR;
//	}
//	else if (numBytesRead != packet->length())
//	{
//		mLastError = numBytesRead;
//		return DXL_PKT_RECV_LENGTH_ERR;
//	}
//
//	if ( packet->calcChecksum() != packet->readChecksum() )
//	{
//		#ifdef __DBG__
//		printf("warning: CDxlCom::receivePacket: checksum error in dlx package. RecvCS: %02X CalcCS: %02X\n",packet->readChecksum(), packet->calcChecksum());
//		#endif
//
//		return DXL_PKT_RECV_CHECKSUM_ERR;
//	}
//	return DXL_SUCCESS;
//}

int CDxlCom::receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds)
{
	int result = mPacketHandler->receivePacketWait(packet, seconds, microseconds);
	
	if (result != DXL_SUCCESS)
		mLastError = mPacketHandler->getLastError();
	return result;
}
