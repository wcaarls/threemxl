#include <threemxl/platform/hardware/dynamixel/CDxlCom.h>
#include <threemxl/platform/hardware/dynamixel/CDxlSerialPacketHandler.h>

CDxlSerialPacketHandler::CDxlSerialPacketHandler(LxSerial *&serialPort) :
  mLog("CDxlCom"),
  mSerialPort(serialPort),
  mLastError(0)
{
}

int CDxlSerialPacketHandler::init()
{
  if (mSerialPort == NULL)
    return DXL_NO_SERIAL_PORT;
    
  return DXL_SUCCESS;
}

int CDxlSerialPacketHandler::sendPacket(CDxlPacket *packet, bool replyExpected)
{
	int numBytesWritten = DXL_ERROR;
	for (int i =0 ; i<=SEND_RETRY_FACTOR; i++)
	{
		mLogCrawlLn("Sending packet " << packet->getPktString() << " try " << i );
		numBytesWritten = mSerialPort->port_write(packet->data(), packet->length());
		if (numBytesWritten == packet->length())
		{
			break; //i=SEND_RETRY_FACTOR + 1; // we are done and exit this loop
		}
		else
		{
			//we did not correctly send the packet so we do a cleanup
			if (numBytesWritten < 0)
				mLogWarningLn("Sending packet failed with error code " << -numBytesWritten << ")...");
			else
				mLogWarningLn("Sending packet failed, only written " << numBytesWritten <<" chars...");

			mSerialPort->flush_buffer();
		}

	}

	if (numBytesWritten != packet->length())
	{
		mLogErrorLn("sendPacket() failed permanently after "<< SEND_RETRY_FACTOR+1 <<" tries!");
		mLastError = numBytesWritten;
		return DXL_PKT_SEND_ERROR;
	}

	return DXL_SUCCESS;
}

int CDxlSerialPacketHandler::receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds)
{
	int numBytesRead = DXL_ERROR;
//	mLogCrawlLn("waiting for packet with length "<< (int)packet->length());
	for (int i =0 ; i<=RECEIVE_RETRY_FACTOR; i++)
	{
		numBytesRead = mSerialPort->port_read(packet->data(), packet->length(), seconds, microseconds);
		if (numBytesRead == packet->length())
		{
		        int s = 0;
		        
		        // Strip leading bytes before packet start
		        while (s < packet->length()-1 && (packet->data()[s] != 0xff || packet->data()[s+1] != 0xff)) s++;
		        
		        // Strip leading ffs before packet start
		        while (s < packet->length()-2 && packet->data()[s+2] == 0xff) s++;
		        
		        if (s)
		        {
		                // We removed some characters. Try to get rest of packet now
        		        memmove(packet->data(), packet->data()+s, packet->length()-s);
        		        int n = mSerialPort->port_read(packet->data()+packet->length()-s, s, seconds, microseconds);
        		        if (n != s)
        		        {
        		                mLogErrorLn("Error getting rest of mangled packet " << packet->getPktString());
              		                mSerialPort->flush_buffer();
              		                usleep(10);
              		                mSerialPort->flush_buffer();
              		                if (n < 0)
              		                        mLastError = n;
        		        }
        		        else
        		        {
                			mLogCrawlLn("Received initially mangled packet " << packet->getPktString() << " try " << i );
                			break; //i = RECEIVE_RETRY_FACTOR + 1; // we are done and exit this loop
        		        }
                        }
                        else
                        {
        			mLogCrawlLn("Received packet " << packet->getPktString() << " try " << i );
        			break; //i = RECEIVE_RETRY_FACTOR + 1; // we are done and exit this loop
                        }
		}
		else
		{
			if (numBytesRead < 0 )
			{
				mLastError = numBytesRead;
				if (errno)
				{
        				mLogErrorLn("Receiving packet failed... with error code \""<< strerror(errno) << "\"(" << errno << ")..." );
					mSerialPort->flush_buffer(); //we want to flush only if it was not a timeout
					usleep(10);					 //we waitpacket->length()
					mSerialPort->flush_buffer(); //we want to flush again
				}
				else
				        mLogDebugLn("Timeout waiting for packet");
			}
			else
			{
				mLogErrorLn("Received packet with different length (" << numBytesRead << " in stead of " << (int)packet->length() << ") " << packet->getPktString(numBytesRead) << " try " << i );
			}
		}
	}




//	if (numBytesRead < 0 )
//	{
//		mLastError = numBytesRead;
//		return DXL_PKT_RECV_ERROR;
//	}
//	else
	if (numBytesRead != packet->length())
	{
//		mLogErrorLn("receivePacketWait() failed permanently after " << RECEIVE_RETRY_FACTOR+1 << " try/tries!");
		mLastError = numBytesRead;
		return DXL_PKT_RECV_ERROR;
	}

	if (packet->calcChecksum() != packet->readChecksum())
	{
//		#ifdef __DBG__
//		printf("warning: checksum error in dxl package. RecvCS: %02X CalcCS: %02X\n",packet->readChecksum(), packet->calcChecksum());
//		#endif
		mLogErrorLn("Checksum error in packet: Received CS:"<< std::setw (2) << std::uppercase << std::hex << (int)packet->readChecksum() << " Calculated CS:"<< std::hex << (int)packet->calcChecksum() << std::dec);
                mLogErrorLn("Packet was " << packet->getPktString());
		mLogCrawlLn("flush buffers");
		mSerialPort->flush_buffer();
		mLastError = numBytesRead;
		return DXL_PKT_RECV_CHECKSUM_ERR;
	}

	return DXL_SUCCESS;
}

int CDxlSerialPacketHandler::getLastError()
{
  return mLastError;
}

