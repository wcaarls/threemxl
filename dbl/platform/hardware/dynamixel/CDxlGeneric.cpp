#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>

int CDxlGeneric::readData(BYTE startingAddress, BYTE dataLength, BYTE *data)
{
	int sendErr = 0 ;
	int receiveErr = 0 ;
	CDxlStatusPacket statusPacket(dataLength);

	CDxlPacket packet(mID, INST_READ, 2);												// create request package
	packet.setParam(0, startingAddress);
	packet.setParam(1, dataLength);
	packet.setChecksum();

	//do the serial communication
	for (int i=0 ; i<=PACKET_RETRY_FACTOR ; i++)
	{
		sendErr = sendPacket(&packet);													// send data request
		if (sendErr == DXL_SUCCESS)
		{
			receiveErr = receivePacketWait(&statusPacket);									// receive data
			if (receiveErr == DXL_SUCCESS)
			{
				i = PACKET_RETRY_FACTOR; //success
			}
			else {
			//	mLogCrawlLn("receivePacketWait error for ID "<< mID);
				mLogWarningLn("receivePacketWait error for ID "<< mID);
				if (i == PACKET_RETRY_FACTOR){ return receiveErr;}
				else
				{
					mLogCrawlLn("retrying to read data from servo with ID "<< mID << ", Try " << i+1 );
				}
			}
		}
		else
		{
			mLogCrawlLn("sendPacket error for ID "<< mID);
			if (i == PACKET_RETRY_FACTOR)
			{
				return sendErr;
			}
			else
			{
				mLogCrawlLn("retrying to read data from servo with ID "<< mID << ", Try " << i+1 );
			}
		}
	}

	mLogCrawlLn("Received status:" << statusPacket.getPktString());

	statusPacket.getParams(0, dataLength, data);										// process received data
	return statusPacket.getError();												// get error out of received data

}

int CDxlGeneric::ping()
{
	mLogInfoLn("sending ping to servo with ID:"<< mID);
	CDxlPacket packet(mID, INST_PING, 0);
	packet.setChecksum();
	int error = sendPacket(&packet);
	if (error != DXL_SUCCESS)
		return error;

	CDxlStatusPacket statusPacket(0);
	error = receivePacketWait(&statusPacket);// , 0, (int)100E3);	// 100 ms
	if (error == DXL_SUCCESS){
		error = statusPacket.getError(); // Perhaps the dynamixel itself has an error
		mLogCrawlLn("servo with ID:"<< mID << " returned "<< (int)error << " on ping");
	}
	else
		mLogDebugLn("ping failed for ID " << mID);

	return error;
}

int CDxlGeneric::action()
{
	mLogCrawlLn("action() called for servo with ID:"<< mID);
	CDxlPacket packet(mID, INST_ACTION, 0);
	packet.setChecksum();
	return sendPacket(&packet, false);
}

int CDxlGeneric::reset()
{
	mLogCrawlLn("reset() called for servo with ID:"<< mID);
	CDxlPacket packet(mID, INST_RESET, 0);
	packet.setChecksum();
	return sendPacket(&packet, false);
}

/**
 * Here we have syncWriteLocation as the position to place the data that otherwise would be send directly to the dynamixel
 */
int CDxlGeneric::writeData(BYTE startingAddress, BYTE dataLength, BYTE *data, bool shouldSyncWrite)
{
	   if (!shouldSyncWrite)
		{	// Direct operation on serial port

			CDxlPacket packet(mID, INST_WRITE, dataLength+1);
			// Two parameters
			packet.setParam(0, startingAddress);
			packet.setParams(1, dataLength, data);
			packet.setChecksum();

			int error = DXL_SUCCESS;

			for (int i=0 ; i<=PACKET_RETRY_FACTOR ; i++)
			{
				int sendErr = sendPacket(&packet, mRetlevel==2);														// send data request
				if (sendErr != DXL_SUCCESS)
				        if (i == PACKET_RETRY_FACTOR)
        					return sendErr;
                                        else
                                                continue;

				switch (mRetlevel)
				{
					case 0:
						i=PACKET_RETRY_FACTOR;
						mLogCrawlLn("not waiting for return packet because mReturnlevel == 0");
						break;
					case 1:
						i=PACKET_RETRY_FACTOR;
						mLogCrawlLn("not waiting for return packet because mReturnlevel == 1");
						break;
					case 2:
						{
							mLogCrawlLn("waiting for status return packet, try "<< i);
							CDxlStatusPacket statusPacket(0);
							int receiveErr = receivePacketWait(&statusPacket);
							if (receiveErr != DXL_SUCCESS)
							        if (i == PACKET_RETRY_FACTOR)
        								return receiveErr; //we return the error from the port driver if we could not receive correctly
                                                                else
                                                                        continue;

							error = statusPacket.getError();						// get status error out of received data
							if (error != DXL_CHECKSUM_ERROR)
							{
								i=PACKET_RETRY_FACTOR;								//only retry on checksum errors
							}

						}
						break;
				}
			}


			// We're done. Return status error!
			return error;
		}
	   else
	   {
	       // do group write (syncwritepacket)
		   if (mpGroup != NULL)
			   return mpGroup->writeData(mID, startingAddress, dataLength, data);
		   else
			   return DXL_PKT_SEND_ERROR;
	   }
}
