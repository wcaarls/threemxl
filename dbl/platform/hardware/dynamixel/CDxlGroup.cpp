// Dynamixel control code - C++ file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#include <threemxl/platform/hardware/dynamixel/CDxlGroup.h>
#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>
#include <threemxl/platform/hardware/dynamixel/3mxl/3mxl.h>
#include <threemxl/platform/hardware/dynamixel/DxlClassFactory.hpp>


//***********************************************************//
//********************* CDynamixelGroup *********************//
//***********************************************************//

CDxlGroup::CDxlGroup(): CDxlCom(), mLog("CDxlGroup")
{
	mSerialPort			= NULL;
	mNumDynamixels		= 0;
	mSyncPacket = new CDxlSyncWritePacket();
	mLog.setLevel(llCrawl);
	mSyncRead = false;
//	mLog.enableConsoleOutput("false");
}

CDxlGroup::~CDxlGroup()
{
	// Destroy all dynamixels
	for(int iDxl = 0; iDxl < mNumDynamixels; iDxl++)
	{
		if (mDynamixels[mNumDynamixels] != NULL)
		{
			delete mDynamixels[mNumDynamixels];
			mDynamixels[mNumDynamixels] = NULL;
			mLogCrawlLn("deleted dynamixel object " << iDxl << " from serialport group " << getName() );
		}
	}
	delete mSyncPacket;
}

void CDxlGroup::setSerialPort(LxSerial* serialport)
{
	mSerialPort	= serialport;
	mName		= "DxlGroup-" + serialport->get_port_name();
}

void CDxlGroup::setName(const std::string& name)
{
	mName = name;
}

std::string& CDxlGroup::getName()
{
	return mName;
}

bool CDxlGroup::init()
{
	return (initAll() == DXL_SUCCESS);
}

bool CDxlGroup::deinit()
{
	//delete all dynamixel objects in this group.
	for(int iDxl = 0; iDxl < mNumDynamixels; iDxl++)
	{
		if (mDynamixels[iDxl] != NULL)
		{
			delete mDynamixels[iDxl];
			mDynamixels[iDxl] = NULL;
			mLogCrawlLn("deleted dynamixel object " << iDxl << " from serialport group " << getName() );
		}
		else
		{
			mLogErrorLn("unable to delete dynamixel object "<< iDxl << " from serialport group " << getName() );
		}
		mNumDynamixels =0;
	}
	return true;
}

int CDxlGroup::action()
{
	CDxlPacket packet(DXL_BROADCAST_ID, INST_ACTION, 0);
	packet.setChecksum();
	return sendPacket(&packet, false);
}

int CDxlGroup::addNewDynamixel(CDxlConfig* config)
{
	CDxlGeneric* pDxl =  gDxlCreate(config->mDxlTypeStr);
	if(pDxl==NULL)
	{
#ifdef __DBG__
		printf("Dynamixel with wrong type was not added to group!\n");
#endif
		return DXL_NOT_INITIALIZED;
	}
	else
	{
		mDynamixels[mNumDynamixels] = pDxl;
		mDynamixels[mNumDynamixels]->setConfig(config);
		mDynamixels[mNumDynamixels]->setGroup(this);		//<register this group with servo for groupwrites
		mNumDynamixels++;
	}
	return 0;
}

int CDxlGroup::setupSyncReadChain()
{
  int result = DXL_SUCCESS;
  for (int i=0; i<mNumDynamixels; i++)
  {
    int newResult = mDynamixels[i]->setSyncReadIndex(i+1);
    #ifdef __DBG__
    if (newResult != DXL_SUCCESS)
      mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ") while setting up sync read chain" );
    #endif

    result |= newResult;
  }

  if (result == DXL_SUCCESS )
    mSyncRead = true;

  return result;
}

/**
 * This function sends the group packed that has been filled by the dynamixels/3mxls
 * after sending it resets the group message
 */
int CDxlGroup::sendSyncWritePacket()
{
	if (mNumDynamixels > 0)
	{
		if (mSyncPacket->getID() != 0xFE)
		{
			mLogCrawlLn("did not send empty syncWritePacket...");
			return DXL_SUCCESS;
		}
		mSyncPacket->setChecksum();
//		std::cout << "sending packet: " << mSyncPacket->getStartingAddress() << endl;
		int result = sendPacket(mSyncPacket, false);
		mLogCrawlLn("reset syncWritePacket");
		mSyncPacket->reset();	//clear syncPacket for next syncwrite
		return result;
	}
	else
		return DXL_SUCCESS;

	// No status packet(s) will be returned since a SYNC WRITE uses the broadcast ID
}

/**
 * This function is used by the CDxlGeneric type classes to write their data to a
 * Group packet in stead of directly to the serial port. If the Servo was added to
 * a group it will have been given a pointer to this group so it can write its data
 * to the group. You can then send the group packet with sendSyncWritePacket().
 */
int	CDxlGroup::writeData(int ID, int startingAddress, int dataLength, BYTE* data)
{
	   if (mSyncPacket->getStartingAddress() == 0) //we are starting a new syncwrite
	   {
		   mLogCrawlLn("configuring syncWritePacket for "<< mNumDynamixels <<" servo's with starting address "<<startingAddress << " and dataLength "<< dataLength );
		   mSyncPacket->configurePacket(mNumDynamixels, startingAddress, dataLength);
	   }
	   else // Make sure that consecutive servo's are building the same packet
	   {
	      if(mSyncPacket->getStartingAddress() != startingAddress)
	      {
	    	  mLogErrorLn("Trying to write different messages in same syncWritePacket "
	    			  << "expecting:"
	    			  << mSyncPacket->getStartingAddress()
	    			  << ", getting:"
	    			  << startingAddress);
	    	  return DXL_INVALID_PARAMETER;
	      }
	   }
	   // Now add ID and data at the end of the writing position in the sync write packet
	   mSyncPacket->addCommand(ID, data, dataLength);
	   return DXL_SUCCESS;
}

int CDxlGroup::syncRead(int startingAddress, int dataLength)
{
  // Construct sync read packet
  CDxlPacket packet(DXL_BROADCAST_ID, INST_READ, 2);
  packet.setParam(0, startingAddress);
  packet.setParam(1, dataLength);
  packet.setChecksum();

  // Send over bus
  if (sendPacket(&packet) != DXL_SUCCESS)
  {
    mLogErrorLn("Couldn't send sync read packet " << packet.getPktString());
    return DXL_ERROR;
  }

  for (int i=0; i<mNumDynamixels; i++)
  {
    // Read status packets in sequence (only works if chain was properly set up)
    CDxlStatusPacket statusPacket(dataLength);
    int result = receivePacketWait(&statusPacket);

    if (result != DXL_SUCCESS)
    {
      mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(result) << "(last error = " << mDynamixels[i]->getLastError() << ") during sync read" );
      return result;
    }

    mDynamixels[i]->interpretControlData(startingAddress, dataLength, statusPacket.data()+DXL_HEADER_LENGTH);
  }

  return DXL_SUCCESS;
}

/**
 * enable group to set instruction type for syncwrite
 */
void CDxlGroup::setSyncWriteType(const BYTE instruction)
{
	mSyncPacket->setInstruction(instruction);
}

int CDxlGroup::initAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		if(mDynamixels[i] != NULL)
		{
			mDynamixels[i]->setSerialPort(mSerialPort);
			result |= mDynamixels[i]->init();
			if(result != DXL_SUCCESS){
				mLogErrorLn("Servo " << i << " did not initialize correctly! Error:" << result);
				break;
			}
		}
		else
		{
			mLogErrorLn("Trying to initialize non-existent servo with nr:" << i);
		}
	}

	return result;
}

int CDxlGroup::getStateAll()
{
  if (mSyncRead)
  {
    return syncRead(M3XL_VOLTAGE_L, 10);
  }
  else
  {
    int result = DXL_SUCCESS;
    for (int i=0; i<mNumDynamixels; i++)
    {
      int newResult = mDynamixels[i]->getState();
      #ifdef __DBG__
      if (newResult != DXL_SUCCESS)
        mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
      #endif

      result |= newResult;
    }

    return result;
  }
}

int CDxlGroup::getStatusAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->getStatus();
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::getPosAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->getPos();
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::getPosAndSpeedAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->getPosAndSpeed();
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}


int CDxlGroup::getPosSpeedTorqueAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->getTorquePosSpeed();
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::getTempAll()
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->getTemp();
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::setEndlessTurnModeAll(bool enabled)
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->setEndlessTurnMode(enabled);
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::enableTorqueAll(int state)
{
	int result = DXL_SUCCESS;
	for (int i=0; i<mNumDynamixels; i++)
	{
		int newResult = mDynamixels[i]->enableTorque(state);
		#ifdef __DBG__
		if (newResult != DXL_SUCCESS)
			mLogErrorLn("Dynamixel with ID " << mDynamixels[i]->getID() << " returned " << translateErrorCode(newResult) << "(last error = " << mDynamixels[i]->getLastError() << ")!" );
		#endif

		result |= newResult;
	}

	return result;
}

int CDxlGroup::getNumDynamixels()
{
	return mNumDynamixels;
}

CDxlGeneric* CDxlGroup::getDynamixel(const int index)
{
	return mDynamixels[index];
}


int CDxlGroup::setConfig(CDxlGroupConfig* config)
{
	int result = 0;
	for (int i=0; i<config->getNumDynamixels(); i++)
	{
		result = addNewDynamixel(config->getDynamixelConfig(i));
	}
	return result;
}
