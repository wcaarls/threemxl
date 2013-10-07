// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXELPACKET_HPP_INCLUDED__
#define __DYNAMIXELPACKET_HPP_INCLUDED__

#include <stdio.h>
#include <string.h>
#include <ios>
#include <iostream>
#include <sstream>

#include "Byte.h"


//Packet defines
#define DXL_PKT_MAX_LENGTH				255
#define DXL_HEADER_LENGTH				5 // [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION]
#define DXL_CRC_LENGTH					1
#define DXL_MAX_NUM_PARAMS				(DXL_PKT_MAX_LENGTH - DXL_HEADER_LENGTH - DXL_CRC_LENGTH)

// Broadcast ID
#define DXL_BROADCAST_ID			0xFE



class CDxlPacket
{
	protected:
		unsigned int	mLength;
		unsigned int	mNumParams;
		bool 			mInit;
		BYTE			mData[DXL_PKT_MAX_LENGTH];

	public:
		CDxlPacket()	{mInit = false;}	// Warning; creates uninitialized packet!!

		CDxlPacket(const int ID, const BYTE instruction, const int numParams)
		{
			mData[0]	= 0xFF;
			mData[1]	= 0xFF;
			mData[2]	= ID;
			mData[3]	= numParams + 2;	// length of the packet according to dynamixel definition
			mData[4]	= instruction;
			mLength		= numParams + DXL_HEADER_LENGTH + DXL_CRC_LENGTH;	// true size of the packet: numParams + header + checksum

			mNumParams	= numParams;
			mInit = true;
		}

		void reset()
		{
			mData[0]	= 0xFF;
			mData[1]	= 0xFF;
			mData[2]	= 0;
			mData[3]	= 2;	// length of the packet according to dynamixel definition
			mData[4]	= 0;
			mLength		= DXL_HEADER_LENGTH + DXL_CRC_LENGTH;	// true size of the packet: numParams + header + checksum

			mNumParams	= 0;
			mInit = true;
		}

		inline void		setID(const int ID)					{mData[2]	= ID;}
		inline BYTE		length()							{return mLength;}
		inline BYTE*	data()								{return mData;}
		inline BYTE		getError()							{return mData[4];}
		inline BYTE		getID()								{return mData[2];}

		// adjustLength() allows to adjust the length fields (2!) of the packet without calling new/delete
		inline void		adjustLength(const int numParams)
		{
			mData[3]	= numParams + 2;
			mLength		= numParams + DXL_HEADER_LENGTH + DXL_CRC_LENGTH;
			mNumParams	= numParams;
//			printf("Number of params = %d \n",mNumParams);
		}

		// Checksum functionality
		inline BYTE		readChecksum()						{return mData[mLength-1];}
		inline void		setChecksum()						{mData[mLength-1] = calcChecksum();}
		inline BYTE		calcChecksum()
		{
			BYTE result=0;
			for (unsigned int i=2; i<mLength-1; i++)
				result += mData[i];

			return ~result;
		}

		// Interface for the parameters of the packet
		inline BYTE		getParam(const BYTE paramIndex)										{return mData[DXL_HEADER_LENGTH + paramIndex];}
		inline void		getParams(const BYTE paramIndex, const BYTE numParams, BYTE* data)	{memcpy(data, mData + DXL_HEADER_LENGTH + paramIndex, numParams);}
		inline void		setParam(const BYTE paramIndex, const BYTE value)					{mData[DXL_HEADER_LENGTH + paramIndex] = value;}
		inline void		setParams(const BYTE paramIndex, const BYTE numParams, BYTE* data)	{memcpy(mData + DXL_HEADER_LENGTH + paramIndex, data, numParams);}
		inline void 	setInstruction(const BYTE instruction)								{mData[4]	= instruction;}
		inline void		clearParams()														{memset(mData + DXL_HEADER_LENGTH, 0, mNumParams);}

		// getParamWord and setParamWord: paramIndex is always counted in BYTES!
		inline WORD		getParamWord(const BYTE paramIndex)									{return *(WORD*)(mData + DXL_HEADER_LENGTH + paramIndex);}
		inline void		setParamWord(const BYTE paramIndex, const WORD value)				{*(WORD*)(mData + DXL_HEADER_LENGTH + paramIndex) = value;}


		std::string			getPktString()
		{
			return getPktString(mLength);
		}
		std::string			getPktString(unsigned char length)
		{
			if(mInit)
			{
				using namespace std;
				stringstream pktString;
				for (unsigned int i = 0; i < length; i++)
				{
					pktString << hex << (unsigned int)mData[i] << " ";
				}
				return pktString.str();
			}
			else
				return "";
		}

};


class CDxlStatusPacket: public CDxlPacket
{
	public:
		CDxlStatusPacket(const int numParams):
			CDxlPacket()
		{
			mLength = numParams + DXL_HEADER_LENGTH + DXL_CRC_LENGTH;
			mNumParams	= numParams;
			memset(mData, 0, mLength);
			mInit = true;
		}
};

class CDxlSyncWritePacket: public CDxlPacket
{
	protected:
	int 			mStartingAddress;
	int 			mDxlPointer;
	int				mNumParamsPerDxl;

		// adjustLengthSyncWrite() adjusts the packet's size for use with the SYNC_WRITE packet (multiple dxl's).
		inline void		adjustLengthSyncWrite(const int numParamsPerDynamixel, const int numDynamixels)
		{
			// numRegularParameters = (numParameters + sizeof(ID))*numDynamixels + [start address] + [length of data]
			adjustLength((numParamsPerDynamixel+1)*numDynamixels + 2);
		}

	public:

		CDxlSyncWritePacket():
			CDxlPacket(DXL_BROADCAST_ID, 0x83, 0)//INST_SYNC_WRITE
		{
			mNumParamsPerDxl = 0;
			mStartingAddress = 0;
			mDxlPointer = 2;
		}

		// instruction should be either INST_SYNC_WRITE or INST_SYNC_REG_WRITE
		// Make sure you call configurePacket() before using the packet.
		CDxlSyncWritePacket(const BYTE instruction):
			CDxlPacket(DXL_BROADCAST_ID, instruction, 0)
		{
			mNumParamsPerDxl = 0;
			mStartingAddress = 0;
			mDxlPointer = 2;
		}

		/**
		 * configurePacket makes an empty group packet that is of a specific command type
		 * i.e. it has a single address location with different parameters for each servo
		 * To add the individual data for each servo call the function addCommand(...)
		 * afterwards
		 */
		void	configurePacket(const int numDynamixels, const BYTE startRegister, const int numParamsPerDynamixel)
		{
			setID(DXL_BROADCAST_ID);
			// Adjust the length of our mSyncPacket and clear it
			adjustLengthSyncWrite(numParamsPerDynamixel, numDynamixels); // Resizes the packet for the number of added dynamixels and the desired parameters settings
			clearParams();
			// Set the first two parameters
			setParam(0, startRegister);
			setParam(1, numParamsPerDynamixel);
			setInstruction(0x83);
			mStartingAddress = startRegister;
			mNumParamsPerDxl = numParamsPerDynamixel;
		}

//		// this function retrieves the location inside the packet where a dynamixel can write its data (ID + params)
//		inline BYTE*	getSyncWriteLocation(const int dynamixelIndex)
//		{
//			// The first parameter in a SYNC_WRITE packet is located at mData + DXL_HEADER_LENGTH
//			// See adjustLengthSyncWrite() for further explanation
//			return mData + DXL_HEADER_LENGTH + 2 + dynamixelIndex*(mNumParamsPerDxl+1);
//		}

		/**
		 * addCommand adds a command to a prepared packet for a specific servo
		 */
		void addCommand(const int ID, BYTE* data, int dataLength)
		{
			setParam(mDxlPointer,ID);
			mDxlPointer++;
			setParams(mDxlPointer, dataLength, data);
			mDxlPointer += dataLength;
		}

		/**
		 * resets the group packet.
		 */
		void reset()
		{
			CDxlPacket::reset();
			mNumParamsPerDxl = 0;
			mStartingAddress = 0;
			mDxlPointer = 2;
		}

		/**
		 * This function returns the address that is to be handled by the group packet
		 * In this way you can check if the group packet is of the correct type
		 */
		int getStartingAddress() {return mStartingAddress;}

};



#endif // __DYNAMIXELPACKET_HPP_INCLUDED__
