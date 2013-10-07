// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXELGROUP_H_INCLUDED__
#define __DYNAMIXELGROUP_H_INCLUDED__

#include "CDxlGeneric.h"
#include "CDxlPacket.hpp"
#include "CDxlConfig.h"
#include <threemxl/platform/io/logging/Log2.h>


// Forward declarations
class CDxlGeneric;
class CDxlConfig;
class CDxlGroupConfig;
class CDxlSyncWritePacket;

class CDxlGroup: public CDxlCom
{
	protected:
		std::string				mName;								// Unique name of the group. Generated in setSerialPort using the serial port name.
		CDxlGeneric*			mDynamixels[MAX_NUM_DYNAMIXELS];	// Array of dynamixel pointers
		int						mNumDynamixels;
		CDxlSyncWritePacket*	mSyncPacket; //< this is the goup packet that is used for group messages
		int						initAll();
		int						action();	// Sends the ACTION command with the broadcast ID.
		CLog2					mLog;
		bool          mSyncRead;
		CDxlStatusPacket* mStatusPackets[MAX_NUM_DYNAMIXELS];
	public:
						CDxlGroup();
		virtual			~CDxlGroup();
		CDxlGeneric*	getDynamixel(const int index);
		int				getNumDynamixels();
		std::string&	getName();
		int				getStateAll();
		int				getPosAll();
		int 			getStatusAll();
		int				getPosAndSpeedAll();
		int 			getPosSpeedTorqueAll();
		int				getTempAll();	// Request temperature of all dynamixels in this group
		int				enableTorqueAll(int state);
		int				setEndlessTurnModeAll(bool enabled);


		/**
		 * Groups can write command to servo's in a group message. To do this the individual servo's
		 * write their data to a group packet with the writeData function below. With the sendSyncWrite
		 * function you can send the packet.
		 *
		 * Documentation from Robotis RX28 about the syncwrite packect:
		 *   This command is used to control several RX-28s simultaneously with one Instruction
		 *   Function Packet transmission. When this command is used, several commands are transmitted
		 *   at once, so that the communication time is reduced when multiple RX-28s are controlled.
		 *   However, the SYNC WRITE command can be used only if both of the address and
		 *   length of the Control Table to write is identical. Besides, ID should be transmitted as
		 *   Broadcasting ID. Make sure that the length of packet does not to exceed 143 bytes since
		 *   the volume of receiving buffer of RX-28 is 143 bytes.
		 */
		int				sendSyncWritePacket();			// Sends out a group packet that has been filled with data by the inividual servo's
		int				writeData(int ID, int startingAddress, int dataLength, BYTE* data); //< servo's can use this function to write data to a group packet in stead of directly to a serial port
		void 			setSyncWriteType(const BYTE instruction=INST_SYNC_WRITE);

		// Read a certain part from the control table of all dynamixels in the group. interpretControlData
		// will be called on each dynamixel afterwards.
		int       syncRead(int startingAddress, int dataLength);

		// Automatic configuration
		int				setConfig(CDxlGroupConfig *config); //< Loads group configuration class and adds servo to group. Returns 0 on success.

		// Manual configuration
		void			setName(const std::string& name);
		void			setSerialPort(LxSerial* serialport);
		int				addNewDynamixel(CDxlConfig* config); //< Adds servo to group. Returns 0 on success
		int       setupSyncReadChain();

		// Init and deinit. Make sure you set the configuration before calling init!
		virtual bool	init();
		virtual bool	deinit();

};


#endif //__DYNAMIXELGROUP_H_INCLUDED__
