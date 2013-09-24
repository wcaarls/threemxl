// testdxlmultiport: Test program to evaluate functionality of
// multiple, parallel (asynchronous) working real-time serial ports
// probing dynamixel servo motors.
//
// FTDI serial communication version
//  Created on: Nov 17, 2010
//     Author: Erik Schuitema

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <PosixNonRealTimeThread.hpp>
#include <Stopwatch.hpp>
#include <stdlib.h>

#include <DynamixelAsync.h>

int timer_started = 0;

#define MAX_DEVICE_NAME_LEN		20
#define MAX_NUM_SERIAL_PORTS	2

int gNumSerialPorts = 0;
char devicename[MAX_NUM_SERIAL_PORTS][MAX_DEVICE_NAME_LEN];

// Sample rate counters
double gTotalStepTime=0;
int gTotalNumSteps=0;

bool gQuit=false;

class CDxlMultiportTester: public PosixNonRealTimeThread
{
	protected:
		LxSerial			mSerialPort[MAX_NUM_SERIAL_PORTS];
		CDxlGroupAsync		mDxlGroup[MAX_NUM_SERIAL_PORTS];
		CDxlCmdInterface*	mDxlCmdInterface[MAX_NUM_SERIAL_PORTS];

	public:
		bool mEcho;
		bool mEchoVel;

		CDxlMultiportTester(const std::string& name):
			PosixNonRealTimeThread(name, BaseThread::NORMAL)
		{
			mEcho = false;
			mEchoVel = false;
		}


		//printf("testdxlmultiport [baudrate] [rtserialport1] [none|ID,ID,ID,...] [rtserialport2] [none|ID,ID,ID,...] [(blank)|ECHO|ECHOVEL]\n");
		bool init(char* argv[])
		{
			// Set serial device names and create dxl groups
			for (int iPort=0; iPort<gNumSerialPorts; iPort++)
			{
				if (strlen(argv[2+2*iPort]) < MAX_DEVICE_NAME_LEN)
					strcpy(devicename[iPort], argv[2+2*iPort]);
				else
					printf("Device name too long (probably not /dev/ttyUSB0 ..)!\n");

				// Save dynamixel IDs
				int tempIDs[MAX_NUM_DYNAMIXELS];
				int numDynamixels=0;
				// If the user provides "none", no dynamixels will be added to this group
				if (strcmp(argv[3+2*iPort], "none") != 0)
				{
					bool stop=false;
					int iID=0;
					char strAllIDs[1000];
					strcpy(strAllIDs, argv[3+2*iPort]);

					while (!stop)
					{
						char* IDstr = strrchr(strAllIDs, ',');
						if (IDstr != NULL)
						{
							tempIDs[iID] = atoi(IDstr + 1);
							IDstr[0] = 0;
							iID++;
						}
						else
						{
							tempIDs[iID] = atoi(strAllIDs);
							stop = true;
							numDynamixels = iID+1;

							// Add dynamixels with the found IDs to the group
							CDxlConfig dxlConfig;
							dxlConfig.mReturnDelay = 4;
							for (iID=0; iID<numDynamixels; iID++)
								mDxlGroup[iPort].addNewDynamixel(dxlConfig.setID(tempIDs[numDynamixels-iID-1]));

							// Report
							printf("Group %d is active and contains %d dynamixels.\n", iPort, mDxlGroup[iPort].getNumDynamixels());
						}
					}
				}
			}

			// Set correct baud rate
			int baudrate = atoi(argv[1]);
			// Create serial ports and init dxl groups with the ports
			for (int iPort=0; iPort<gNumSerialPorts; iPort++)
			{
				if (mSerialPort[iPort].port_open(devicename[iPort], LxSerial::RS485_FTDI))
				{
					// printf("Serial port %s opened\n", devicename);
					// Set correct baud rate
					mSerialPort[iPort].set_speed_int(baudrate);
					//mSerialPort[iPort].set_fifo_depth(RTSER_FIFO_DEPTH_8);
					mDxlGroup[iPort].setSerialPort(&mSerialPort[iPort]);
					if (!mDxlGroup[iPort].init())
						printf("Initialization of dynamixel group \"%s\" failed!\n", mDxlGroup[iPort].getName().c_str());
					// Request command interface
					mDxlCmdInterface[iPort] = mDxlGroup[iPort].createCommandInterface();
				}
				else
				{
					printf("FAILED to open serial port \"%s\"!\n", devicename[iPort]);
					return false;
				}
			}
			return true;
		}

		bool deinit()
		{
			// Delete dxl groups and serial ports
			for (int iPort=0; iPort<gNumSerialPorts; iPort++)
			{
				mDxlGroup[iPort].deinit();
				mSerialPort[iPort].port_close();
			}
			return true;
		}

	protected:
		void run()
		{
			// Execute the following code a number of times. In ECHO mode: VERY often. In non-ECHO mode (benchmark mode): 2000 times.
			//SRTIME tStart, tEnd;
			Stopwatch stopwatch;
			// Create command objects
			CDxlGrpCmd_getPosAll* cmd[2];
			cmd[0] = new CDxlGrpCmd_getPosAll(&mDxlGroup[0]);
			cmd[1] = new CDxlGrpCmd_getPosAll(&mDxlGroup[1]);
			// Start timing
			stopwatch.start();
			for (int iRep=0; iRep<((mEcho|mEchoVel)?999999999:999999999); iRep++)
			{

				// Synchronous getStateAll
				//for (int iGroup=0; iGroup<NUM_SERIAL_PORTS; iGroup++)
				//	mDxlGroup[iGroup]->getStateAll();



				// Asynchronous getStateall
				// First, send get state command to both groups


				for (int iGroup=0; iGroup<gNumSerialPorts; iGroup++)
				{
					mDxlCmdInterface[iGroup]->sendCommand(cmd[iGroup]);
				}

				// Then, wait for both groups to finish all transactions
				for (int iGroup=0; iGroup<gNumSerialPorts; iGroup++)
				{
					mDxlCmdInterface[iGroup]->waitForTransactions();
				}

				// Keep track of number of iterations
				gTotalNumSteps++;

				for (int iGroup=0; iGroup<gNumSerialPorts; iGroup++)
				{
					// Start of echo line
					if (mEcho)
						printf("POS|");
					if (mEchoVel)
						printf("VEL|");
					// Middle of echo line
					for (int iDxl=0; iDxl<mDxlGroup[iGroup].getNumDynamixels(); iDxl++)
					{
						if (mEcho)
							printf(" %d>%.2f (raw: %hu)", mDxlGroup[iGroup].getDynamixel(iDxl)->getID(), mDxlGroup[iGroup].getDynamixel(iDxl)->presentPos()*180.0/M_PI, mDxlGroup[iGroup].getDynamixel(iDxl)->presentRawPos());
						if (mEchoVel)
							printf(" %d>%.2f", mDxlGroup[iGroup].getDynamixel(iDxl)->getID(), mDxlGroup[iGroup].getDynamixel(iDxl)->presentSpeed()*180.0/M_PI);
					}
					// End of echo line
					if (mEcho | mEchoVel)
						printf("\n");
				}
				//usleep((int)2E5); //200ms

				if (gQuit)
					break;
			}
			// Store total run time
			stopwatch.stop();
			gTotalStepTime = stopwatch.elapsed();

			// Throw away the command objects
			delete cmd[0];
			delete cmd[1];

			printf("Dynamixel multiport test done! (Press CTRL-C to exit if you didn't already.)\n");
		}
};

void catch_signal(int sig)
{
	gQuit = true;
	printf("\nSignal received, stopping.\n");
}

void dynamixel_test_proc(void *arg)
{
}

int main(int argc, char* argv[])
{
	CDxlMultiportTester multiportTester("RT-testdxlmultiport");

	// Start with numParams for 1 port
	int numParams = 3;
	gNumSerialPorts = 1;

	if (argc < numParams+1)
	{
		printf("Usage:\n");
		printf("testdxlmultiport [baudrate] [serial device 1] [ID,ID,ID,...][(blank)|ECHO|ECHOVEL]\n\tor\n");
		printf("testdxlmultiport [baudrate] [serial device 1] [none|ID,ID,ID,...] [none|serial device 2] [none|ID,ID,ID,...] [(blank)|ECHO|ECHOVEL]\n");
		return -1;
	}
	else
	{
		if (argc > numParams+2)
		{
			numParams += 2;
			gNumSerialPorts++;
		}
		// Set echo mode
		if (argc > numParams+1)
		{
			if (strcmp(argv[numParams+1],"ECHO") == 0)
				multiportTester.mEcho = true;
			else
				multiportTester.mEcho = false;

			if (strcmp(argv[numParams+1],"ECHOVEL") == 0)
				multiportTester.mEchoVel = true;
			else
				multiportTester.mEchoVel = false;
		}

		multiportTester.init(argv);
	}

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
	printf("Use Ctrl-C to exit program.\n");

	multiportTester.start();

	// Wait for break signal. Use the while loop + gQuit to prevent SIGWINCH (console resizing) from quitting the program
	while (!gQuit)
		pause();

	multiportTester.stop();
	multiportTester.awaitTermination();

	multiportTester.deinit();

	printf("[STATS] Average read-out frequency: %.2fHz\n", 1E9/(gTotalStepTime/gTotalNumSteps));

	printf("End of program\n");

	return 0;
}
