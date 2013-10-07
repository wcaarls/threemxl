/*
 * dnx-all.cpp - FTDI serial communication version
 *
 *  Created on: Nov 17, 2010
 *      Author: Erik Schuitema
 */

#include <sys/mman.h>
#include <signal.h>
#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LxSerial serialPort;

#define MAX_DEVICE_NAME_LEN 20
char devicename[MAX_DEVICE_NAME_LEN];

int gNumDynamixels	= 0;
int gDxlMinID		= 0;
int gDxlMaxID		= DXL_BROADCAST_ID-1;
//int gDynamixelIDs[gNumDynamixels]	= {100, 101, 102, 103, 104, 105}; // hard-coded IDs; legs only
CDynamixel gDynamixels[MAX_NUM_DYNAMIXELS];

bool gDxlTaskProcDone=false;
bool gMotorsInitialized=false;

bool gQuit=false;

void dxl_init_all_motors()
{
	if (!gMotorsInitialized)
	{
		printf("Detecting dynamixels in the ID range %d-%d ", gDxlMinID, gDxlMaxID);
		gNumDynamixels=0;
		// Find all dynamixels and configure and init them
		CDxlConfig dxlConfig;
		for (int iID=gDxlMinID; iID<=gDxlMaxID; iID++)
		{
			gDynamixels[gNumDynamixels].setSerialPort(&serialPort);
			gDynamixels[gNumDynamixels].setConfig(dxlConfig.setID(iID));
			if (gDynamixels[gNumDynamixels].init(false) == DXL_SUCCESS)	// false means: do not send (the default) config to motor
			{
				// Dynamixel with ID = iID responded!
				gNumDynamixels++;
				printf("\n * Dynamixel found with ID %d. Setting joint mode.", iID);
				gDynamixels[gNumDynamixels].setEndlessTurnMode(false);
			}
			else
			{
				printf(".");
				fflush(stdout);
			}
		}
		gMotorsInitialized=true;
		printf("\nDetection done. Found %d Dynamixels. %s\n", gNumDynamixels, gNumDynamixels>0?"Now reporting:\n":"Quitting.");
	}
}

void dxl_task_off_proc(void *arg)
{
	for (int iDx=0; iDx<gNumDynamixels; iDx++)
	{
		gDynamixels[iDx].enableTorque(DXL_OFF);
	}
}

void catch_signal(int sig)
{
	printf("Break signal received.\n");
	gQuit = true;
}

int main(int argc, char** argv)
{
	if (argc < 6)
	{
		printf("Usage: dxl-goto [serial device] [baud rate] [ID] [pos] [speed]\n");
		return -1;
	}

	// Set serial device name
	if (strlen(argv[1]) < MAX_DEVICE_NAME_LEN)
		strcpy(devicename, argv[1]);
	else
	{
		printf("[ERROR] Device name too long (probably not /dev/ttyUSB0 ..)!\n");
		return -1;
	}

	// Open serial port
	if (!serialPort.port_open(devicename, LxSerial::RS485_FTDI))
	{
		printf("[ERROR] Failed to open serial port!\n");
		return -1;
	}

	// Set correct baud rate
	int baudrate = atoi(argv[2]);
	serialPort.set_speed_int(baudrate);

	int ret=0;

	// Find dynamixel and configure and init
	int ID = atoi(argv[3]);
	CDxlConfig dxlConfig;
	gDynamixels[0].setSerialPort(&serialPort);
	gDynamixels[0].setConfig(dxlConfig.setID(ID));
	if (gDynamixels[0].init(false) == DXL_SUCCESS)	// false means: do not send (the default) config to motor
	{
		// Dynamixel with ID = iID responded!
		gNumDynamixels++;
		printf("\n * Dynamixel found with ID %d. Setting joint mode.\n", ID);
		gDynamixels[0].setEndlessTurnMode(false);
	}
	else
	{
		printf("\n Dynamixel not found. Quitting.\n");
		fflush(stdout);
		return -1;
	}

	// Goal position
	int pos = atoi(argv[4]);
	if ((pos < 0) || (pos > 1023))
	{
		printf("\n Error: position must be between 0 and 1023. Quitting.\n");
		return -1;
	}
	// Speed
	int speed = atoi(argv[5]);
	if ((speed < 0) || (speed > 1023))
	{
		printf("\n Error: speed must be between 0 and 1023. Quitting.\n");
		return -1;
	}

	printf("\n Press Ctrl-C to terminate\n");
	// This job can be canceled using Ctrl-C
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	//gDynamixels[0].setRawPos(pos, speed);

	gDynamixels[0].setRawPos(500, 100);
	gDynamixels[0].setCompliance(0, 32);
	usleep(1000000);
	int stepper=0;
	double samplefreq = 75.0;
	double sinefreq = 1.0;
	double amplitude = 8.0;
	while (!gQuit)
	{
		/*
		int sleeptime=500000;
		printf("Pos 400\n");
		gDynamixels[0].setRawPos(300, 0);
		gDynamixels[0].setRawPos(300, speed);
		usleep(sleeptime);
		printf("Pos 600\n");
		gDynamixels[0].setRawPos(700, 0);
		gDynamixels[0].setRawPos(700, speed);
		usleep(sleeptime);
		*/
		double speed = amplitude*sin((sinefreq*stepper/samplefreq)*6.28);
		double posdiff = speed*(1.0/samplefreq);
		printf("posdiff: %.4f(rad) (%d)\n", posdiff, (int)(posdiff/DXL_STEPS_TO_RAD));
		gDynamixels[0].getPos();
		gDynamixels[0].setPos(gDynamixels[0].presentPos() + posdiff, -1);
		//gDynamixels[0].setSpeed(speed);
		usleep((1.0/samplefreq)*1E6);
		stepper++;
	}
	gDynamixels[0].setRawPos(500, speed);
	//gDynamixels[0].setCompliance(0, 32);

	// ===== PREMATURE ENDING ====
	printf("Quitting while keeping motor operational...\n"); serialPort.port_close(); return 0;
	// ===========================

	while (!gQuit)
		pause();

	// Always turn off at the end
	dxl_task_off_proc(NULL);

	// Clean up
	serialPort.port_close();
	printf("End of dxl-all.\n");
	return 0;
}
