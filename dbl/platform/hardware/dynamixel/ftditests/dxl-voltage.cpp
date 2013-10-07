/*
 * dxl-voltage.cpp - FTDI serial communication version
 *
 *  Created on: Dec 23, 2010
 *      Author: Erik Schuitema
 */

#include <sys/mman.h>
#include <signal.h>
#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <randomc.h>

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
			if (gDynamixels[gNumDynamixels].init(DXL_PING_TIMEOUT, false) == DXL_SUCCESS)	// false means: do not send (the default) config to motor
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
	if (argc < 5)
	{
		printf("Usage: dxl-goto [serial device] [baud rate] [ID] [voltage ratio -1..1] (transition fraction)\n");
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
	if (gDynamixels[0].init(DXL_PING_TIMEOUT, false) == DXL_SUCCESS)	// false means: do not send (the default) config to motor
	{
		// Dynamixel with ID = iID responded!
		gNumDynamixels++;
		printf("\n * Dynamixel found with ID %d. Setting voltage mode.\n", ID);
		gDynamixels[0].setEndlessTurnMode(true);
	}
	else
	{
		printf("\n Dynamixel not found. Quitting.\n");
		fflush(stdout);
		return -1;
	}

	// Desired voltage
	double voltageRatio = atof(argv[4]);
	if ((voltageRatio < -1.0) || (voltageRatio > 1.0))
	{
		printf("\n Error: voltage ratio must be between -1.0 and 1.0. Quitting.\n");
		return -1;
	}

	// Desired fraction of transition steps (e.g., 0.2 means 20% of the steps are used as intermediate voltages between levels)
	double transitionFraction = atof(argv[5]);
	if ((transitionFraction < 0.0) || (transitionFraction > 1.0))
	{
		printf("\n Error: transition fraction must be between 0.0 and 1.0. Quitting.\n");
		return -1;
	}

	printf("\n Press Ctrl-C to terminate\n");
	// This job can be canceled using Ctrl-C
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	gDynamixels[0].setEndlessTurnTorque(voltageRatio);

	gRanrotB.RandomInit(123);
	int stepper=0;
	double samplefreq = 300.0;
	double actuationfreq = 30.0;
	int actuationDivider = floor(samplefreq/actuationfreq + 0.5);
	double lastVoltageRatio = 0.0;
	int mode=0;	// Mode 0: transition steps, mode 1: normal steps
	int numTransitionSteps = floor(transitionFraction*actuationDivider + 0.5);	// Number of intermediate levels between old voltage and new voltage
	int numNormalSteps = actuationDivider - numTransitionSteps;
	printf("Running at %.2fHz, changing actuation pattern at %.2fHz using %d transition steps (=%.1f%%)\n", samplefreq, samplefreq/actuationDivider, numTransitionSteps, 100.0*(double)numTransitionSteps/actuationDivider);
	double newVoltageRatio = 0.0;
	while (!gQuit)
	{
		if (stepper == (numTransitionSteps+numNormalSteps))
		{
			lastVoltageRatio = newVoltageRatio;
			newVoltageRatio = -fabs(voltageRatio) + 2.0*fabs(voltageRatio)*gRanrotB.Random();
			//printf("New voltage ratio calculated: %.3f\n", newVoltageRatio);
			stepper = 0;
		}

		double actuationRatio = lastVoltageRatio + (newVoltageRatio - lastVoltageRatio)*(std::min(1.0, (double)(stepper+1)/(numTransitionSteps+1)));
		//printf("Setting voltage ratio to: %.3f\n", actuationRatio);
		// Actuate!
		gDynamixels[0].setEndlessTurnTorque(actuationRatio);

		usleep(1.0E6/samplefreq);
		stepper++;
	}

	// ===== PREMATURE ENDING ====
	//printf("Quitting while keeping motor operational...\n"); serialPort.port_close(); return 0;
	// ===========================

	//while (!gQuit)
		//pause();

	// Always put the motor in position mode before shutting down to work around the bug with magnetic encoders in Dynamixels
	// Turn the torque off first to prevent the motor from trying to reach the *last known* position goal
	printf("Putting %d motors back in position mode..\n", gNumDynamixels);
	for (int iDxl=0; iDxl<gNumDynamixels; iDxl++)
	{
		gDynamixels[iDxl].enableTorque(DXL_OFF);
		gDynamixels[iDxl].setEndlessTurnMode(false);
	}

	// Always turn off at the end
	dxl_task_off_proc(NULL);

	// Clean up
	serialPort.port_close();
	printf("End of dxl-all.\n");
	return 0;
}
