/*
 * dxl-changeid.cpp - FTDI serial communication version
 *
 *  Created on: Nov 17, 2010
 *      Author: Erik Schuitema
 */

#include <sys/mman.h>
#include <signal.h>
#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

LxSerial serialPort;

#define MAX_DEVICE_NAME_LEN 20
char devicename[MAX_DEVICE_NAME_LEN];

int gNumDynamixels	= 0;
int gOldID			= 1;
int gNewID			= 1;
int gNewBaudRate	= -1;
CDynamixel gDynamixel;

bool gDxlTaskProcDone=false;

bool gQuit=false;

bool dxl_init_all_motors()
{
	printf("Trying to init Dynamixel with ID %d ...", gOldID);
	// Find all dynamixels and configure and init them
	CDxlConfig dxlConfig;
	gDynamixel.setSerialPort(&serialPort);
	gDynamixel.setConfig(dxlConfig.setID(gOldID));
	int initResult = gDynamixel.init(false);
	if (initResult == DXL_SUCCESS)	// false means: do not send (the default) config to motor
	{
		// Dynamixel with ID = iID responded!
		printf("success!\n");
		return true;
	}
	else
	{
		printf("failed (error = %d).\n", initResult);
		fflush(stdout);
		return false;
	}
}

void dxl_task_change_proc(void *arg)
{
	// Check if init worked
	if (!dxl_init_all_motors())
	{
		printf("Check if initial baud rate and ID were set correctly. Quitting.\n");
		gDxlTaskProcDone = true;
		return;
	}

	printf("Changing ID to %d...", gNewID);
	int error = gDynamixel.changeID(gNewID);
	if (error == DXL_SUCCESS)
		printf("done! ID is now %d!\n", gDynamixel.getID());
	else
	{
		printf("failed (error %d)\n", error);
		printf("Could not set new ID. Quitting.\n");
		gDxlTaskProcDone = true;
		return;
	}

	// Change baud rate if needed
	if (gNewBaudRate > 0)
	{
		// Send new baud rate command
		gDynamixel.setBaudRate(gNewBaudRate);
		// Change serial port speed
		serialPort.set_speed_int(gNewBaudRate);
	}

	int pingResult = gDynamixel.ping();
	if (pingResult == DXL_SUCCESS)
		printf("Final ping check .. done!\n");
	else
		printf("Final ping check .. fail! (ID = %d; error = %d)\nSomething went wrong!?\n", gDynamixel.getID(), pingResult);

	gDxlTaskProcDone = true;
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
		printf("Usage: dxl-changeid [serial device] [baud-rate] [old-ID] [new-ID] [new baud rate (optional)]\n");
		return -1;
	}
	else
	if (argc > 6)
	{
		printf("Too many arguments!\n");
		printf("Usage: dxl-changeid [serial device] [baud-rate] [old-ID] [new-ID] [new baud rate (optional)]\n");
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

	// Create dynamixel task - not needed anymore
	int ret = 0;

	// Detect old ID
	gOldID = atoi(argv[3]);

	// Detect new ID
	gNewID = atoi(argv[4]);

	// Detect possible new baud rate
	if (argc == 6)
		gNewBaudRate = atoi(argv[5]);

	// Start the appropriate task
	dxl_task_change_proc(NULL);

	// Now wait for the main proc to finish
	while (!gDxlTaskProcDone)
		usleep((int)1E5);	// 100ms

	// Clean up
	serialPort.port_close();
	printf("End of dxl-all.\n");
	return 0;
}
