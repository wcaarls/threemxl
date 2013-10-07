/*
 * 3mxl-all.cpp nonrt version
 *
 *  Created on: Oct 20, 2008
 *      Author: erik / eelko
 */

#include <sys/mman.h>
#include <signal.h>
#include <threemxl/platform/hardware/dynamixel/3mxl/3mxl.h>


#include <stdio.h>
//#include <stdio.h>
//#include <signal.h>
//#include <unistd.h>
//#include <sys/io.h>
#include <stdlib.h>
//#include <conio.h>
#include <string.h>

LxSerial serialPort;

#define MAX_DEVICE_NAME_LEN 20
char devicename[MAX_DEVICE_NAME_LEN];

int gNumDynamixels	= 0;
int gDxlMinID		= 0;
int gDxlMaxID		= DXL_BROADCAST_ID-1;
//int gDynamixelIDs[gNumDynamixels]	= {100, 101, 102, 103, 104, 105}; // hard-coded IDs; legs only
C3mxl gDynamixels[MAX_NUM_DYNAMIXELS];

bool gDxlTaskProcDone=false;
bool gMotorsInitialized=false;

bool gQuit=false;

void dxl_init_all_motors()
{
	if (!gMotorsInitialized)
	{
		printf("Detecting 3mxls in the ID range %d-%d ", gDxlMinID, gDxlMaxID);
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
				printf("\n * Dynamixel found with ID %d ", iID);
			}
			else
			{
				printf(".");
				fflush(stdout);
			}
		}
		gMotorsInitialized=true;
		printf("\nDetection done. Found %d 3mxls. %s\n", gNumDynamixels, gNumDynamixels>0?"Now reporting:\n":"Quitting.");
	}
}

void dxl_task_on_proc(void *arg)
{
	dxl_init_all_motors();
	for (int iDx=0; iDx<gNumDynamixels; iDx++)
	{
		gDynamixels[iDx].enableTorque(DXL_ON);
	}
	// Do not automatically terminate; do not use gDxlTaskProcdone
}

void dxl_task_off_proc(void *arg)
{
	dxl_init_all_motors();
	for (int iDx=0; iDx<gNumDynamixels; iDx++)
	{
		gDynamixels[iDx].enableTorque(DXL_OFF);
	}
	gDxlTaskProcDone = true;
}

void dxl_task_ping_proc(void *arg)
{
	dxl_init_all_motors();
	for (int iDx=0; iDx<gNumDynamixels; iDx++)
	{
		int error = gDynamixels[iDx].ping();
		if (error != DXL_SUCCESS)
			printf("Pinging 3mxl with ID=%d returned an error: %d\n", gDynamixels[iDx].getID(), error);
		else
			printf("3mxl (ID=%d) responded to PING!\n", gDynamixels[iDx].getID());
	}
	gDxlTaskProcDone = true;
}

void dxl_task_report_proc(void *arg)
{
	dxl_init_all_motors();
	for (int iDx=0; iDx<gNumDynamixels; iDx++)
	{
		printf("Report of 3mxl with ID=%d:\n", gDynamixels[iDx].getID());
		int error = gDynamixels[iDx].printReport(stdout);
		if (error != DXL_SUCCESS)
			printf("Reporting 3mxl with ID=%d returned an error: %d\n", gDynamixels[iDx].getID(), error);
		printf("\n");
	}
	gDxlTaskProcDone = true;
}

void dxl_task_communication_endurence_test(void *arg)
{
	int run = 1;
	int count = 0;
	dxl_init_all_motors();
	if(gMotorsInitialized == false)
		return;
	while(run)
	{
		for (int iDx=0; iDx<gNumDynamixels; iDx++)
		{
			printf("Test of 3mxl with ID=%d:\n", gDynamixels[iDx].getID());
			int error = gDynamixels[iDx].printReport(stdout);
			if (error != DXL_SUCCESS){
				printf("Reporting 3mxl with ID=%d returned an error: %d on count:%d\n", gDynamixels[iDx].getID(), error, count);
				run = 0;
				break;
			}
		}
		count++;
	}
	gDxlTaskProcDone = true;
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
		printf("Usage: dxl-all [real time serial device] [baud rate] [min-dynamixel-ID] [max-dynamixel-ID] [report|ping|on|off]\n");
		printf(" * report\t// shows information on all motors\n * on\t\t// enables the torque of all motors\n * off\t\t// disables the torque of all motors\n * ping\t\t// pings all motors\n");
		return -1;
	}
//	rt_print_auto_init(1); // enable rt_printf functionality

	// Set serial device name
	if (strlen(argv[1]) < MAX_DEVICE_NAME_LEN)
		strcpy(devicename, argv[1]);
	else
	{
		printf("[ERROR] Device name too long (probably not rtser0 or rtser1 ..)!\n");
		return -1;
	}

	// Open serial port
	if (!serialPort.port_open(devicename, LxSerial::RS485_SMSC))
	{
		printf("[ERROR] Failed to open serial port!\n");
		return -1;
	}

	// Set correct baud rate
	int baudrate = atoi(argv[2]);
	serialPort.set_speed_int(baudrate);
	// Set correct baud rate
//	serialPort.set_speed(LxSerial::S500000);


	// Create dynamixel task

	// Detect minimum ID
	gDxlMinID = atoi(argv[3]);

	// Detect maximum ID
	gDxlMaxID = atoi(argv[4]);

	gLogFactory().enableConsoleOutput(true);
	gLogFactory().setLevel(llCrawl);
	logInfoLn(CLog2("3mxl-all"), "logging enabled");
	// Detect task type and start the appropriate task
	if (strcmp(argv[5], "ping") == 0)
	{
		dxl_task_ping_proc(NULL);

	}
	else if ((strcmp(argv[5], "on") == 0) || (strcmp(argv[5], "off") == 0)) // if 'on' or 'off'
	{
		if (strcmp(argv[5], "on") == 0)
		{
			printf("Press Ctrl-C to terminate\n");
			// This job can be canceled using Ctrl-C
			signal(SIGTERM, catch_signal);
			signal(SIGINT, catch_signal);

			dxl_task_on_proc(NULL);

			while (!gQuit)
				pause();
		}
		// Always turn off at the end
		dxl_task_off_proc( NULL);
	}
	else if (strcmp(argv[5], "report") == 0)
	{
		dxl_task_report_proc( NULL);
	}
	else if (strcmp(argv[5], "test") ==0)
	{
		dxl_task_communication_endurence_test( NULL);
	}


	// Now wait for the main proc to finish
	while (!gDxlTaskProcDone)
		usleep((__useconds_t)1E5);	// 100ms

	// Clean up
	serialPort.port_close();
	printf("End of dxl-all.\n");
	return 0;
}
