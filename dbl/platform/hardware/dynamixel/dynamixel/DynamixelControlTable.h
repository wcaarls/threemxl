// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXELCONTROLTABLE_H_INCLUDED__
#define __DYNAMIXELCONTROLTABLE_H_INCLUDED__

// Control Table Addresses
//EEPROM AREA

#ifndef __DYNAMIXEL_CONTROL_TABLE_HEADER__ //we need this define because we share this table with the 3mxl
#define __DYNAMIXEL_CONTROL_TABLE_HEADER__
#define P_MODEL_NUMBER_L			0
#define P_MODEL_NUMBER_H			1
#define P_VERSION					2
#define P_ID						3
#endif //__DYNAMIXEL_CONTROL_TABLE_HEADER__

#define P_BAUD_RATE					4
#define P_RETURN_DELAY_TIME			5
#define P_CW_ANGLE_LIMIT_L			6
#define P_CW_ANGLE_LIMIT_H			7
#define P_CCW_ANGLE_LIMIT_L			8
#define P_CCW_ANGLE_LIMIT_H			9
#define P_SYSTEM_DATA2				10
#define P_LIMIT_TEMPERATURE			11
#define P_DOWN_LIMIT_VOLTAGE		12
#define P_UP_LIMIT_VOLTAGE			13
#define P_MAX_TORQUE_L				14
#define P_MAX_TORQUE_H				15
#define P_RETURN_LEVEL				16
#define P_ALARM_LED					17
#define P_ALARM_SHUTDOWN			18
#define P_OPERATING_MODE			19
#define P_DOWN_CALIBRATION_L		20
#define P_DOWN_CALIBRATION_H		21
#define P_UP_CALIBRATION_L			22
#define P_UP_CALIBRATION_H			23

#define P_TORQUE_ENABLE				(24)
#define P_LED						(25)
#define P_CW_COMPLIANCE_MARGIN		(26)
#define P_CCW_COMPLIANCE_MARGIN		(27)
#define P_CW_COMPLIANCE_SLOPE		(28)
#define P_CCW_COMPLIANCE_SLOPE		(29)
#define P_GOAL_POSITION_L			(30)
#define P_GOAL_POSITION_H			(31)
#define P_GOAL_SPEED_L				(32)
#define P_GOAL_SPEED_H				(33)
#define P_TORQUE_LIMIT_L			(34)
#define P_TORQUE_LIMIT_H			(35)
#define P_PRESENT_POSITION_L		(36)
#define P_PRESENT_POSITION_H		(37)
#define P_PRESENT_SPEED_L			(38)
#define P_PRESENT_SPEED_H			(39)
#define P_PRESENT_LOAD_L			(40)
#define P_PRESENT_LOAD_H			(41)
#define P_PRESENT_VOLTAGE			(42)
#define P_PRESENT_TEMPERATURE		(43)
#define P_REGISTERED_INSTRUCTION	(44)
#define P_PAUSE_TIME				(45)
#define P_MOVING					(46)
#define P_LOCK						(47)
#define P_PUNCH_L					(48)
#define P_PUNCH_H					(49)




// Dynamixel supoorted baud rates
#define DXL_BAUD_RATE_1000000		1
#define DXL_BAUD_RATE_500000		3
#define DXL_BAUD_RATE_400000		4
#define DXL_BAUD_RATE_250000		7
#define DXL_BAUD_RATE_200000		9
#define DXL_BAUD_RATE_115200		16
#define DXL_BAUD_RATE_57600			34
#define DXL_BAUD_RATE_19200			103
#define DXL_BAUD_RATE_9600			207




#endif //__DYNAMIXELCONTROLTABLE_H_INCLUDED__
