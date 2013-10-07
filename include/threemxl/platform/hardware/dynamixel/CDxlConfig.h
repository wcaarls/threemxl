// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXELCONFIG_H_INCLUDED__
#define __DYNAMIXELCONFIG_H_INCLUDED__

#include "CDxlGeneric.h"
#include "dynamixel/DynamixelSpecs.h"
#include "CDxlGroup.h"


#include <threemxl/platform/io/configuration/Configuration.h>
#include <threemxl/platform/io/configuration/OptionVars.h>

#define DXLCONFIG_NUM_CALIBPOINTS	301

// Conversion factors between Dynamixel and SI units
#define DXL_NUM_POSITIONS				DXL_RX28_NUM_POSITIONS
#define DXL_MAX_POSITION				DXL_RX28_MAX_POSITION
#define DXL_SPEED_TO_RAD_S				DXL_RX28_SPEED_TO_RAD_S
#define DXL_STEPS_TO_RAD				DXL_RX28_STEPS_TO_RAD
#define DXL_TORQUE_TO_RATIO				DXL_RX28_TORQUE_TO_RATIO

// Temperature needs no conversion
#define DXL_MAX_RAD_S_SPEED				(DXL_MAX_POSITION*DXL_SPEED_TO_RAD_S)
#define DXL_MAX_RAD_ANGLE				(DXL_MAX_POSITION*DXL_STEPS_TO_RAD)

#define MOTOR_CONSTANT_MULTIPLIER 10000

enum EDxlCalibType
{
	dxlCtNone,
	dxlCtAuto,	// Calib data obtained with the automatic calibration device
	dxlCtManual	// Calib data obtained with the manual calibration device ('wijzerplaat')
};

// Forward declaration
class CDxlGeneric;

/**
 * CDxlConfig is a class that describes the configuration of a dynamixel/3mxl
 */
class CDxlConfig
{
	public:
		// All in SI units where possible
		COptionInt		mID;
		std::string 	mDxlTypeStr;
		COptionInt		mReturnDelay;	// Return delay time in microseconds
		COptionDouble	mAngleLowerLimit; //motorAngleLowerlimit for 3mxl
		COptionDouble	mAngleUpperLimit; //motorAngleUpperlimit for 3mxl
		COptionBool		mClockwiseIsPositive;// Motor direction for 3mxl

		//<ROBOTIS dynamixel specific>
		COptionDouble	mVoltageLowerLimit;
		COptionDouble	mVoltageUpperLimit;
		COptionChar		mAlarmLED;
		COptionChar		mAlarmShutdown;
		COptionByte		mComplianceMargin;
		COptionByte		mComplianceSlope;
		COptionInt		mTempLimit;		// In degrees celcius
		COptionBool		mLED;
		COptionDouble	mTorqueLimit;
		COptionDouble	mNullAngle;				// Gives the natural 0-angle of the motor AFTER the sign convention, counted from the real 0-angle of the dynamixel (counted from -150 degrees from the mid point, so you will)
		COptionWord		mPunch;

		// Either calibdate or a direct angleLUT can be provided in the configuration.
		// When such data is available, memory will be created for it.
		EDxlCalibType	mCalibType;
		double	mCalibData[DXLCONFIG_NUM_CALIBPOINTS];	// No. of items: DXLCONFIG_NUM_CALIBPOINTS. Gives the dynamixel position readout for every degree in the range 0 to 300 degrees (use the home-made calibration device to get this data!)
		double	mAngleLUT[DXL_NUM_POSITIONS];		// No. of items: DXL_NUM_POSITIONS. Lookup table for angles. Generate this data with the automatic calibration device
		//</ROBOTIS dynamixel specific>
		//<3mxl specific>
		COptionByte		m3mxlMode;
		COptionByte		mWatchdogMode;
		COptionByte		mWatchdogTime;
		COptionByte		mWatchdogMult;
		COptionByte		mStatusReturnLevel;
		COptionDouble	mMotorConstant;
		COptionDouble	mGearboxRatioMotor;
		COptionDouble	mGearboxRatioJoint;
		COptionWord		mEncoderCountMotor;
		COptionDouble	mOffsetMotor;					//zero position of the motor in encoder counts
		COptionDouble	mMaxUninitialisedMotorCurrent;
		COptionDouble	mMaxMotorCurrent;
		COptionWord		mEncoderCountJoint;
		COptionDouble	mOffsetJoint;					//zero position of the joint in encoder counts
//		COptionDouble	mMinJointAngle;
//		COptionDouble	mMaxJointAngle;
		COptionDouble	mZeroLengthSpring;
		COptionDouble	mSpringStiffness;
		COptionBool		mJointClockWiseIsPositive;
		COptionDouble	mAcceleration;
		
		COptionDouble	mMaxPeakMotorCurrent;
		COptionDouble	mMaxContinuousMotorCurrent;
		COptionDouble	mMotorWindingTimeConstant;
		COptionByte	mEncoderIndexLevelMotor;
		COptionDouble	mWheelDiameter;
		
		COptionDouble   mPCurrent, mICurrent, mDCurrent, mILCurrent;
		COptionDouble   mPPosition, mIPosition, mDPosition, mILPosition;
		COptionDouble   mPSpeed, mISpeed, mDSpeed, mILSpeed;
		COptionDouble   mPEnergy, mIEnergy, mDEnergy, mILEnergy;
		COptionDouble   mPTorque, mITorque, mDTorque, mILTorque;

		//</3mxl specific>
		CDxlConfig();
		~CDxlConfig();

		CDxlConfig*	setID(const int ID);					// useful in passing this config pointer right after changing the ID.
		bool		readConfig(const CConfigSection &configNode);
		int			configureDynamixel(CDxlGeneric* dxl);	// call configureDynamixel() AFTER calling CDynamixel::init()!
};

/**
 * CDxlGroupConfig is a group of dynamixel configs that can be handeled as one.
 * This way you can read a group configuration easily from xml and create a
 * whole group in a single command.
 */
class CDxlGroupConfig
{
	protected:
		CDxlConfig	mDxlConfigs[MAX_NUM_DYNAMIXELS];
		int			mNumDynamixels;
	public:
		CDxlGroupConfig()								{mNumDynamixels=0;}
		int			getNumDynamixels()					{return mNumDynamixels;}
		CDxlConfig*	getDynamixelConfig(const int index)	{return &mDxlConfigs[index];}

		// Automatic config readout from XML file
		bool		readFromXML(const std::string &filename);

		// If you want, you can fill the config by hand (for debugging)
		/*
		int			addDynamixel(const int ID)
		{
			mDxlConfigs[mNumDynamixels].mID = ID;
			return mNumDynamixels++;
		}
		*/
		int			addDynamixel(CDxlConfig* config) //< store dynamixel config in group config object
		{
			mDxlConfigs[mNumDynamixels] = *config;
			return mNumDynamixels++;
		}
};


#endif //__DYNAMIXELCONFIG_H_INCLUDED__
