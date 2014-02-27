// Dynamixel control code - C++ file
// Copyright (c) 2010 Eelko van Breda
// based on dynamixel.cpp (Erik Schuitema & Eelko van Breda)
// Delft University of Technology
// www.dbl.tudelft.nl

#include <threemxl/platform/hardware/dynamixel/3mxl/3mxl.h>
#include <threemxl/platform/hardware/dynamixel/CDxlPacketHandler.h>

#include <math.h>
#include <sstream>
#include <algorithm>
#include <string.h>
#include <unistd.h> // we have a usleep in report which is nonrt

#include <threemxl/externals/half/half.h>

using namespace std;

//***********************************************************//
//*********************** CDynamixel ************************//
//***********************************************************//


C3mxl::C3mxl(): CDxlGeneric(), mLog("3mxl")
{
	//NOTE: Most of the initial values are set in the  setConfig() and init() methods!
//	mLog.setLevel(llCrawl);
	mID					= -1;
	mPosition			= 0;
	mSpeed 				= 0;
	mRetlevel			= 0;
	mStatus				= 0;
}

C3mxl::~C3mxl()
{
}

const char*	C3mxl::translateErrorCode(int errorCode)
{
	switch (errorCode)
	{
		case M3XL_STATUS_MOVING				: return "M3XL_STATUS_MOVING";break;
		case M3XL_STATUS_MOVE_DONE                      : return "M3XL_STATUS_MOVE_DONE";break;
		case M3XL_STATUS_INITIALIZE_BUSY                : return "M3XL_STATUS_INITIALIZE_BUSY";break;
		case M3XL_STATUS_INIT_DONE                      : return "M3XL_STATUS_INIT_DONE";break;
		case M3XL_STATUS_POS_MODE_EXECUTING             : return "M3XL_STATUS_POS_MODE_EXECUTING";break;
		case M3XL_STATUS_POS_MODE_DONE                  : return "M3XL_STATUS_POS_MODE_DONE";break;
		case M3XL_STATUS_SPEED_MODE_EXECUTING           : return "M3XL_STATUS_SPEED_MODE_EXECUTING";break;
		case M3XL_STATUS_SPEED_MODE_DONE                : return "M3XL_STATUS_SPEED_MODE_DONE";break;
		case M3XL_STATUS_TORQUE_MODE_EXECUTING          : return "M3XL_STATUS_TORQUE_MODE_EXECUTING";break;
		case M3XL_STATUS_TORQUE_MODE_DONE               : return "M3XL_STATUS_TORQUE_MODE_DONE";break;
		case M3XL_STATUS_CURRENT_MODE_EXECUTING         : return "M3XL_STATUS_CURRENT_MODE_EXECUTING";break;
		case M3XL_STATUS_CURRENT_MODE_DONE              : return "M3XL_STATUS_CURRENT_MODE_DONE";break;
		case M3XL_STATUS_SEA_MODE_EXECUTING             : return "M3XL_STATUS_SEA_MODE_EXECUTING";break;
		case M3XL_STATUS_SEA_MODE_DONE                  : return "M3XL_STATUS_SEA_MODE_DONE";break;
		case M3XL_STATUS_PWM_MODE_EXECUTING             : return "M3XL_STATUS_PWM_MODE_EXECUTING";break;
		case M3XL_STATUS_PWM_MODE_DONE                  : return "M3XL_STATUS_PWM_MODE_DONE";break;
		case M3XL_STATUS_IDLE_STATE                     : return "M3XL_STATUS_IDLE_STATE";break;
		default                                         : return CDxlCom::translateErrorCode(errorCode);
	}
}

//3MXL_OK
double C3mxl::mxlCurrentToInternalCurrent(WORD current) //convert from mA to A
{
	return (((short)current)/1000.0);
}

//3MXL_OK
WORD C3mxl::internalCurrentToMxlCurrent(double current) //convert from A to mA
{
	return clipToMaxWord((int)(current*1000));
}

//3MXL_OK
double C3mxl::mxlVoltageToInternalVoltage(WORD voltage) //convert from cV to V
{
	return (((int16_t)voltage)/100.0);
}

//3MXL_OK
WORD C3mxl::internalVoltageToMxlVoltage(double voltage) //convert from V to cV
{
	return (int)(voltage*100);
}

//3MXL_OK
double C3mxl::mxlPosToInternalPos(WORD pos) //convert from mrad to rad
{
	return (((short)pos)/1000.0);
}

//3MXL_OK
WORD C3mxl::internalPosToMxlPos(double pos) //convert from rad to mrad
{
	return clipToMaxWord((int)(pos*1000));
}

//3MXL_OK
double C3mxl::mxlSpeedToInternalSpeed(WORD speed) //convert from 10mrad/s to rad/s
{
	return  (((short)speed)/100.0);
}

//3MXL_OK
WORD C3mxl::internalSpeedToMxlSpeed(double speed) //convert from rad/s to 10mrad/s
{
	return clipToMaxWord((int)(speed*100));
}

//3MXL_OK
double C3mxl::mxlAccelerationToInternalAcceleration(WORD acceleration) //convert from 10mrad/s^2 to rad/s^2
{
	return  (((short)acceleration)/100.0);
}

//3MXL_OK
WORD C3mxl::internalAccelerationToMxlAcceleration(double acceleration) //convert from rad/s^2 to 10mrad/s^2
{
	return clipToMaxWord((int)(acceleration*100));
}

//3MXL_OK
double C3mxl::mxlLinearPosToInternalLinearPos(DWORD pos) //convert from 100um to m
{
	return (((int)pos)/10000.0);
}

//3MXL_OK
DWORD C3mxl::internalLinearPosToMxlLinearPos(double pos) //convert from m to 100um
{
	return clipToMaxDWord((int)(pos*10000));
}

//3MXL_OK
double C3mxl::mxlLinearSpeedToInternalLinearSpeed(WORD speed) //convert from mm/s to m/s
{
	return  (((short)speed)/1000.0);
}

//3MXL_OK
WORD C3mxl::internalLinearSpeedToMxlLinearSpeed(double speed) //convert from m/s to mm/s
{
	return clipToMaxWord((int)(speed*1000));
}

//3MXL_OK
double C3mxl::mxlLinearAccelerationToInternalLinearAcceleration(WORD acceleration) //convert from mm/s^2 to m
{
	return  (((short)acceleration)/1000.0);
}

//3MXL_OK
WORD C3mxl::internalLinearAccelerationToMxlLinearAcceleration(double acceleration) //convert from m/s^2 to mm/s^2
{
	return clipToMaxWord((int)(acceleration*1000));
}

//3MXL_OK
double C3mxl::mxlTorqueToInternalTorque(WORD torque) //convert from 1 mNm to Nm
{
	return  (((short)torque)/1000.0) ;
}

//3MXL_OK
WORD C3mxl::internalTorqueToMxlTorque(double torque) //convert from Nm to mNm
{
	return clipToMaxWord((int)(torque*1000));
}

//3MXL_OK
WORD C3mxl::internalFreqToMxlFreq(double frequency) //convert from Hz to cHz
{
	return clipToMaxWord((int)(frequency*100));
}

//3MXL_OK
WORD C3mxl::internalAmplitudeToMxlAmplitude(double amplitude) //convert from rad to mrad
{
	return clipToMaxWord((int)(amplitude*1000));
}

//3MXL_OK
WORD C3mxl::internalPhaseToMxlPhase(double phase) //convert from rad to mrad
{
	return clipToMaxWord((int)(phase*1000));
}

//3MXL_OK
double C3mxl::mxlPWMToInternalPWM(WORD pwm) //convert percentage to fraction
{
	return (((short)pwm)/100.0);
}

//3MXL_OK
WORD C3mxl::internalPWMToMxlPWM(double pwm) //convert fraction to percentage
{
	return clipToMaxWord((int)(pwm*100));
}

int C3mxl::clipToMaxWord(int value)
{
	if (value>INT16_MAX)
	{
		mLogWarningLn("Clipping " << value << " to " << INT16_MAX);
		value = INT16_MAX;
	}
	else if (value<INT16_MIN)
	{
		mLogWarningLn("Clipping " << value << " to " << INT16_MIN);
		value = INT16_MIN;
	}
	return value;
}

int C3mxl::clipToMaxDWord(int value)
{
	if (value>INT32_MAX)
	{
		mLogWarningLn("Clipping " << value << " to " << INT32_MAX);
		value = INT32_MAX;
	}
	else if (value<INT32_MIN)
	{
		mLogWarningLn("Clipping " << value << " to " << INT32_MIN);
		value = INT32_MIN;
	}
	return value;
}
//still needs work
void C3mxl::setConfig(CDxlConfig* config)
{
	// Just copy the configuration
	mConfig = *config;
	
        // Immediately take over new ID when configuring a new motor.
	if (!mInitialized && mConfig.mID.isSet())
	  mID = mConfig.mID;

	// Set direction convention
	if (mConfig.mClockwiseIsPositive.isSet())
		{setPositiveDirection(mConfig.mClockwiseIsPositive);}
	else
		{setPositiveDirection(1);}

	//set initial configuration if not defined in the configuration
	if (!mConfig.mReturnDelay.isSet() )					{setReturnDelayTime(INITIAL_RETURN_DELAY_TIME);}
	if (!mConfig.mAngleLowerLimit.isSet() )				{setAngleLowerLimit(0);}
	if (!mConfig.mAngleUpperLimit.isSet() )				{setAngleUpperLimit(0);}
	if (!mConfig.mTorqueLimit.isSet() )					{setTorqueLimit(INITIAL_TORQUE_LIMIT);}
	if (!mConfig.m3mxlMode.isSet())						{set3MxlMode(STOP_MODE);}
	if (!mConfig.mWatchdogMode.isSet())					{setWatchdogMode(INITIAL_WATCHDOG_MODE);}
	if (!mConfig.mWatchdogTime.isSet())					{setWatchdogTime(INITIAL_WATCHDOG_TIME);}
	if (!mConfig.mWatchdogMult.isSet())					{setWatchdogMultiplier(INITIAL_WATCHDOG_MULT);}
	if (!mConfig.mStatusReturnLevel.isSet())			{setRetlevel(INITIAL_STATUS_RETURN_LEVEL);}
	if (!mConfig.mMotorConstant.isSet())				{setMotorConstant(INITIAL_MOTOR_CONSTANT);}
	if (!mConfig.mGearboxRatioMotor.isSet())			{setGearboxRatioMotor(INITIAL_GEARBOX_RATIO);}
	if (!mConfig.mGearboxRatioJoint.isSet())			{setGearboxRatioJoint(INITIAL_GEARBOX_RATIO);}
	if (!mConfig.mEncoderCountMotor.isSet())			{setEncoderCountMotor(INITIAL_ENCODER_COUNT_MOTOR);}
	if (!mConfig.mOffsetMotor.isSet())					{setMotorOffset(INITIAL_OFFSET_MOTOR);}
//	if (!mConfig.mMaxUninitialisedMotorCurrent.isSet())	{setMaxUninitializedMotorCurrent(INITIAL_MAX_UNINITIALIZED_MOTOR_CURRENT);}
//	if (!mConfig.mMaxMotorCurrent.isSet())				{setMaxMotorCurrent(INITIAL_MAX_MOTOR_CURRENT);}
	if (!mConfig.mEncoderCountJoint.isSet())			{setEncoderCountJoint(INITIAL_ENCODER_COUNT_JOINT);}
	if (!mConfig.mOffsetJoint.isSet())					{setJointOffset(INITIAL_OFFSET_JOINT);}
	if (!mConfig.mZeroLengthSpring.isSet())				{setZeroLengthSpring(INITIAL_ZERO_SPRING_LENGTH);}
	if (!mConfig.mSpringStiffness.isSet())				{setSpringStiffness(INITIAL_SPRING_STIFFNESS);}
	if (!mConfig.mJointClockWiseIsPositive.isSet())		{setPositiveDirectionJoint(INITIAL_JOINT_DIRECTION);}

}
//3MXL_OK
void C3mxl::setSerialPort(LxSerial* serialPort)
{
	mSerialPort = serialPort;
	mLogCrawlLn("set serialport called");
}
//3MXL_OK

void C3mxl::setPositiveDirection(bool clockwiseIsPositive)
{
	setPositiveDirectionMotor(clockwiseIsPositive);
}

int C3mxl::setPositiveDirectionMotor(bool clockwiseIsPositive)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setPositiveDirectionMotor with parameter " << (int)clockwiseIsPositive);

	BYTE direction = 1;
	if (!clockwiseIsPositive)
	{
		direction = 0;
	}
	return writeData(M3XL_MOTOR_ENC_DIRECTION, 1, &direction);

}
//3MXL_OK
int C3mxl::setPositiveDirectionJoint(bool clockwiseIsPositive)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setPositiveDirectionJoint with parameter " << (int)clockwiseIsPositive);

		BYTE direction = 1;
	if (!clockwiseIsPositive)
	{
		direction = 0;
	}
	return writeData(M3XL_JOINT_ENC_DIRECTION, 1, &direction);
}

int C3mxl::setMotorOffset(double offset)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("set motorOffset with parameter " << offset);
	WORD offsetmRad = internalPosToMxlPos(offset); // offset mRad
	return writeData(M3XL_OFFSET_MOTOR_L, 2,(BYTE*)&offsetmRad , false);

}

int C3mxl::setJointOffset(double offset)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setJointOffset with parameter " << offset);

	WORD offsetmRad = internalPosToMxlPos(offset); // offset in mRad
	return writeData(M3XL_OFFSET_JOINT_L, 2,(BYTE*)&offsetmRad , false);
}

int C3mxl::setEncoderCountMotor(WORD encodercount)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setEncoderCountMotor with parameter " << (int)encodercount);

	return writeData(M3XL_ENCODER_COUNT_MOTOR_L, 2,(BYTE*)&encodercount , false); //send to 3mxl
}

int C3mxl::setEncoderCountJoint(WORD encodercount)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setEncoderCountJoint with parameter " << (int)encodercount);

	return writeData(M3XL_ENCODER_COUNT_JOINT_L, 2,(BYTE*)&encodercount , false);
}

int C3mxl::setMotorConstant(WORD motorconstant)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setMotorConstant with parameter " << (int)motorconstant);

	return writeData(M3XL_MOTOR_CONSTANT_L, 2,(BYTE*)&motorconstant , false);
}

int C3mxl::setMaxPeakMotorCurrent(double current)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setMaxPeakMotorCurrent with parameter " << current);

	WORD data = internalCurrentToMxlCurrent(current);
	return writeData(M3XL_MAX_MOTOR_PEAK_CURRENT_L, 2, (BYTE*)&data, false);
}

int C3mxl::setMaxContinuousMotorCurrent(double current)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setMaxContinuousMotorCurrent with parameter " << current);

	WORD data = internalCurrentToMxlCurrent(current);
	return writeData(M3XL_MAX_CONTINUOUS_MOTOR_CURRENT_L, 2, (BYTE*)&data, false);
}

int C3mxl::setMotorWindingTimeConstant(double time)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setMotorWindingTimeConstant with parameter " << time);

	WORD data = time*100;
	return writeData(M3XL_MOTOR_WINDING_TIME_CONSTANT_L, 2, (BYTE*)&data, false);
}

int C3mxl::setEncoderIndexLevelMotor(BYTE level)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setEncoderIndexLevelMotor with parameter " << (int)level);
	
	return writeData(M3XL_MOTOR_ENC_INDEX_LEVEL, 1, &level, false);
}

int C3mxl::setWheelDiameter(double diameter)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setWheelDiameter with parameter " << diameter);

    WORD data = internalLinearPosToMxlLinearPos(diameter);
        
    return writeData(M3XL_WHEEL_DIAMETER_L, 2, (BYTE*)&data, false);
}

int C3mxl::setGearboxRatioMotor(float gearboxratiomotor)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setGearboxRatioMotor with parameter " << (int)gearboxratiomotor);

#ifdef PHIDES
	WORD dGearboxRatio = gearboxratiomotor;
#else
	WORD dGearboxRatio = gearboxratiomotor * 10;
#endif

	return writeData(M3XL_GEARBOX_RATIO_MOTOR_L, 2,(BYTE*)&dGearboxRatio , false);
}

int C3mxl::setGearboxRatioJoint(float gearboxratiojoint)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setGearboxRatioJoint with parameter " << (int)gearboxratiojoint);

#ifdef PHIDES
	WORD dGearboxRatio = gearboxratiojoint;
#else
	WORD dGearboxRatio = gearboxratiojoint * 10;
#endif

	return writeData(M3XL_GEARBOX_RATIO_JOINT_L, 2,(BYTE*)&dGearboxRatio , false);
}

int C3mxl::set3MxlMode(BYTE mxlMode, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("set 3mxl " << mID << " to mode " << (int)mxlMode << " with shouldSyncWrite == " << shouldSyncWrite);

	int result = writeData(M3XL_CONTROL_MODE, 1,(BYTE*)&mxlMode , shouldSyncWrite);
	if (result != DXL_SUCCESS)
	    return result;
        else
            mMxlMode = mxlMode;
            
        return DXL_SUCCESS;
}

int C3mxl::get3MxlMode()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	int result = readData(M3XL_CONTROL_MODE, 1, &mMxlMode);
	if (result != DXL_SUCCESS)
		return result;

	return DXL_SUCCESS;
}

int C3mxl::getAcceleration()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getAcceleration()");

	WORD data;
	int result = readData(M3XL_DESIRED_ACCEL_L, 2, (BYTE *)&data);
	if (result != DXL_SUCCESS)
		return result;

	mAcceleration = mxlAccelerationToInternalAcceleration(data);

	return DXL_SUCCESS;
}

int C3mxl::getLinearAcceleration()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getLinearAcceleration()");

	WORD data;
	int result = readData(M3XL_DESIRED_LINEAR_ACCEL_L, 2, (BYTE *)&data);
	if (result != DXL_SUCCESS)
		return result;

	mLinearAcceleration = mxlLinearAccelerationToInternalLinearAcceleration(data);

	return DXL_SUCCESS;
}

int C3mxl::setWatchdogMode(BYTE watchdogmode)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setWatchdogMode with parameter " << (int)watchdogmode);

	return writeData(M3XL_WATCHDOG_MODE, 1,(BYTE*)&watchdogmode , false);
}

int C3mxl::setWatchdogTime(BYTE watchdogtime)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setWatchdogTime with parameter " << (int)watchdogtime);

	return writeData(M3XL_WATCHDOG_TIME_MS, 1,(BYTE*)&watchdogtime , false);
}

int C3mxl::setWatchdogMultiplier(BYTE watchdogmultiplier)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setWatchdogMultiplier with parameter " << (int)watchdogmultiplier);

	return writeData(M3XL_WATCHDOG_TIMER_MUL, 1,(BYTE*)&watchdogmultiplier , false);
}

//int C3mxl::setMaxUninitializedMotorCurrent(WORD maxcurrent)
//{
//	if (!mInitialized)
//		return DXL_NOT_INITIALIZED;
//
//	mLogCrawlLn("setMaxUninitializedMotorCurrent with parameter " << (int)maxcurrent);
//
//	return writeData(M3XL_MAX_UNINITIALIZED_MOTOR_CURRENT_L, 2,(BYTE*)&maxcurrent , false);
//}
//
//int C3mxl::setMaxMotorCurrent(WORD maxcurrent)
//{
//	if (!mInitialized)
//		return DXL_NOT_INITIALIZED;
//
//	mLogCrawlLn("setMaxMotorCurrent with parameter " << (int)maxcurrent);
//
//	return writeData(M3XL_MAX_MOTOR_CURRENT_L, 2,(BYTE*)&maxcurrent , false);
//}

//3MXL_OK
int C3mxl::setRetlevel(const int returnlevel)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setRetlevel with parameter " << (int)returnlevel);

	BYTE bRetlevel = returnlevel;
	mRetlevel = returnlevel;
	return writeData(M3XL_STATUS_RETURN_LEVEL, 1, &bRetlevel);
}

//3MXL_OK
int C3mxl::setBaudRate(const int baudRate)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setBaudRate with parameter " << (int)baudRate);

	return writeData(M3XL_BAUD_RATE_L, 3, (BYTE*)&baudRate);
}

//3MXL_OK
int C3mxl::setReturnDelayTime(const int microsecondsReturnDelay)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setReturnDelayTime with parameter " << (int)microsecondsReturnDelay);

	BYTE bReturnDelayIndex = microsecondsReturnDelay/2;
	return writeData(M3XL_RETURN_DELAY_TIME, 1, &bReturnDelayIndex);
}


//todo: convert initial torque to maxuninitialisedmotorcurrent
int C3mxl::setInitialTorqueLimit(double absMaxTorque)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setInitialTorqueLimit with parameter " << absMaxTorque << "calls setTorqueLimit for now... nice to change to set maxUninitialisedMotorCurrent");

	setTorqueLimit(absMaxTorque);
	return DXL_SUCCESS;
}

int C3mxl::setTorqueLimit(double absMaxTorque)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setTorqueLimit with parameter " << absMaxTorque);

	WORD data = internalTorqueToMxlTorque(absMaxTorque);
	return writeData(M3XL_MAX_JOINT_TORQUE_L, 2, (BYTE*)&data);
}

int C3mxl::setAngleLowerLimit(double limit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setAngleLowerLimit with parameter " << limit);

	WORD data = internalPosToMxlPos(limit);
	return writeData(M3XL_CCW_JOINT_ANGLE_LIMIT_L, 2, (BYTE*)&data);
}

int C3mxl::setAngleUpperLimit(double limit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setAngleUpperLimit with parameter " << limit);

	WORD data = internalPosToMxlPos(limit);
	return writeData(M3XL_CW_JOINT_ANGLE_LIMIT_L, 2, (BYTE*)&data);
}

//int C3mxl::getAngleLimits()
//{
//	if (!mInitialized)
//		return DXL_NOT_INITIALIZED;
//	if (mEndlessTurnMode)
//		return -DXL_ERR_INSTRUCTION;
//
//	WORD data[2];
//	int result = readData(P_CW_ANGLE_LIMIT_L, 4, (BYTE*)data);
//	if (result != DXL_SUCCESS)
//		return result;
//
//	// The angle limits are NOT saved in SI units (see member field definitions).
//	mCWAngleLimit	= data[0];
//	mCCWAngleLimit	= data[1];
//	return DXL_SUCCESS;
//}

int C3mxl::setAngleLimits(double lowerLimit, double upperLimit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setAngleLimits with parameters " << lowerLimit << " and " << upperLimit);

	if (upperLimit < lowerLimit)
		return DXL_INVALID_PARAMETER;

	// The angle limits are NOT saved in SI units (see member field definitions).
	WORD data[2];
	data[0]	= internalPosToMxlPos(upperLimit);
	data[1]	= internalPosToMxlPos(lowerLimit);
	return writeData(M3XL_CW_JOINT_ANGLE_LIMIT_L, 4, (BYTE*)data);
}

int C3mxl::setZeroLengthSpring(double parameterInRad)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setZeroLengthSpring with parameter " << parameterInRad);

	WORD data = internalPosToMxlPos(parameterInRad);
	return writeData(M3XL_ZERO_LENGTH_SPRING_L, 2, (BYTE*)&data);
}

int C3mxl::setSpringStiffness(double stiffness)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setSpringStiffness with parameter " << stiffness);

		WORD data[1];
		data[0] = 100 * stiffness; //convert to cNm/rad

	return writeData(M3XL_SPRING_STIFFNESS_L, 2, (BYTE*)data);
}

int C3mxl::setReferenceEnergy(double energy)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("setReferenceEnergy with parameter " << energy);

		WORD data[1];
		data[0] = 1000 * energy; //convert to mJ

	return writeData(M3XL_REFERENCE_ENERGY_L, 2, (BYTE*)data);
}

//int C3mxl::set
//{
//	if (!mInitialized)
//		return DXL_NOT_INITIALIZED;
//
//}

//double C3mxl::presentAngleLowerLimit()
//{
//	if (mDirection<0)
//		return dxlPosToInternalPos(mCCWAngleLimit);
//	else
//		return dxlPosToInternalPos(mCWAngleLimit);
//}
//
//double C3mxl::presentAngleUpperLimit()
//{
//	if (mDirection<0)
//		return dxlPosToInternalPos(mCWAngleLimit);
//	else
//		return dxlPosToInternalPos(mCCWAngleLimit);
//}


//3MXL_OK
int C3mxl::init(bool sendConfigToMotor)
{
//	if (mInitialized)
//		return DXL_ALREADY_INITIALIZED;
	int initResult = initPacketHandler();
	if (initResult != DXL_SUCCESS)
	{
		logDebugLn(mLog,"Error initializing packet handler!");
		return initResult;
	}

	// Set internal ID to perform initial ping() and return level readout
	if (mConfig.mID.isSet())
		{mID = mConfig.mID;}
	else
	{
		mInitialized = false;
		return DXL_NOT_INITIALIZED;
	}
	
	// Can't ask anything of the broadcast address
	if (mID == BROADCAST_ID)
	{
	    mInitialized = true;
	    return DXL_SUCCESS;
        }

	// Is this dynamixel alive?
	int pingResult = ping();
	if (pingResult != DXL_SUCCESS)
	{
		mInitialized = false;
		return pingResult;
	}
	else
	{
		mLogNoticeLn("3mxl "<< mID << " responded to ping...");
	}

	// Check return level
	BYTE retlevel;
	int result = readData(M3XL_STATUS_RETURN_LEVEL, 1, &retlevel);
	if (result == DXL_SUCCESS)
	{
		mRetlevel		= retlevel;
		mInitialized	= true;
	}
	else
	{
		if (result == DXL_PKT_RECV_TIMEOUT)		// possible that return level is 0 and dynamixel doesn't respond
		{
			mLogNoticeLn("3mxl "<< mID << " did not return status return level, assuming it is 0");
			mRetlevel		= 0;
			mInitialized	= true;
		}
		else
		{
			// on all other errors do not initialize
			mLogErrorLn("3mxl " << mID << " did not return status return level!");
			mInitialized = false;
		}
	}

	if (sendConfigToMotor)
	{	// Configure the motor according to mConfig
		result = mConfig.configureDynamixel(this);
	}
//	result += setPositiveDirectionMotor(clockwiseIsPositive);

	return result;
}

//3MXL_OK
int C3mxl::changeID(const int newID)
{
	BYTE bNewID = newID;

	mLogCrawlLn("changeID with parameter " << newID);

	int result = writeData(M3XL_ID, 1, &bNewID);

	if (result == DXL_PKT_RECV_ID_ERR)
	{
		// This means the ID actually changed! Success!
		mID = newID;
		return DXL_SUCCESS;
	}
	else
	if (result == DXL_SUCCESS)
	{
		// We should get a DXL_PKT_RECV_ID_ERR but hey.. DXL_SUCCESS is better than FAIL!
		//printf(" (changeID returned DXL_SUCCESS instead of DXL_PKT_RECV_ID_ERR) \n");
		mID = newID;
		return DXL_SUCCESS;
	}
	else
		return result;
}

//3MXL_OK
int C3mxl::getState()
{
	if (!mInitialized)
	{
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("getState called... ");

	BYTE data[10];
	memset(data, 0, 10*sizeof(BYTE));

	int result = readData(M3XL_VOLTAGE_L, 10, data);
	if (result != DXL_SUCCESS)
	{
		return result;
	}

	mVoltage                = mxlVoltageToInternalVoltage		(*(WORD*)(data+0));
	mCurrent 		= mxlCurrentToInternalCurrent		(*(WORD*)(data+2));
	mTorque			= mxlTorqueToInternalTorque		(*(WORD*)(data+4));
	mPosition		= mxlPosToInternalPos			(*(WORD*)(data+6));
	mSpeed			= mxlSpeedToInternalSpeed		(*(WORD*)(data+8));

	return DXL_SUCCESS;
}

int C3mxl::setLogInterval(BYTE interval)
{
	BYTE one = 1;

	int result = writeData(M3XL_LOG_DATA_INTERVAL, 1, &interval);
	if (result != DXL_SUCCESS)
		return result;
	result = writeData(M3XL_ENABLE_DATA_LOGGER, 1, &one);
	if (result != DXL_SUCCESS)
		return result;
		
	return DXL_SUCCESS;
}

int C3mxl::setSyncReadIndex(BYTE index)
{
  int result = writeData(M3XL_SYNC_READ_INDEX, 1, &index);
  if (result != DXL_SUCCESS)
    return result;

  return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getPos()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("getPos called... ");

	BYTE data[2];
	int result = readData(M3XL_ANGLE_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

 	mPosition = mxlPosToInternalPos(*(WORD*)(data));
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getLinearPos()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	mLogCrawlLn("getLinearPos called... ");

	DWORD data[1];
	int result = readData(M3XL_POSITION_32_1, 4, (BYTE*)data);
	if (result != DXL_SUCCESS)
		return result;

 	mLinearPosition = mxlLinearPosToInternalLinearPos(data[0]);
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getTorquePosSpeed()
{
	if (!mInitialized)
	{
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("getTorquePosSpeed called... ");
	BYTE data[6];
	memset(data, 0, 6*sizeof(BYTE));

	int result = readData(M3XL_TORQUE_L, 6, data);
	if (result != DXL_SUCCESS)
		return result;

	mTorque		= mxlTorqueToInternalTorque(*(WORD*)(data));
	mPosition	= mxlPosToInternalPos(*(WORD*)(data+2));
	mSpeed		= mxlSpeedToInternalSpeed((*(WORD*)(data+4)));

	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getPosAndSpeed()
{
	if (!mInitialized)
	{
		return DXL_NOT_INITIALIZED;
	}

	mLogCrawlLn("getPosAndSpeed called... ");

	BYTE data[4];
	memset(data, 0, 4*sizeof(BYTE));

	int result = readData(M3XL_ANGLE_L, 4, data);
	if (result != DXL_SUCCESS)
		return result;

	mPosition	= mxlPosToInternalPos(*(WORD*)(data));
	mSpeed		= mxlSpeedToInternalSpeed((*(WORD*)(data+2)));

	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getVoltage()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getVoltage called... ");
	BYTE data[2];
	int result = readData(M3XL_VOLTAGE_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

	mVoltage = mxlVoltageToInternalVoltage(*(WORD*)(data));
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getBusVoltage()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getBusVoltage called... ");
	BYTE data[2];
	int result = readData(M3XL_BUS_VOLTAGE_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

	mBusVoltage = mxlVoltageToInternalVoltage(*(WORD*)(data));
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getSensorVoltages()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getSensorVoltages called... ");
	BYTE data[12];
	int result = readData(M3XL_BUS_VOLTAGE_L, 12, data);
	if (result != DXL_SUCCESS)
		return result;

        half h;

	mBusVoltage = mxlVoltageToInternalVoltage(*(WORD*)(data));
	h.setBits(*(WORD*)(data+2)); mCurrentADCVoltage = h;
	h.setBits(*(WORD*)(data+4)); mAnalog1Voltage = h;
        h.setBits(*(WORD*)(data+6)); mAnalog2Voltage = h;
        h.setBits(*(WORD*)(data+8)); mAnalog3Voltage = h;
        h.setBits(*(WORD*)(data+10)); mAnalog4Voltage = h;
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getCurrent()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getCurrent called... ");
	BYTE data[2];
	int result = readData(M3XL_CURRENT_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

	mCurrent = mxlCurrentToInternalCurrent(*(WORD*)(data));
	return DXL_SUCCESS;
}

//3MXL_OK
int C3mxl::getTorque()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getTorque called... ");
	BYTE data[2];
	int result = readData(M3XL_TORQUE_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

	mTorque = mxlTorqueToInternalTorque(*(WORD*)(data));
	return DXL_SUCCESS;
}

int C3mxl::getStatus()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getStatus called... ");

	BYTE data[2];
	int result = readData(M3XL_STATUS, 2, data);
	if (result != DXL_SUCCESS)
		return result;
	mStatus = data[0];
	mMotorInitialized = data[1];
	return DXL_SUCCESS;
}

int	C3mxl::getLog()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("getLog called... ");

	//clear buffer
	mMxlLog.clear();

	TMxlLogEntry data[M3XL_NR_OF_SAMPLES_PER_BLOCK];

	//Read to buffer loop
	for (BYTE i = 1 ; i <= M3XL_NR_OF_BLOCKS; i++ ){
		//write block pointer to 3mxl
		writeData(M3XL_DATA_LOGGER, 1, &i);
		//read the bytes to the buffer
		int result = readData(M3XL_DATA_LOGGER, M3XL_NR_OF_BYTES_PER_BLOCK, (BYTE*)data);
		if (result != DXL_SUCCESS)
			return result;
		for( int k = 0; k < M3XL_NR_OF_SAMPLES_PER_BLOCK; k++){
		        if (data[k].time != 0 || data[k].pwm != 0 || data[k].current != 0 || data[k].voltage != 0 || data[k].desired != 0 || data[k].actual != 0)
				mMxlLog.push_back(data[k]);
		}
	}
	return DXL_SUCCESS;
}

int C3mxl::setPos(double pos, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPos(double pos, bool shouldSyncWrite) with parameters " << pos << " " << shouldSyncWrite);

	WORD data[1];
	data[0] = internalPosToMxlPos(pos);
	return writeData(M3XL_DESIRED_ANGLE_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setLinearPos(double pos, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setLinearPos(double pos, bool shouldSyncWrite) with parameters " << pos << " " << shouldSyncWrite);

	DWORD data[1];
	data[0] = internalLinearPosToMxlLinearPos(pos);
	return writeData(M3XL_DESIRED_POSITION_32_1, 4, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setPos(double pos, double absSpeed, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPos(double pos, double absSpeed, bool shouldSyncWrite) with parameters " << pos << " " << absSpeed <<" " << shouldSyncWrite);

	WORD data[2];
	data[0] = internalPosToMxlPos(pos);
	if (absSpeed < 0)	// Interpret negative speeds as maximum speed
		data[1] = 0;	// speed = 0 means MAXIMUM speed.
	else
		data[1] = abs(internalSpeedToMxlSpeed(absSpeed));	// use abs() here, because internalSpeedToDxlSpeed() may return something negative.
	return writeData(M3XL_DESIRED_ANGLE_L, 4, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setLinearPos(double pos, double absSpeed, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setLinearPos(double pos, bool shouldSyncWrite) with parameters " << pos << " " << absSpeed << " " << shouldSyncWrite);

	if (absSpeed < 0)
	  absSpeed = 0;
	  
	setLinearSpeed(absSpeed, shouldSyncWrite);

	DWORD data[1];
	data[0] = internalLinearPosToMxlLinearPos(pos);
	writeData(M3XL_DESIRED_POSITION_32_1, 4, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite) with parameters " << pos << " " << absSpeed << " " << acceleration << " " << shouldSyncWrite);
	
	setAcceleration(acceleration, shouldSyncWrite);

	WORD data[2];
	data[0] = internalPosToMxlPos(pos);
	if (absSpeed < 0)	// Interpret negative speeds as maximum speed
		data[1] = 0;	// speed = 0 means MAXIMUM speed.
	else
		data[1] = abs(internalSpeedToMxlSpeed(absSpeed));	// use abs() here, because internalSpeedToDxlSpeed() may return something negative.
	return writeData(M3XL_DESIRED_ANGLE_L, 4, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setLinearPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setLinearPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite) with parameters " << pos << " " << absSpeed << " " << acceleration << " " << shouldSyncWrite);
	
	if (absSpeed < 0) absSpeed = 0;
	  
	setLinearSpeed(absSpeed, shouldSyncWrite);
	setLinearAcceleration(acceleration, shouldSyncWrite);

	DWORD data[1];
	data[0] = internalLinearPosToMxlLinearPos(pos);
	return writeData(M3XL_DESIRED_POSITION_32_1, 4, (BYTE*)data, shouldSyncWrite);
}

//TODO: needs work, directly set correct mode on 3mxl if not in sync
int C3mxl::setSpeed(double speed, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setSpeed(double speed, bool shouldSyncWrite) with parameters " << speed <<" " << shouldSyncWrite);

	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalSpeedToMxlSpeed(speed);
	return writeData(M3XL_DESIRED_SPEED_L, 2, (BYTE*)data, shouldSyncWrite);
}

//TODO: needs work, directly set correct mode on 3mxl if not in sync
int C3mxl::setLinearSpeed(double speed, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setLinearSpeed(double speed, bool shouldSyncWrite) with parameters " << speed <<" " << shouldSyncWrite);

	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalLinearSpeedToMxlLinearSpeed(speed);
	return writeData(M3XL_DESIRED_LINEAR_SPEED_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setAcceleration(double acceleration, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setAcceleration(double acceleration, bool shouldSyncWrite) with parameters " << acceleration <<" " << shouldSyncWrite);

	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalAccelerationToMxlAcceleration(acceleration);
	return writeData(M3XL_DESIRED_ACCEL_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setLinearAcceleration(double acceleration, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setAcceleration(double acceleration, bool shouldSyncWrite) with parameters " << acceleration <<" " << shouldSyncWrite);

	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalLinearAccelerationToMxlLinearAcceleration(acceleration);
	return writeData(M3XL_DESIRED_LINEAR_ACCEL_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setTorque(double torque, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setTorque(double torque, bool shouldSyncWrite) with parameters " << torque <<" " << shouldSyncWrite);
	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalTorqueToMxlTorque(torque);
	return writeData(M3XL_DESIRED_TORQUE_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setCurrent(double current, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setCurrent(double current, bool shouldSyncWrite) with parameters " << current <<" " << shouldSyncWrite);
	//TODO: do something with mxlMode_Speed
	WORD data[1];
	data[0] = internalCurrentToMxlCurrent(current);
	return writeData(M3XL_DESIRED_CURRENT_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setPWM(double pwm, bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	mLogCrawlLn("setPWM(double pwm, bool shouldSyncWrite) with parameters " << pwm <<" " << shouldSyncWrite);
	
	WORD data[1];
	data[0] = internalPWMToMxlPWM(pwm);
	return writeData(M3XL_DESIRED_PWM_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setSineFrequency(double frequency, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setSineFrequency(double frequency, bool shouldSyncWrite) with parameters " << frequency << " " << shouldSyncWrite);

	WORD data[1];
	data[0] = internalFreqToMxlFreq(frequency);
	return writeData(M3XL_SINUSOIDAL_FREQUENCY_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setSineAmplitude(double amplitude, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setSineAmplitude(double amplitude, bool shouldSyncWrite) with parameters " << amplitude <<" " << shouldSyncWrite);

	WORD data[1];
	data[0] = internalAmplitudeToMxlAmplitude(amplitude);
	return writeData(M3XL_SINUSOIDAL_AMPLITUDE_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setSinePhase(double phase, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setSinePhase(double phase, bool shouldSyncWrite) with parameters " << phase <<" " << shouldSyncWrite);

	WORD data[1];
	data[0] = internalPhaseToMxlPhase(phase);
	return writeData(M3XL_SINUSOIDAL_PHASE_ANGLE_L, 2, (BYTE*)data, shouldSyncWrite);
}

int C3mxl::setPosSpeedTorquePPosDPos(double pos, double speed, double torque, int pPos, int dPos, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPosSpeedTorquePPosDPos(double pos, double speed, double torque, int pPos, int dPos, bool shouldSyncWrite) with parameters "
			<< pos << " " << speed << " "<< torque << " " << pPos << " " << dPos <<" " << shouldSyncWrite);
//	if (m3mxlMode == UNINITIALISED){
//		mLogWarningLn("3mxl not initialized!");
//		return DXL_NOT_INITIALIZED;
//	}
	//TODO: do something with mxlMode_PosSpeedTorquePPosDPos

	WORD data[5];
	data[0] = internalPosToMxlPos(pos);
	data[1] = internalSpeedToMxlSpeed(speed);
	data[2] = internalTorqueToMxlTorque(torque);
	data[3] = pPos;
	data[4] = dPos;
	return writeData(M3XL_DESIRED_ANGLE_L, 10, (BYTE*)data, shouldSyncWrite);
}

int		C3mxl::setPIDCurrent(double p, double d, double i, double i_limit, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPIDCurrent(double p, double d, double i, double i_limit, bool shouldSyncWrite) with parameters "
			<< p << " " << d << " "<< i << " " << i_limit << " " << shouldSyncWrite);

	WORD data[4];
	data[0] = half(p).bits();
	data[1] = half(d).bits();
	data[2] = half(i).bits();
	data[3] = half(i_limit).bits();
	
	return writeData(M3XL_P_CURRENT_L, 8, (BYTE*)data, shouldSyncWrite);
}

int		C3mxl::setPIDPosition(double p, double d, double i, double i_limit, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPIDPosition(double p, double d, double i, double i_limit, bool shouldSyncWrite) with parameters "
			<< p << " " << d << " "<< i << " " << i_limit << " " << shouldSyncWrite);

	WORD data[4];
	data[0] = half(p).bits();
	data[1] = half(d).bits();
	data[2] = half(i).bits();
	data[3] = half(i_limit).bits();
	return writeData(M3XL_P_POSITION_L, 8, (BYTE*)data, shouldSyncWrite);
}

int		C3mxl::setPIDSpeed(double p, double d, double i, double i_limit, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPIDSpeed(double p, double d, double i, double i_limit, bool shouldSyncWrite) with parameters "
			<< p << " " << d << " "<< i << " " << i_limit << " " << shouldSyncWrite);

	WORD data[4];
	data[0] = half(p).bits();
	data[1] = half(d).bits();
	data[2] = half(i).bits();
	data[3] = half(i_limit).bits();
	return writeData(M3XL_P_SPEED_L, 8, (BYTE*)data, shouldSyncWrite);
}

int		C3mxl::setPIDTorque(double p, double d, double i, double i_limit, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPIDTorque(double p, double d, double i, double i_limit, bool shouldSyncWrite) with parameters "
			<< p << " " << d << " "<< i << " " << i_limit << " " << shouldSyncWrite);

	WORD data[4];
	data[0] = half(p).bits();
	data[1] = half(d).bits();
	data[2] = half(i).bits();
	data[3] = half(i_limit).bits();
	return writeData(M3XL_P_TORQUE_L, 8, (BYTE*)data, shouldSyncWrite);
}

int		C3mxl::setPIDEnergy(double p, double d, double i, double i_limit, bool shouldSyncWrite)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}
	mLogCrawlLn("setPIDEnergy(double p, double d, double i, double i_limit, bool shouldSyncWrite) with parameters "
			<< p << " " << d << " "<< i << " " << i_limit << " " << shouldSyncWrite);

	WORD data[4];
	data[0] = half(p).bits();
	data[1] = half(d).bits();
	data[2] = half(i).bits();
	data[3] = half(i_limit).bits();
	return writeData(M3XL_P_ENERGY_L, 8, (BYTE*)data, shouldSyncWrite);
}

int	C3mxl::getPIDCurrent(double &p, double &d, double &i, double &i_limit)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}

	WORD data[4];
	int result;
	
	if ((result = readData(M3XL_P_CURRENT_L, 8, (BYTE*)data)) != DXL_SUCCESS)
		return result;

	half h;
	h.setBits(data[0]); p = (float) h;
	h.setBits(data[1]); d = (float) h;
	h.setBits(data[2]); i = (float) h;
	h.setBits(data[3]); i_limit = (float) h;
	
	return DXL_SUCCESS;
}

int	C3mxl::getPIDPosition(double &p, double &d, double &i, double &i_limit)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}

	WORD data[4];
	int result;
	
	if ((result = readData(M3XL_P_POSITION_L, 8, (BYTE*)data)) != DXL_SUCCESS)
		return result;
  
	half h;
	h.setBits(data[0]); p = (float) h;
	h.setBits(data[1]); d = (float) h;
	h.setBits(data[2]); i = (float) h;
	h.setBits(data[3]); i_limit = (float) h;
	
	return DXL_SUCCESS;
}

int	C3mxl::getPIDSpeed(double &p, double &d, double &i, double &i_limit)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}

	WORD data[4];
	int result;
	
	if ((result = readData(M3XL_P_SPEED_L, 8, (BYTE*)data)) != DXL_SUCCESS)
		return result;
  
	half h;
	h.setBits(data[0]); p = (float) h;
	h.setBits(data[1]); d = (float) h;
	h.setBits(data[2]); i = (float) h;
	h.setBits(data[3]); i_limit = (float) h;
	
	return DXL_SUCCESS;
}

int	C3mxl::getPIDTorque(double &p, double &d, double &i, double &i_limit)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}

	WORD data[4];
	int result;
	
	if ((result = readData(M3XL_P_TORQUE_L, 8, (BYTE*)data)) != DXL_SUCCESS)
		return result;
  
	half h;
	h.setBits(data[0]); p = (float) h;
	h.setBits(data[1]); d = (float) h;
	h.setBits(data[2]); i = (float) h;
	h.setBits(data[3]); i_limit = (float) h;
	
	return DXL_SUCCESS;
}

int	C3mxl::getPIDEnergy(double &p, double &d, double &i, double &i_limit)
{
	if (!mInitialized){
		return DXL_NOT_INITIALIZED;
	}

	WORD data[4];
	int result;
	
	if ((result = readData(M3XL_P_ENERGY_L, 8, (BYTE*)data)) != DXL_SUCCESS)
		return result;
  
	half h;
	h.setBits(data[0]); p = (float) h;
	h.setBits(data[1]); d = (float) h;
	h.setBits(data[2]); i = (float) h;
	h.setBits(data[3]); i_limit = (float) h;
	
	return DXL_SUCCESS;
}

//string readToStr(BYTE startingAddress, BYTE dataLength)
//{
//	BYTE data[dataLength];
//	readData(startingAddress,dataLength,data);
//	string answer = data;
//	return answer;
//}
int C3mxl::printReport(FILE* fOut)
{
	int waitingtime = 0;
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data[LAST_MESSAGE_ADDRESS];
	memset(data, 0, LAST_MESSAGE_ADDRESS*sizeof(BYTE));
	//	int result = readData(MODEL_NUMBER_L, M3XL_MAX_TABLE_SIZE, data);	// Read 50 bytes to read the complete control table
	mLogCrawlLn("waiting time set to "<< waitingtime);
	for(int i=0; i<LAST_MESSAGE_ADDRESS ; i++)
	{
		mLogCrawlLn("checking adress "<<i);
		int result = readData(i, 1, &data[i]);	// Read each byte
		if (result != DXL_SUCCESS)
		{
			return result;
		}
		//usleep(10000);
		usleep(waitingtime);
		//	s  m  u  n
		//	rtdm_task_sleep((nanosecs_rel_t)0010000000);
	}

	return DXL_SUCCESS;
}

int C3mxl::interpretControlData(BYTE address, BYTE length, BYTE *data)
{
  // should be like:
  // if (address >= M3XL_VOLTAGE_L && address+length >= M3XL_VOLTAGE_L+2)
  //   mVoltage    = mxlVoltageToInternalVoltage   (*(WORD*)(data+xxx));

  if (address == M3XL_VOLTAGE_L && length == 10)
  {
    mVoltage    = mxlVoltageToInternalVoltage   (*(WORD*)(data+0));
    mCurrent    = mxlCurrentToInternalCurrent   (*(WORD*)(data+2));
    mTorque     = mxlTorqueToInternalTorque   (*(WORD*)(data+4));
    mPosition   = mxlPosToInternalPos     (*(WORD*)(data+6));
    mSpeed      = mxlSpeedToInternalSpeed   (*(WORD*)(data+8));
  }
}

