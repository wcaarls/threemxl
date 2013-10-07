// Dynamixel control code - C++ file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>
#include <threemxl/platform/hardware/dynamixel/CDxlPacketHandler.h>

#include <math.h>
#include <sstream>
#include <algorithm>
#include <string.h>

using namespace std;

//***********************************************************//
//*********************** CDynamixel ************************//
//***********************************************************//


CDynamixel::CDynamixel(): CDxlGeneric(), mLog("Dynamixel")
{
	mLog.setLevel(llCrawl);
	//NOTE: Most of the initial values are set in the  setConfig() and init() methods!
	mID					= -1;
	mPosition			= 0;
	mSpeed 				= 0;
	mLoad 				= 0;
	mVoltage			= 0;
	mTemperature		= 45;	// Assume 45 degrees if not measured
	mRetlevel			= 0;
	mNullAngle			= DXL_MAX_RAD_ANGLE/2.0;	// Put the null angle in the middle of its range by default

	mDirection			= 1.0;
	mCWAngleLimit		= 0;
	mCCWAngleLimit		= DXL_MAX_POSITION;
	mEndlessTurnMode	= false;
	// Fill the angle LookUp Table with initial data
	for (int i=0; i<DXL_NUM_POSITIONS; i++)
		mAngleLUT[i] = (double)i*DXL_STEPS_TO_RAD;
}

CDynamixel::~CDynamixel()
{
}

void CDynamixel::setConfig(CDxlConfig* config)
{
	// Just copy the configuration
	mConfig = *config;

	// Set direction convention
	if (mConfig.mClockwiseIsPositive.isSet())
		{setPositiveDirection(mConfig.mClockwiseIsPositive);}
	else
		{setPositiveDirection(false);}
	//printf("Dynamixel with ID %d has direction %.1f\n", mConfig.mID, mDirection);

	// Set null angle
	if (mConfig.mNullAngle.isSet())
		{setNullAngle(mConfig.mNullAngle);}

	//set initial configuration if not defined in the configuration
	if(!mConfig.mReturnDelay.isSet() ){setReturnDelayTime(INITIAL_RETURN_DELAY_TIME);}
	if(!mConfig.mAngleLowerLimit.isSet() ){setAngleLowerLimit(-DXL_MAX_RAD_ANGLE/2.0);}
	if(!mConfig.mAngleUpperLimit.isSet() ){setAngleUpperLimit(DXL_MAX_RAD_ANGLE/2.0);}
	if(!mConfig.mTempLimit.isSet() ){setTemperatureLimit(INITIAL_TEMPERATURE_LIMIT);}
	if(!mConfig.mLED.isSet() ){enableLED(false);}
	if(!mConfig.mTorqueLimit.isSet() ){setTorqueLimit(INITIAL_TORQUE_LIMIT);}
	if(!mConfig.mAlarmLED.isSet() ){setAlarmLEDMask(DXL_ERR_OVERHEATING | DXL_ERR_OVERLOAD);}
	if(!mConfig.mAlarmShutdown.isSet() ){setAlarmShutdownMask(DXL_ERR_OVERHEATING);}
	if(!mConfig.mPunch.isSet() ){setPunch(INITIAL_PUNCH);}

	// the following twin configuration parameters are set if one or both are missing in the config
	// in this way we always have a base value even if you have only one parameter in the config.
	if(!(mConfig.mVoltageLowerLimit.isSet() && mConfig.mVoltageUpperLimit.isSet()) )
		{setVoltageLimits(INITIAL_VOLTAGE_LOWER_LIMIT, INITIAL_VOLTAGE_UPPER_LIMIT);}
	if(!(mConfig.mComplianceMargin.isSet() && mConfig.mComplianceSlope.isSet()) )
		{setCompliance(INITIAL_COMPLIENCE_MARGIN,INITIAL_COMPLIENCE_SLOPE);}

	// Process calibration data
	if (config->mCalibType == dxlCtAuto)
	{
		// Just copy the LUT
		memcpy(mAngleLUT, config->mAngleLUT, DXL_NUM_POSITIONS*sizeof(mAngleLUT[0]));
	}
	else
	if (config->mCalibType == dxlCtManual)
	{
		int calibIndex	= 1;	// Start at 1 because we interpolate between calibIndex and calibIndex-1
		for (int lutIndex=0; lutIndex<DXL_NUM_POSITIONS; lutIndex++)
		{
			// Search for the first calibIndex point for which the calib data exceeds the lut index
			while ((config->mCalibData[calibIndex] <= (double)lutIndex) && (calibIndex < DXLCONFIG_NUM_CALIBPOINTS-1))
				calibIndex++;
			// Interpolate between calibIndex and calibIndex-1
			double interpolfact = ((double)lutIndex - config->mCalibData[calibIndex-1])/(config->mCalibData[calibIndex] - config->mCalibData[calibIndex-1]);
				// calibIndex represents a value in degrees. Therefore, the number of degrees between calibIndex and (calibIndex-1) is just 1 :)
			mAngleLUT[lutIndex] = ((double)(calibIndex-1) + interpolfact/**1.0*/)*M_PI/180.0;
			// DEBUG output
			//printf("DXL ID %d; Angle LUT [%d] = %.3f deg\n", mConfig.mID, lutIndex, mAngleLUT[lutIndex]*180.0/M_PI);
		}
	}
}

void CDynamixel::setSerialPort(LxSerial* serialPort)
{
	mSerialPort = serialPort;
}

void CDynamixel::setPositiveDirection(bool clockwiseIsPositive)
{
	if (clockwiseIsPositive)
		mDirection = -1.0;
	else
		mDirection = 1.0;
}

void CDynamixel::setNullAngle(double nullAngle)
{
	mNullAngle = nullAngle;
}

double CDynamixel::dxlPosToInternalPos(WORD pos)
{
	int lutPos = clip(pos, 0, DXL_MAX_POSITION);
	if (mDirection > 0)
		return mAngleLUT[lutPos] - mNullAngle;
	else
		return DXL_MAX_RAD_ANGLE - mAngleLUT[lutPos] - mNullAngle;
}

int CDynamixel::internalPosToDxlPos(double pos)
{
	// First, take care of sign (mDirection) and null-angle conventions
	double transpos;
	if (mDirection > 0)
		transpos = pos + mNullAngle;
	else
		transpos = DXL_MAX_RAD_ANGLE - (pos + mNullAngle);

	// The plan: find the LUT index for which mAngleLUT has the closest match to 'pos'
	// Make a first estimate and start searching from there on
	int lutIndex = clip(round(transpos/DXL_STEPS_TO_RAD), 0, DXL_MAX_POSITION);

	if (mAngleLUT[lutIndex] > transpos)
	{
		// we went too far -> go back
		while (mAngleLUT[lutIndex] > transpos)
		{
			if (lutIndex > 0)
				lutIndex--;
			else
				break;
		}
		// Now lutIndex >= 0 (guaranteed) and mAngleLUT[lutIndex] <= pos (under normal circumstances)
		if ( fabs(transpos - mAngleLUT[lutIndex]) < fabs(mAngleLUT[lutIndex+1] - transpos) )
			return lutIndex;
		else
			return lutIndex+1;
	}
	else // apparently, mAngleLUT[lutIndex] <= pos
	{
		// we are not there yet -> go forth
		while (mAngleLUT[lutIndex] <= transpos)
		{
			if (lutIndex < DXL_NUM_POSITIONS-1)
				lutIndex++;
			else
				break;
		}
		// Now lutIndex <= DXL_NUM_POSITIONS-1 (guaranteed) and mAngleLUT[lutIndex] > pos (under normal circumstances)
		if ( fabs(mAngleLUT[lutIndex] - transpos) < fabs(transpos - mAngleLUT[lutIndex-1]) )
			return lutIndex;
		else
			return lutIndex-1;
	}
}

double CDynamixel::dxlSpeedToInternalSpeed(WORD speed)
{
	if (speed & 0x400)	// Handle the sign bit
		// Negative speed
		return -mDirection*DXL_SPEED_TO_RAD_S*(double)(speed & 0x3FF);
	else
		// Positive speed
		return  mDirection*DXL_SPEED_TO_RAD_S*(double)(speed & 0x3FF);
}

int CDynamixel::internalSpeedToDxlSpeed(double speed)
{
	return round(mDirection*speed/DXL_SPEED_TO_RAD_S);
}

double CDynamixel::dxlTorqueToInternalTorque(WORD torque)
{
	if (torque & 0x400)	// Handle the sign bit
		// Negative load or torque
		return -mDirection*DXL_TORQUE_TO_RATIO*(double)(torque & 0x3FF);	// 10 bits accuracy
	else
		// Positive load or torque
		return  mDirection*DXL_TORQUE_TO_RATIO*(double)(torque & 0x3FF);	// 10 bits accuracy
}

WORD CDynamixel::internalTorqueToDxlTorque(double torqueRatio)
{
	int rawTorque = round(mDirection*torqueRatio/DXL_TORQUE_TO_RATIO);
	// Clip to 1 here, not to 0. Because when switching to position control after torque control,
	// 'torque' set to 0, which is actually the goal speed register, means maximum speed instead of zero speed.
	if (rawTorque < 0)
		rawTorque = clip(-rawTorque, 1, 1023) | 0x400;	// direction bit; negative torques will decrease position
	else
		rawTorque = clip(rawTorque, 1, 1023);

	return (WORD)rawTorque;
}

double CDynamixel::presentAngleLowerLimit()
{
	if (mDirection<0)
		return dxlPosToInternalPos(mCCWAngleLimit);
	else
		return dxlPosToInternalPos(mCWAngleLimit);
}

double CDynamixel::presentAngleUpperLimit()
{
	if (mDirection<0)
		return dxlPosToInternalPos(mCWAngleLimit);
	else
		return dxlPosToInternalPos(mCCWAngleLimit);
}

int CDynamixel::init(bool sendConfigToMotor)
{
	logCrawlLn(mLog,"INIT STARTED");
	if (mInitialized)
		return DXL_ALREADY_INITIALIZED;
		
	int initResult = initPacketHandler();
	if (initResult != DXL_SUCCESS)
	{
		logDebugLn(mLog,"Error initializing packet handler!");
		return initResult;
	}

	// Set internal ID to perform initial ping() and return level readout
	if (mConfig.mID.isSet())
	{
		mID = mConfig.mID;
	}
	else
	{
		logErrorLn(mLog,"No ID set for this dynamixel");
		mInitialized = false;
		return DXL_NOT_INITIALIZED;
	}

	// Is this dynamixel alive?
	int pingResult = ping();
	if (pingResult != DXL_SUCCESS)
	{
		mInitialized = false;
		logErrorLn(mLog,"dynamixel with ID " << mID << " failed ping test");
		return pingResult;
	}

	// Check return level
	BYTE retlevel;
	int result = readData(P_RETURN_LEVEL, 1, &retlevel);
	if (result == DXL_SUCCESS)
	{
		mRetlevel		= retlevel;
		mInitialized	= true;
	}
	else
	{
		if (result == DXL_PKT_RECV_TIMEOUT)		// possible that return level is 0 and dynamixel doesn't respond
		{
			mRetlevel		= 0;
			mInitialized	= true;
		}
		else
			// on all other errors do not initialize
			mInitialized = false;
	}

	if (sendConfigToMotor)
		// Configure the motor according to mConfig
		mConfig.configureDynamixel(this);

	return result;
}

int CDynamixel::changeID(const int newID)
{
	BYTE bNewID = newID;
	int result = writeData(P_ID, 1, &bNewID);
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

int CDynamixel::enableLED(int state)
{
	int result=DXL_SUCCESS;

	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE ledStatus;
	switch(state)
	{
		case DXL_ON:
			ledStatus = 1;
			result = writeData(P_LED, 1, &ledStatus);
			break;
		case DXL_OFF:
			ledStatus = 0;
			result = writeData(P_LED, 1, &ledStatus);
			break;
		case DXL_TOGGLE:
			readData(P_LED, 1, &ledStatus);
			ledStatus = !ledStatus;
			result = writeData(P_LED, 1, &ledStatus);
			break;
	}
	return result;
}

int CDynamixel::getState()
{
	if (!mInitialized)
	{
		return DXL_NOT_INITIALIZED;
	}
	BYTE data[8];
	memset(data, 0, 8*sizeof(BYTE));

	int result = readData(P_PRESENT_POSITION_L, 8, data);
	if (result != DXL_SUCCESS)
	{
		return result;
	}

	mPosition		= dxlPosToInternalPos(*(WORD*)(data));
	mSpeed			= dxlSpeedToInternalSpeed((*(WORD*)(data+2)));
	mLoad			= dxlTorqueToInternalTorque((*(WORD*)(data+4)));	// 10 bits accuracy + sign bit
	mVoltage		= DXL_VOLTAGE_TO_VOLT*data[6];
	mTemperature	= data[7];	// No conversion needed

	return DXL_SUCCESS;
}

int CDynamixel::getPos()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data[2];
	int result = readData(P_PRESENT_POSITION_L, 2, data);
	if (result != DXL_SUCCESS)
		return result;

	mPosition = dxlPosToInternalPos(*(WORD*)(data));
	return DXL_SUCCESS;
}

int CDynamixel::getPosAndSpeed()
{
	if (!mInitialized)
	{
		return DXL_NOT_INITIALIZED;
	}
	BYTE data[4];
	memset(data, 0, 4*sizeof(BYTE));

	int result = readData(P_PRESENT_POSITION_L, 4, data);
	if (result != DXL_SUCCESS)
		return result;

	mPosition	= dxlPosToInternalPos(*(WORD*)(data));
	mSpeed		= dxlSpeedToInternalSpeed((*(WORD*)(data+2)));

	return DXL_SUCCESS;
}

int CDynamixel::getTemp()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data;
	int result = readData(P_PRESENT_TEMPERATURE, 1, &data);
	if (result != DXL_SUCCESS)
		return result;

	mTemperature = data;	// No conversion needed
	return DXL_SUCCESS;
}

int CDynamixel::setPos(double pos, double absSpeed,bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data[2];
	data[0] = clip(internalPosToDxlPos(pos), 0, DXL_MAX_POSITION);
	if (absSpeed < 0)	// Interpret negative speeds as maximum speed
		data[1] = 0;	// speed = 0 means MAXIMUM speed.
	else
		data[1] = clip(abs(internalSpeedToDxlSpeed(absSpeed)), 1, 1023);	// use abs() here, because internalSpeedToDxlSpeed() may return something negative.
	return writeData(P_GOAL_POSITION_L, 4, (BYTE*)data, shouldSyncWrite);
}

int CDynamixel::setSpeed(double speed,bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	int dxlSpeed = internalSpeedToDxlSpeed(speed);
	WORD data[2];
	if (dxlSpeed > 0)
	{
		data[0] = mCCWAngleLimit;
		data[1] = clip(dxlSpeed, 1, 1023);
	}
	else
	{
		data[0] = mCWAngleLimit;
		data[1] = clip(-dxlSpeed, 1, 1023);
	}

	return writeData(P_GOAL_POSITION_L, 4, (BYTE*)data, shouldSyncWrite);
}

int CDynamixel::setRetlevel(const int returnlevel)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE bRetlevel = returnlevel;
	writeData(P_RETURN_LEVEL, 1, &bRetlevel);
	mRetlevel = returnlevel;
	return DXL_SUCCESS;
}

int CDynamixel::setBaudRateIndex(const BYTE baudRateIndex)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE bBaudRateIndex = baudRateIndex;
	return writeData(P_BAUD_RATE, 1, &bBaudRateIndex);
}

int CDynamixel::setBaudRate(const int baudRate)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE bBaudRateIndex = (int)2E6/baudRate - 1;
	return writeData(P_BAUD_RATE, 1, &bBaudRateIndex);
}

int CDynamixel::setReturnDelayTime(const int microsecondsReturnDelay)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE bReturnDelayIndex = microsecondsReturnDelay/2;
	return writeData(P_RETURN_DELAY_TIME, 1, &bReturnDelayIndex);
}

int CDynamixel::enableTorque(int state)
{
	int result = DXL_SUCCESS;
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE torqueStatus;
	switch(state)
	{
		case DXL_ON:
			torqueStatus = 1;
			result = writeData(P_TORQUE_ENABLE, 1, &torqueStatus);
			break;
		case DXL_OFF:
			torqueStatus = 0;
			result = writeData(P_TORQUE_ENABLE, 1, &torqueStatus);
			break;
		case DXL_TOGGLE:
			readData(P_TORQUE_ENABLE, 1, &torqueStatus);
			torqueStatus = !torqueStatus;
			result = writeData(P_TORQUE_ENABLE, 1, &torqueStatus);
			break;
	}
	return result;
}

int CDynamixel::setInitialTorqueLimit(double absMaxTorque)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data = clip(round(absMaxTorque/DXL_TORQUE_TO_RATIO), 1, 1023);	// Maximum torque of 0 would mean free run state!
	return writeData(P_MAX_TORQUE_L, 2, (BYTE*)&data);
}

int CDynamixel::setTorqueLimit(double absMaxTorque)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data = clip(round(absMaxTorque/DXL_TORQUE_TO_RATIO), 1, 1023);	// Maximum torque of 0 would mean free run state!
	return writeData(P_TORQUE_LIMIT_L, 2, (BYTE*)&data);
}

int CDynamixel::setAngleLowerLimit(double limit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data = clip(internalPosToDxlPos(limit), 0, DXL_MAX_POSITION);
	BYTE dxlAddress;
	if (mDirection<0)
		dxlAddress = P_CCW_ANGLE_LIMIT_L;
	else
		dxlAddress = P_CW_ANGLE_LIMIT_L;

	return writeData(dxlAddress, 2, (BYTE*)&data);
}

int CDynamixel::setAngleUpperLimit(double limit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data = clip(internalPosToDxlPos(limit), 0, DXL_MAX_POSITION);
	BYTE dxlAddress;
	if (mDirection<0)
		dxlAddress = P_CW_ANGLE_LIMIT_L;
	else
		dxlAddress = P_CCW_ANGLE_LIMIT_L;

	return writeData(dxlAddress, 2, (BYTE*)&data);
}

int CDynamixel::setTemperatureLimit(const int maxTemp)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data = maxTemp;
	return writeData(P_LIMIT_TEMPERATURE, 1, &data);
}

int CDynamixel::setVoltageLimits(double minVoltage, double maxVoltage)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data[2];
	data[0] = round(minVoltage/DXL_VOLTAGE_TO_VOLT);
	data[1] = round(maxVoltage/DXL_VOLTAGE_TO_VOLT);
	return writeData(P_DOWN_LIMIT_VOLTAGE, 2, data);
}

int CDynamixel::getAngleLimits()
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	if (mEndlessTurnMode)
		return -DXL_ERR_INSTRUCTION;

	WORD data[2];
	int result = readData(P_CW_ANGLE_LIMIT_L, 4, (BYTE*)data);
	if (result != DXL_SUCCESS)
		return result;

	// The angle limits are NOT saved in SI units (see member field definitions).
	mCWAngleLimit	= data[0];
	mCCWAngleLimit	= data[1];
	return DXL_SUCCESS;
}

int CDynamixel::setAngleLimits(double lowerLimit, double upperLimit)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	if (upperLimit < lowerLimit)
		return DXL_INVALID_PARAMETER;

	// The angle limits are NOT saved in SI units (see member field definitions).
	WORD data[2];
	if (mDirection<0)
	{
		data[0]	= mCWAngleLimit		= clip(internalPosToDxlPos(upperLimit), 0, DXL_MAX_POSITION);
		data[1]	= mCCWAngleLimit	= clip(internalPosToDxlPos(lowerLimit), 0, DXL_MAX_POSITION);
	}
	else
	{
		data[0]	= mCWAngleLimit		= clip(internalPosToDxlPos(lowerLimit), 0, DXL_MAX_POSITION);
		data[1]	= mCCWAngleLimit	= clip(internalPosToDxlPos(upperLimit), 0, DXL_MAX_POSITION);
	}
	return writeData(P_CW_ANGLE_LIMIT_L, 4, (BYTE*)data);
}

int CDynamixel::setEndlessTurnMode(bool enabled,bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data[2];
	int result = DXL_SUCCESS;
	if (enabled)
	{
		if (result != DXL_SUCCESS)
			return result;
		// The angle limits are set to 0 and the internal mCWAngleLimit and mCCWAngleLimit are not changed
		// in order to be able to set the mode to false again
		data[0]	= 0;
		data[1]	= 0;
		result = writeData(P_CW_ANGLE_LIMIT_L, 4, (BYTE*)data, shouldSyncWrite);
		if (result == DXL_SUCCESS)
			mEndlessTurnMode = true;
	}
	else
	{
		data[0]	= mCWAngleLimit;
		data[1]	= mCCWAngleLimit;
		result = writeData(P_CW_ANGLE_LIMIT_L, 4, (BYTE*)data, shouldSyncWrite);
		if (result == DXL_SUCCESS)
			mEndlessTurnMode = false;
	}
	return result;
}

int CDynamixel::setEndlessTurnTorque(double torqueRatio,bool shouldSyncWrite)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;
	/*
	if (!mEndlessTurnMode)
		return -DXL_ERR_INSTRUCTION;
	 *
	 * Commented out this safety check because sometimes,
	 * you want to set the torque to zero first before changing to endless turn mode.
	 */

	WORD data = internalTorqueToDxlTorque(torqueRatio);
	return writeData(P_GOAL_SPEED_L, 2, (BYTE*)&data, shouldSyncWrite);	// The endless turn torque must be written to the GOAL_SPEED field!
}

int CDynamixel::setOperatingMode(const BYTE mode)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data = mode;
	return writeData(P_OPERATING_MODE, 1, &data);
}

int CDynamixel::setAlarmLEDMask(const BYTE mask)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data = mask;
	return writeData(P_ALARM_LED, 1, &data);
}

int CDynamixel::setAlarmShutdownMask(const BYTE mask)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data = mask;
	return writeData(P_ALARM_SHUTDOWN, 1, &data);
}

int CDynamixel::setCompliance(BYTE complianceMargin, BYTE complianceSlope)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data[4];
	data[0] = complianceMargin;
	data[1] = complianceMargin;
	data[2] = complianceSlope;
	data[3] = complianceSlope;
	return writeData(P_CW_COMPLIANCE_MARGIN, 4, data);
}

int CDynamixel::setPunch(WORD punch)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	WORD data = punch;
	return writeData(P_PUNCH_L, 2, (BYTE*)&data);
}

int CDynamixel::printReport(FILE* fOut)
{
	if (!mInitialized)
		return DXL_NOT_INITIALIZED;

	BYTE data[50];
	memset(data, 0, 50*sizeof(BYTE));
//	int result = readData(P_MODEL_NUMBER_L, 50, data);	// Read 50 bytes to read the complete control table
//	if (result != DXL_SUCCESS)
//	{
//		return result;
//	}
	for(int i=0; i<50 ; i++)
	{
		mLogInfoLn("checking adress "<<i);
		int result = readData(i, 1, &data[i]);	// Read each byte
		if (result != DXL_SUCCESS)
		{
			return result;
		}
	}
	// Model, version, return delay, return level, voltage, temperature
	fprintf(fOut, "Model=%d (v.%d), Ret.delay=%dus, Ret.level=%d, V=%.1f, T=%d°\n", (int)(*(unsigned short*)(data+P_MODEL_NUMBER_L)), (int)data[P_VERSION], 2*(int)data[P_RETURN_DELAY_TIME], data[P_RETURN_LEVEL], DXL_VOLTAGE_TO_VOLT*(double)data[P_PRESENT_VOLTAGE], data[P_PRESENT_TEMPERATURE]);
	// Torque on, LED on
	fprintf(fOut, "Pause time: %d, Torque: %s, LED: %s\n", (int)data[P_PAUSE_TIME], data[P_TORQUE_ENABLE]?"ON":"OFF", data[P_LED]?"ON":"OFF");
	// Angle limit, torque limit
	fprintf(fOut, "%.1f° <= angle <= %.1f°, torque <= %.0f%%\n", (300.0*(int)(*(unsigned short*)(data+P_CW_ANGLE_LIMIT_L)))/(double)DXL_MAX_POSITION, (300.0*((int)(*(unsigned short*)(data+P_CCW_ANGLE_LIMIT_L))))/(double)DXL_MAX_POSITION, 100.0*(int)(*(unsigned short*)(data+P_TORQUE_LIMIT_L))/1023.0);
	// Current Angle
	fprintf(fOut, "Current Angle %.1f°\n",(300.0*(int)(*(unsigned short*)(data+P_PRESENT_POSITION_L)))/1023.0);
	// Control parameters
	fprintf(fOut, "Control compliance: \\%d\\ -%d- |*%d*| -%d- \\%d\\\n", (int)data[P_CW_COMPLIANCE_SLOPE], (int)data[P_CW_COMPLIANCE_MARGIN], (int)(*(unsigned short*)(data+P_PUNCH_L)), (int)data[P_CCW_COMPLIANCE_MARGIN], (int)data[P_CCW_COMPLIANCE_SLOPE]  );
	return DXL_SUCCESS;
}



