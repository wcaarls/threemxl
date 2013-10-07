// Dynamixel control code - C++ file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl


#include <stdlib.h>

#include <threemxl/platform/hardware/dynamixel/CDxlConfig.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>


CDxlConfig::CDxlConfig()
{
	mCalibType				= dxlCtNone;
}

CDxlConfig::~CDxlConfig()
{
}

int CDxlConfig::configureDynamixel(CDxlGeneric* dxl)
{
	int result=0;
	// OR'ing the results is not exactly what we want, but anyway it will be
	// a good indication in case something goes wrong.

	// Skip internal ID and serial port; they are used inside CDynamixel only.
	if (mReturnDelay.isSet())
		{ if( dxl->setReturnDelayTime(mReturnDelay) != DXL_SUCCESS) return DXL_ERROR;}
	if (mAngleLowerLimit.isSet() && mAngleUpperLimit.isSet())
		{if( dxl->setAngleLimits(mAngleLowerLimit, mAngleUpperLimit) != DXL_SUCCESS) return DXL_ERROR;}
	if (mTorqueLimit.isSet())
		{if( dxl->setTorqueLimit(mTorqueLimit) != DXL_SUCCESS) return DXL_ERROR;}
	if (mClockwiseIsPositive.isSet())
		{dxl->setPositiveDirection(mClockwiseIsPositive);}

//<RobotisDynamixel>
	if (mTempLimit.isSet())
		{if( dxl->setTemperatureLimit(mTempLimit) != DXL_SUCCESS) return DXL_ERROR;}
	if (mVoltageLowerLimit.isSet() && mVoltageUpperLimit.isSet())
		{if( dxl->setVoltageLimits(mVoltageLowerLimit, mVoltageUpperLimit) != DXL_SUCCESS) return DXL_ERROR;}
	if (mLED.isSet())
		{if( dxl->enableLED(mLED) != DXL_SUCCESS) return DXL_ERROR;}
	if (mTorqueLimit.isSet())
		{if( dxl->setInitialTorqueLimit(mTorqueLimit) != DXL_SUCCESS) return DXL_ERROR;}
	if (mAlarmLED.isSet())
		{if( dxl->setAlarmLEDMask(mAlarmLED) != DXL_SUCCESS) return DXL_ERROR;}
	if (mAlarmShutdown.isSet())
		{if( dxl->setAlarmShutdownMask(mAlarmShutdown) != DXL_SUCCESS) return DXL_ERROR;}
	if (mComplianceMargin.isSet() && mComplianceSlope.isSet())
		{if( dxl->setCompliance(mComplianceMargin, mComplianceSlope) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPunch.isSet())
		{if( dxl->setPunch(mPunch) != DXL_SUCCESS) return DXL_ERROR;}
//</RobotisDynamixel>
//<3mxl>
	if (m3mxlMode.isSet())
		{if( dxl->set3MxlMode(m3mxlMode) != DXL_SUCCESS) return DXL_ERROR;}
	if (mWatchdogMode.isSet())
		{if( dxl->setWatchdogMode(mWatchdogMode) != DXL_SUCCESS) return DXL_ERROR;}
	if (mWatchdogTime.isSet())
		{if( dxl->setWatchdogTime(mWatchdogTime) != DXL_SUCCESS) return DXL_ERROR;}
	if (mWatchdogMult.isSet())
		{if( dxl->setWatchdogMultiplier(mWatchdogMult) != DXL_SUCCESS) return DXL_ERROR;}
	if (mStatusReturnLevel.isSet())
		{if( dxl->setRetlevel(mStatusReturnLevel) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMotorConstant.isSet())
		{if( dxl->setMotorConstant(mMotorConstant * MOTOR_CONSTANT_MULTIPLIER) != DXL_SUCCESS) return DXL_ERROR;}
	if (mGearboxRatioMotor.isSet())
		{if( dxl->setGearboxRatioMotor(mGearboxRatioMotor) != DXL_SUCCESS) return DXL_ERROR;}
	if (mGearboxRatioJoint.isSet())
		{if( dxl->setGearboxRatioJoint(mGearboxRatioJoint) != DXL_SUCCESS) return DXL_ERROR;}
	if (mEncoderCountMotor.isSet())
		{if( dxl->setEncoderCountMotor(mEncoderCountMotor) != DXL_SUCCESS) return DXL_ERROR;}
	if (mOffsetMotor.isSet())
		{if( dxl->setMotorOffset(mOffsetMotor) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMaxUninitialisedMotorCurrent.isSet())
		{if( dxl->setMaxUninitializedMotorCurrent(mMaxUninitialisedMotorCurrent) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMaxMotorCurrent.isSet())
		{if( dxl->setMaxMotorCurrent(mMaxMotorCurrent) != DXL_SUCCESS) return DXL_ERROR;}
	if (mEncoderCountJoint.isSet())
		{if( dxl->setEncoderCountJoint(mEncoderCountJoint) != DXL_SUCCESS) return DXL_ERROR;}
	if (mOffsetJoint.isSet())
		{if( dxl->setJointOffset(mOffsetJoint) != DXL_SUCCESS) return DXL_ERROR;}
	if (mZeroLengthSpring.isSet())
		{if( dxl->setZeroLengthSpring(mZeroLengthSpring) != DXL_SUCCESS) return DXL_ERROR;}
	if (mSpringStiffness.isSet())
		{if( dxl->setSpringStiffness(mSpringStiffness) != DXL_SUCCESS) return DXL_ERROR;}
	if (mJointClockWiseIsPositive.isSet())
		{if( dxl->setPositiveDirectionJoint(mJointClockWiseIsPositive) != DXL_SUCCESS) return DXL_ERROR;}
	if (mAcceleration.isSet())
		{if( dxl->setAcceleration(mAcceleration) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMaxPeakMotorCurrent.isSet())
		{if( dxl->setMaxPeakMotorCurrent(mMaxPeakMotorCurrent) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMaxContinuousMotorCurrent.isSet())
		{if( dxl->setMaxContinuousMotorCurrent(mMaxContinuousMotorCurrent) != DXL_SUCCESS) return DXL_ERROR;}
	if (mMotorWindingTimeConstant.isSet())
		{if( dxl->setMotorWindingTimeConstant(mMotorWindingTimeConstant) != DXL_SUCCESS) return DXL_ERROR;}
	if (mEncoderIndexLevelMotor.isSet())
		{if( dxl->setEncoderIndexLevelMotor(mEncoderIndexLevelMotor) != DXL_SUCCESS) return DXL_ERROR;}
	if (mWheelDiameter.isSet())
		{if( dxl->setWheelDiameter(mWheelDiameter) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPCurrent.isSet())
		{if( dxl->setPIDCurrent(mPCurrent, mDCurrent, mICurrent, mILCurrent) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPPosition.isSet())
		{if( dxl->setPIDPosition(mPPosition, mDPosition, mIPosition, mILPosition) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPSpeed.isSet())
		{if( dxl->setPIDSpeed(mPSpeed, mDSpeed, mISpeed, mILSpeed) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPTorque.isSet())
		{if( dxl->setPIDTorque(mPTorque, mDTorque, mITorque, mILTorque) != DXL_SUCCESS) return DXL_ERROR;}
	if (mPEnergy.isSet())
		{if( dxl->setPIDEnergy(mPEnergy, mDEnergy, mIEnergy, mILEnergy) != DXL_SUCCESS) return DXL_ERROR;}
//</3mxl>

	return result;
}

CDxlConfig* CDxlConfig::setID(const int ID)
{
	mID = ID;
	return this;
}

bool CDxlConfig::readConfig(const CConfigSection &configNode)
{
	bool configresult = true;
	configresult &= configNode.get("ID", &mID);
	configresult &= configNode.get("type", &mDxlTypeStr, "undefined");
	configresult &= configNode.get("returndelay", &mReturnDelay);
	configresult &= configNode.get("anglelowerlimit", &mAngleLowerLimit);
	configresult &= configNode.get("angleupperlimit", &mAngleUpperLimit);
	configresult &= configNode.get("clockwiseispositive", &mClockwiseIsPositive);
	configresult &= configNode.get("torquelimit", &mTorqueLimit);
	//<3mxl>
	configresult &= configNode.get("mxlmode",&m3mxlMode);
	configresult &= configNode.get("watchdogmode",&mWatchdogMode);
	configresult &= configNode.get("watchdogtime",&mWatchdogTime);
	configresult &= configNode.get("watchdogmultiplier",&mWatchdogMult);
	configresult &= configNode.get("statusreturnlevel",&mStatusReturnLevel);
	configresult &= configNode.get("motorconstant",&mMotorConstant);
	configresult &= configNode.get("gearboxratiomotor",&mGearboxRatioMotor);
	configresult &= configNode.get("gearboxratiojoint",&mGearboxRatioJoint);
	configresult &= configNode.get("encodercountmotor",&mEncoderCountMotor);
	configresult &= configNode.get("offsetmotor",&mOffsetMotor);
	configresult &= configNode.get("maxuninitialisedmotorcurrent",&mMaxUninitialisedMotorCurrent);
	configresult &= configNode.get("maxmotorcurrent",&mMaxMotorCurrent);
	configresult &= configNode.get("encodercountjoint",&mEncoderCountJoint);
	configresult &= configNode.get("offsetjoint",&mOffsetJoint);
//	configresult &= configNode.get("minjointangle",&mMinJointAngle); we already have mAngleLowerLimit && mAngleUpperLimit
//	configresult &= configNode.get("maxjointangle",&mMaxJointAngle);
	configresult &= configNode.get("zerolengthspring",&mZeroLengthSpring);
	configresult &= configNode.get("springstiffness",&mSpringStiffness);
	configresult &= configNode.get("jointclockwiseispositive",&mJointClockWiseIsPositive);
	configresult &= configNode.get("acceleration",&mAcceleration);

	configresult &= configNode.get("maxpeakmotorcurrent",&mMaxPeakMotorCurrent);
	configresult &= configNode.get("maxcontinuousmotorcurrent",&mMaxContinuousMotorCurrent);
	configresult &= configNode.get("motorwindingtimeconstant",&mMotorWindingTimeConstant);
	configresult &= configNode.get("encoderindexlevelmotor",&mEncoderIndexLevelMotor);
	configresult &= configNode.get("wheeldiameter",&mWheelDiameter);
	
	configresult &= configNode.get("pcurrent",&mPCurrent);
	configresult &= configNode.get("icurrent",&mICurrent);
	configresult &= configNode.get("dcurrent",&mDCurrent);
	configresult &= configNode.get("ilcurrent",&mILCurrent);

	configresult &= configNode.get("pposition",&mPPosition);
	configresult &= configNode.get("iposition",&mIPosition);
	configresult &= configNode.get("dposition",&mDPosition);
	configresult &= configNode.get("ilposition",&mILPosition);
	
	configresult &= configNode.get("pspeed",&mPSpeed);
	configresult &= configNode.get("ispeed",&mISpeed);
	configresult &= configNode.get("dspeed",&mDSpeed);
	configresult &= configNode.get("ilspeed",&mILSpeed);

	configresult &= configNode.get("ptorque",&mPTorque);
	configresult &= configNode.get("itorque",&mITorque);
	configresult &= configNode.get("dtorque",&mDTorque);
	configresult &= configNode.get("iltorque",&mILTorque);

	configresult &= configNode.get("penergy",&mPEnergy);
	configresult &= configNode.get("ienergy",&mIEnergy);
	configresult &= configNode.get("denergy",&mDEnergy);
	configresult &= configNode.get("ilenergy",&mILEnergy);
	
	//</3mxl>
	//<RobotisDynamixel>
	configresult &= configNode.get("templimit", &mTempLimit);
	configresult &= configNode.get("voltagelowerlimit", &mVoltageLowerLimit);
	configresult &= configNode.get("voltageupperlimit", &mVoltageUpperLimit);
	configresult &= configNode.get("led", &mLED);
	configresult &= configNode.get("alarmled", &mAlarmLED);
	configresult &= configNode.get("alarmshutdown", &mAlarmShutdown);
	configresult &= configNode.get("compliancemargin", &mComplianceMargin);
	configresult &= configNode.get("complianceslope", &mComplianceSlope);
	configresult &= configNode.get("punch", &mPunch);
	configresult &= configNode.get("nullangle", &mNullAngle);
	// Read calibration data: this is a '|'-separated string with DXLCONFIG_NUM_CALIBPOINTS values,
	// produced by the manual calibration device (large protractor plate).
	std::string calibDataStr;
	if (configNode.get("calibdata", &calibDataStr))
	{
		if (!calibDataStr.empty())
		{
			int calibDataIndex = 0;
			std::istringstream calibStream(calibDataStr);
			std::string dataFieldStr;

			while (std::getline(calibStream, dataFieldStr, '|'))
			{
				if (calibDataIndex >= DXLCONFIG_NUM_CALIBPOINTS)
				{
					printf("[ERROR] in CDxlConfig::readConfig() (Dxl ID = %d): more than %d calibration data points found!\n", (int)mID, DXLCONFIG_NUM_CALIBPOINTS);
					configresult = false;
					break;
				}
				mCalibData[calibDataIndex++] = atof(dataFieldStr.c_str()) + 1.5;	// Add 1.5 to compensate the slack in the calibration measuring method
			}
			if (calibDataIndex != DXLCONFIG_NUM_CALIBPOINTS)
			{
				printf("[ERROR] in CDxlConfig::readConfig() (Dxl ID = %d): less than %d calibration data points found!\n", (int)mID, DXLCONFIG_NUM_CALIBPOINTS);
				configresult = false;
			}
			else
				mCalibType = dxlCtManual;
		}
	}

	// Read direct angle LUT
	if (configNode.get("anglelut", &calibDataStr))
	{
		if (!calibDataStr.empty())
		{
			int calibDataIndex = 0;
			std::istringstream calibStream(calibDataStr);
			std::string dataFieldStr;

			while (std::getline(calibStream, dataFieldStr, '|'))
			{
				if (calibDataIndex >= DXL_NUM_POSITIONS)
				{
					printf("[ERROR] in CDxlConfig::readConfig() (Dxl ID = %d): more than %d calibration data points found!\n", (int)mID, DXL_NUM_POSITIONS);
					configresult = false;
					break;
				}
				mAngleLUT[calibDataIndex++] = atof(dataFieldStr.c_str());
			}
			if (calibDataIndex != DXL_NUM_POSITIONS)
			{
				printf("[ERROR] in CDxlConfig::readConfig() (Dxl ID = %d): less than %d calibration data points found!\n", (int)mID, DXL_NUM_POSITIONS);
				configresult = false;
			}
			else
			{
				//printf("[DXLCONFIG] Successfully read anglelut!\n");
				mCalibType = dxlCtAuto;
			}
		}
	}

	//</RobotisDynamixel>

	return configresult;
}
