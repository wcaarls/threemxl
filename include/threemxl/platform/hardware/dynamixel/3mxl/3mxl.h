// Dynamixel control code - header file
// Copyright (c) 2010 Eelko van Breda
// based on dynamixel.h (Erik Schuitema & Eelko van Breda)
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __3MXL_H_INCLUDED__
#define __3MXL_H_INCLUDED__

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <errno.h>
#include <math.h>
#include <threemxl/platform/io/logging/Log2.h>


#include "../Byte.h"
#include "../CDxlGeneric.h"
#include "../CDxlPacket.hpp"
#include "3mxlControlTable.h"

#define __DBG__

// Dynamixel error codes: they are all > 0
// When returned < 0 from any function, this means the
// error code was returned internally by CDynamixel and the
// code does not originate from an actual motor.
#define DXL_ERR_INPUT_VOLTAGE			1
#define DXL_ERR_ANGLE_LIMIT				2
#define DXL_ERR_OVERHEATING				4
#define DXL_ERR_RANGE					8
#define DXL_ERR_CHECKSUM				16
#define DXL_ERR_OVERLOAD				32
#define DXL_ERR_INSTRUCTION				64

#define INITIAL_RETURN_DELAY_TIME		500
#define INITIAL_TORQUE_LIMIT			1.0
#define INITIAL_3MXL_MODE				POSITION_MODE
#define INITIAL_WATCHDOG_MODE			0
#define INITIAL_WATCHDOG_TIME			100
#define INITIAL_WATCHDOG_MULT			1
#define	INITIAL_STATUS_RETURN_LEVEL		1
#define	INITIAL_MOTOR_CONSTANT			0
#define	INITIAL_GEARBOX_RATIO			1
#define INITIAL_ENCODER_COUNT_MOTOR		0
#define	INITIAL_ENCODER_COUNT_JOINT		0
#define	INITIAL_OFFSET_MOTOR			0
#define	INITIAL_OFFSET_JOINT			0
#define INITIAL_MAX_UNINITIALIZED_MOTOR_CURRENT 0
#define	INITIAL_MAX_MOTOR_CURRENT		0
#define INITIAL_MIN_JOINT_ANGLE			0
#define	INITIAL_MAX_JOINT_ANGLE			0
#define	INITIAL_SPRING_STIFFNESS		0
#define	INITIAL_ZERO_SPRING_LENGTH 		0
#define	INITIAL_JOINT_DIRECTION			1


#define PI_DOUBLE 3.1415926535897932


// Dynamixel on, off and toggle convention
#define DXL_OFF					0
#define DXL_ON					1
#define DXL_TOGGLE				2

//enum E3mxlMode
//{
//	mxlMode_Uninitialised,	//startup mode
//	mxrMode_Torque,			//torque control
//	mxlMode_Speed,			//speed control
//	mxlMode_SpeedPosition,	//get to a position with the given speed as max speed
//	mxlMode_PositionSpeed,	//get to a position and have the speed at that position
//	mxlMode_SEA_PSTPD		//setPosSpeedTorquePPosDPos mode
//};


/**
 * This class interfaces a 3mxl
 * all settings are directly given to the 3mxl if possible.
 * Status information is cached in this object
 */
class C3mxl: public CDxlGeneric
{
	protected:

		// Internal motor configuration
		CDxlConfig	mConfig;
		CLog2 		mLog;
		// In SI units
		double		mVoltage;		// Motor voltage [V]
		double		mBusVoltage;		// Bus voltage [V]
		double		mCurrentADCVoltage;	// Current ADC voltage [V]
		double		mAnalog1Voltage;	// Analog input 1 voltage [V]
		double		mAnalog2Voltage;	// Analog input 2 voltage [V]
		double		mAnalog3Voltage;	// Analog input 3 voltage [V]
		double		mAnalog4Voltage;	// Analog input 4 voltage [V]
		double 		mCurrent;		// Current [A]
		double 		mTorque;		// Torque [Nm]
		double		mPosition;		// Position [rad]
		double		mLinearPosition;	// Position [m]
		double		mSpeed;			// Speed [rad/s]
		double		mAcceleration;	// Acceleration [rad/s^2]
		double		mLinearAcceleration;	// Acceleration [m/s^2]
		unsigned char		mStatus;			// Status value of 3mxl
		unsigned char		mMotorInitialized;	// this value tells you that the motor did the initialization procedure

		TMxlLog mMxlLog;
		BYTE mMxlMode;

//		double		mLoad;			// Current load expressed as a ratio of the maximum torque [dimensionless]
//		double		mTemperature;	// Temperature [deg. C]

		// Return level. Can be one of the following:
		// 0: respond to Ping only
		// 1: respond only when asked for data
		// 2: respond always

		// Clockwise and counter-clockwise angle limits.
		// These are NOT saved in SI units but as Dynamixel WORD values, because
		// we want to use the exact CWAngleLimit and CCWAngleLimit in setSpeed()
		// as they are present in the control table of the Dynamixel (also as WORD values).
		// Otherwise, we may set a slightly larger or smaller goal position in setSpeed() and
		// encounter an angle limit error. Then setSpeed() would fail completely.

//		unsigned char	m3mxlMode;
//		BYTE		mWatchdogMode;
//		BYTE		mWatchdogTime;
//		BYTE		mWatchdogMult;
//		BYTE		mStatusReturnLevel;
//		double		mMotorConstant;
//		double		mGearboxRatio;
//		COptionWord	mEncoderCountMotor;				//total nr of encoder counts motor
//		COptionDouble	mOffsetMotor;				//zero position of the motor in rad only used temporarily
//		double		mMinMotorAngleLimit;
//		double		mMaxMotorAngleLimit;
//		double		mMaxUninitialisedMotorCurrent;
//		double		mMaxMotorCurrent;
//		int			mMotorDirection;				// Should be either 1.0 or -1.0 and is used as internal multiplication factor!
//		COptionWord	mEncoderCountJoint;				//total number of encoder counts joint
//		COptionDouble	mOffsetJoint;				//zero position of the joint in rad only used temporarily
//		double		mMinJointAngle;
//		double		mMaxJointAngle;
//		double		mZeroLengthSpring;
//		double		mSpringStiffness;
//		int			mJointDirection;

		int			clip(const int value, const int min, const int max)
		{
			int result = value;
			if (result < min)
				result = min;
			if (result > max)
				result = max;
			return result;
		}

		int			round(double val)
		{
			return (int)floor(val+0.5);
		}

		// Internal conversions
		double 		mxlCurrentToInternalCurrent(WORD current);
		WORD 		internalCurrentToMxlCurrent(double current);
		double 		mxlVoltageToInternalVoltage(WORD voltage);
		WORD 		internalVoltageToMxlVoltage(double voltage);
		double		mxlPosToInternalPos(WORD pos);
		WORD		internalPosToMxlPos(double pos);
		double		mxlSpeedToInternalSpeed(WORD speed);
		WORD		internalSpeedToMxlSpeed(double speed);
		double		mxlAccelerationToInternalAcceleration(WORD acceleration);
		WORD		internalAccelerationToMxlAcceleration(double acceleration);
		double		mxlLinearPosToInternalLinearPos(DWORD pos);
		DWORD		internalLinearPosToMxlLinearPos(double pos);
		double		mxlLinearSpeedToInternalLinearSpeed(WORD speed);
		WORD		internalLinearSpeedToMxlLinearSpeed(double speed);
		double		mxlLinearAccelerationToInternalLinearAcceleration(WORD acceleration);
		WORD		internalLinearAccelerationToMxlLinearAcceleration(double acceleration);
		double		mxlTorqueToInternalTorque(WORD torque);
		WORD		internalTorqueToMxlTorque(double torque);
		WORD 		internalFreqToMxlFreq(double frequency);
		WORD 		internalAmplitudeToMxlAmplitude(double amplitude);
		WORD 		internalPhaseToMxlPhase(double phase);
		double 		mxlPWMToInternalPWM(WORD current);
		WORD 		internalPWMToMxlPWM(double current);
		int 		clipToMaxWord(int value);
		int 		clipToMaxDWord(int value);

	public:

		C3mxl();
		virtual			~C3mxl();
		
		static const char*	translateErrorCode(int errorCode);

		int			getID() {return mID;}
		virtual void		setConfig(CDxlConfig* config);
		virtual void		setSerialPort(LxSerial* serialPort);
		virtual int		init(bool sendConfigToMotor=true);

		virtual int		setPos(double pos, bool shouldSyncWrite=false);
		virtual int		setLinearPos(double pos, bool shouldSyncWrite=false);
		virtual int		setPos(double pos, double absSpeed, bool shouldSyncWrite=false);
		virtual int		setLinearPos(double pos, double absSpeed, bool shouldSyncWrite=false);
		virtual int		setPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite=false);
		virtual int		setLinearPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite=false);
		virtual int		setSpeed(double speed, bool shouldSyncWrite=false);
		virtual int		setLinearSpeed(double speed, bool shouldSyncWrite=false);
		virtual int		setAcceleration(double acceleration, bool shouldSyncWrite=false);
		virtual int		setLinearAcceleration(double acceleration, bool shouldSyncWrite=false);
		virtual int		setTorque(double torque, bool shouldSyncWrite=false);
		virtual int		setCurrent(double current, bool shouldSyncWrite=false);
		virtual int		setPWM(double pwm, bool shouldSyncWrite=false);
		virtual int		setPosSpeedTorquePPosDPos(double pos, double speed, double torque, int pPos, int dPos, bool shouldSyncWrite=false);
		virtual int		setPIDCurrent(double p, double d, double i, double i_limit, bool shouldSyncWrite=false);
		virtual int		setPIDPosition(double p, double d, double i, double i_limit, bool shouldSyncWrite=false);
		virtual int		setPIDSpeed(double p, double d, double i, double i_limit, bool shouldSyncWrite=false);
		virtual int		setPIDTorque(double p, double d, double i, double i_limit, bool shouldSyncWrite=false);
		virtual int		setPIDEnergy(double p, double d, double i, double i_limit, bool shouldSyncWrite=false);

		virtual int		getPIDCurrent(double &p, double &d, double &i, double &i_limit);
		virtual int		getPIDPosition(double &p, double &d, double &i, double &i_limit);
		virtual int		getPIDSpeed(double &p, double &d, double &i, double &i_limit);
		virtual int		getPIDTorque(double &p, double &d, double &i, double &i_limit);
		virtual int		getPIDEnergy(double &p, double &d, double &i, double &i_limit);

		virtual int		setRetlevel(const int returnlevel);
		virtual int		setReturnDelayTime(const int microsecondsReturnDelay);
		virtual int		setBaudRate(const int baudRate);
		virtual int		setInitialTorqueLimit(double absMaxTorque);
		virtual int		setTorqueLimit(double absMaxTorque);
		virtual int		setAngleLimits(double lowerLimit, double upperLimit);
		virtual int		setAngleLowerLimit(double limit);
		virtual int		setAngleUpperLimit(double limit);
		virtual int		changeID(const int newID);

		virtual int		set3MxlMode(BYTE mxlMode, bool shouldSyncWrite = false);
		virtual void		setPositiveDirection(bool clockwiseIsPositive);
		virtual int		setPositiveDirectionMotor(bool clockwiseIsPositive);
		virtual int		setPositiveDirectionJoint(bool clockwiseIsPositive);
		virtual int		setMotorOffset(double offset);
		virtual int		setJointOffset(double offset);
		virtual int		setEncoderCountMotor(WORD encodercount);
		virtual int		setEncoderCountJoint(WORD encodercount);
		virtual int		setMotorConstant(WORD motorconstant);
		virtual int		setMaxPeakMotorCurrent(double current);
		virtual int		setMaxContinuousMotorCurrent(double current);
		virtual int		setMotorWindingTimeConstant(double time);
		virtual int		setEncoderIndexLevelMotor(BYTE level);
		virtual int		setWheelDiameter(double diameter);		
		virtual int		setGearboxRatioMotor(float gearboxratio);
		virtual int		setGearboxRatioJoint(float gearboxratio);
		virtual int		setWatchdogMode(BYTE watchdogmode);
		virtual int		setWatchdogTime(BYTE watchdogtime);
		virtual int		setWatchdogMultiplier(BYTE watchdogmultiplier);;
		virtual int		setZeroLengthSpring(double parameterInRad);
		virtual int		setSpringStiffness(double stiffness);
		virtual int		setReferenceEnergy(double energy);
		virtual int		setSineFrequency(double frequency, bool shouldSyncWrite=false);
		virtual int		setSineAmplitude(double amplitude, bool shouldSyncWrite=false);
		virtual int		setSinePhase(double phase, bool shouldSyncWrite=false);
		virtual int		setLogInterval(BYTE interval);
		virtual int		setSyncReadIndex(BYTE index);

		virtual int		getPos();
		virtual int		getLinearPos();
		virtual int		getPosAndSpeed();
		virtual int		getTorquePosSpeed();
		virtual int		getAcceleration();
		virtual int		getLinearAcceleration();
		virtual int		getState();
		virtual int		getVoltage();
		virtual int		getBusVoltage();
		virtual int		getSensorVoltages();
		virtual int		getCurrent();
		virtual int		getTorque();
		virtual int		getStatus();
		virtual int		getLog();
		virtual int		get3MxlMode();

		virtual double	presentPos()			{return mPosition;}
		virtual double	presentLinearPos()		{return mLinearPosition;}
		virtual double	presentSpeed()			{return mSpeed;}
		virtual double  presentAcceleration()		{return mAcceleration;};
		virtual double  presentLinearAcceleration()	{return mLinearAcceleration;};
		virtual double	presentLoad()			{mLogWarningLn("presentLoadfunction not implemented");return DXL_NOT_INITIALIZED;}
		virtual double	presentVoltage()		{return mVoltage;}
		virtual double	presentBusVoltage()		{return mBusVoltage;}
		virtual double	presentCurrentADCVoltage()	{return mCurrentADCVoltage;}
		virtual double	presentAnalog1Voltage()		{return mAnalog1Voltage;}
		virtual double	presentAnalog2Voltage()		{return mAnalog2Voltage;}
		virtual double	presentAnalog3Voltage()		{return mAnalog3Voltage;}
		virtual double	presentAnalog4Voltage()		{return mAnalog4Voltage;}
		virtual double	presentTemp()			{mLogWarningLn("presentTemp function not implemented");return DXL_NOT_INITIALIZED;}
		virtual double	presentCurrent()		{return mCurrent;}
		virtual double	presentTorque()			{return mTorque;}
		virtual int	presentStatus()			{return (int)mStatus;}
		virtual bool	presentMotorInitState()		{return (bool)mMotorInitialized;}

		WORD			presentCWAngleLimit()	{mLogWarningLn("presentCWAngleLimit function not implemented"); return DXL_NOT_INITIALIZED;}
		WORD			presentCCWAngleLimit()	{mLogWarningLn("presentCCWAngleLimit function not implemented"); return DXL_NOT_INITIALIZED;}
		double			presentAngleLowerLimit(){mLogWarningLn("presentAngleLowerLimit function not implemented"); return DXL_NOT_INITIALIZED;};
		double			presentAngleUpperLimit(){mLogWarningLn("presentAngleUpperLimit function not implemented"); return DXL_NOT_INITIALIZED;};

		virtual TMxlLog		presentLog()		{return mMxlLog;};
		virtual BYTE		present3MxlMode()	{return mMxlMode;};

		int				printReport(FILE* fOut);
    int       interpretControlData(BYTE address, BYTE length, BYTE *data);


		//not implemented:
//		virtual void	setNullAngle(double nullAngle);
//		virtual int		setBaudRateIndex(const BYTE baudRateIndex);
//		virtual int		setCompliance(BYTE complianceMargin, BYTE complianceSlope);
//		virtual int		setPunch(WORD punch);
//		virtual int		setAlarmLEDMask(const BYTE mask);
//		virtual int		setOperatingMode(const BYTE mode);
//		virtual int		setAlarmShutdownMask(const BYTE mask);
//		virtual int		enableLED(int state);
//		virtual int		enableTorque(int state);
//		virtual int		setVoltageLimits(double minVoltage, double maxVoltage);
//		virtual int		setTemperatureLimit(const int maxTemp);
//		virtual int		setEndlessTurnMode(bool enabled, bool shouldSyncWrite=false);
//		virtual int		setEndlessTurnTorque(double torqueRatio, bool shouldSyncWrite=false);
//		virtual int		getTemp();
//		virtual int		getAngleLimits();
};

#endif
