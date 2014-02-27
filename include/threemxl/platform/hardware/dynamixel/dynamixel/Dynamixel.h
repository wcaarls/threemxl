// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DYNAMIXEL_H_INCLUDED__
#define __DYNAMIXEL_H_INCLUDED__

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <threemxl/platform/io/logging/Log2.h>

#include "../Byte.h"
#include "../CDxlGeneric.h"
#include "../CDxlPacket.hpp"
#include "../CDxlConfig.h"
#include "DynamixelControlTable.h"

#define __DBG__

#define INITIAL_RETURN_DELAY_TIME		500
#define INITIAL_TEMPERATURE_LIMIT		80
#define INITIAL_VOLTAGE_LOWER_LIMIT		6.0
#define INITIAL_VOLTAGE_UPPER_LIMIT		24.0
#define INITIAL_TORQUE_LIMIT			1.0
#define INITIAL_COMPLIENCE_MARGIN		1
#define INITIAL_COMPLIENCE_SLOPE		32
#define INITIAL_PUNCH					32


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


// Dynamixel on, off and toggle convention
#define DXL_OFF					0
#define DXL_ON					1
#define DXL_TOGGLE				2

class CDynamixel: public CDxlGeneric
{
	protected:
		CLog2		mLog;

		// Internal motor configuration
		CDxlConfig	mConfig;
		double		mAngleLUT[DXL_NUM_POSITIONS];		// Lookup table that
		double		mDirection;							///< Should be either 1.0 or -1.0 and is used as internal multiplication factor!
		double		mNullAngle;							///< Null-angle in SI units AFTER the mDirection sign convention

		// In SI units
		double		mPosition;		///< Position [rad] in the range [0 - (10/6)pi] (0-300 degrees)
		double		mSpeed;			///< Speed [rad/s]
		double		mLoad;			///< Current load expressed as a ratio of the maximum torque [dimensionless]
		double		mVoltage;		///< Voltage [V]
		double		mTemperature;	///< Temperature [deg. C]

		/// Clockwise angle limit.
		/**
		 * Angle limits are NOT saved in SI units but as Dynamixel WORD values, because
		 * we want to use the exact CWAngleLimit and CCWAngleLimit in setSpeed()
		 * as they are present in the control table of the Dynamixel (also as WORD values).
		 * Otherwise, we may set a slightly larger or smaller goal position in setSpeed() and
		 * encounter an angle limit error. Then setSpeed() would fail completely.
		 */
		WORD		mCWAngleLimit;
		WORD		mCCWAngleLimit;	///< Counter-clockwise angle limit. \see mCWAngleLimit.

		bool		mEndlessTurnMode;///< Virtual state

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
		double		dxlPosToInternalPos(WORD pos);
		int			internalPosToDxlPos(double pos);
		double		dxlSpeedToInternalSpeed(WORD speed);
		int			internalSpeedToDxlSpeed(double speed);
		double		dxlTorqueToInternalTorque(WORD torque);
		WORD		internalTorqueToDxlTorque(double torque);

	public:
		CDynamixel();
		virtual			~CDynamixel();

		virtual int		getID() {return mID;}
		virtual void	setConfig(CDxlConfig* config);
		virtual void	setSerialPort(LxSerial* serialPort);
		virtual int		init(bool sendConfigToMotor=true);

		virtual void	setPositiveDirection(bool clockwiseIsPositive);
		virtual void	setNullAngle(double nullAngle);
		virtual int		setPos(double pos, double absSpeed, bool shouldSyncWrite=false);

		/// Move to angle limit with reference speed.
		/**
		 * \warning Does not actually implement velocity control! The dynamixel generates a reference trajectory!
		 * \param speed Speed in [rad/s]. Speed can be positive or negative.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setSpeed(double speed, bool shouldSyncWrite=false);
		virtual int		setRetlevel(const int returnlevel);
		virtual int		setBaudRateIndex(const BYTE baudRateIndex);
		virtual int		setBaudRate(const int baudRate);
		virtual int		setReturnDelayTime(const int microsecondsReturnDelay);
		virtual int		setInitialTorqueLimit(double absMaxTorque);
		virtual int		setTorqueLimit(double absMaxTorque);
		virtual int		setAngleLimits(double lowerLimit, double upperLimit);
		virtual int		setVoltageLimits(double minVoltage, double maxVoltage);
		virtual int		setTemperatureLimit(const int maxTemp);
		virtual int		setAngleLowerLimit(double limit);
		virtual int		setAngleUpperLimit(double limit);
		virtual int		setCompliance(BYTE complianceMargin, BYTE complianceSlope);
		virtual int		setPunch(WORD punch);
		virtual int		setAlarmLEDMask(const BYTE mask);
		virtual int		setOperatingMode(const BYTE mode);
		virtual int		setAlarmShutdownMask(const BYTE mask);
		virtual int		changeID(const int newID);
		virtual int		enableLED(int state);
		virtual int		enableTorque(int state);
		virtual int		setEndlessTurnMode(bool enabled, bool shouldSyncWrite=false);
		virtual int		setEndlessTurnTorque(double torqueRatio, bool shouldSyncWrite=false);

		virtual int		getPos();
		virtual int		getPosAndSpeed();
		virtual int		getTemp();
		virtual int		getState();
		virtual int		getAngleLimits();

		virtual double	presentPos()			{return mPosition;}
		virtual double	presentLinearPos()		{mLogWarningLn("presentLinearPos function not implemented");return 0;}
		virtual double	presentSpeed()			{return mSpeed;}
		virtual double  presentAcceleration()		{mLogWarningLn("presentAcceleration function not implemented");return 0;};
		virtual double  presentLinearAcceleration()	{mLogWarningLn("presentLinearAcceleration function not implemented");return 0;};
		virtual double	presentLoad()			{return mLoad;}
		virtual double	presentVoltage()		{return mVoltage;}
		virtual double	presentBusVoltage()		{mLogWarningLn("presentBusVoltage function not implemented"); return 0;}
		virtual double	presentCurrentADCVoltage()	{mLogWarningLn("presentCurrentADCVoltage function not implemented"); return 0;}
		virtual double	presentAnalog1Voltage()		{mLogWarningLn("presentAnalog1Voltage function not implemented"); return 0;}
		virtual double	presentAnalog2Voltage()		{mLogWarningLn("presentAnalog2Voltage function not implemented"); return 0;}
		virtual double	presentAnalog3Voltage()		{mLogWarningLn("presentAnalog3Voltage function not implemented"); return 0;}
		virtual double	presentAnalog4Voltage()		{mLogWarningLn("presentAnalog4Voltage function not implemented"); return 0;}
		virtual double	presentTemp()			{return mTemperature;}
		virtual double	presentCurrent()		{mLogWarningLn("presentCurrent function not implemented");return 0;}
		virtual double	presentTorque()			{mLogWarningLn("presentTorque function not implemented");return 0;}
		virtual WORD	presentCWAngleLimit()		{return mCWAngleLimit;}
		virtual WORD	presentCCWAngleLimit()		{return mCCWAngleLimit;}
		virtual double	presentAngleLowerLimit();
		virtual double	presentAngleUpperLimit();

		virtual int		printReport(FILE* fOut);
};

#endif
