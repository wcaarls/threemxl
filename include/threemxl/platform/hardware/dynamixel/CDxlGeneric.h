// Dynamixel control code - header file
// Copyright (c) 2008 Erik Schuitema, Eelko van Breda
// Delft University of Technology
// www.dbl.tudelft.nl

#ifndef __DXLGENERIC_H_INCLUDED__
#define __DXLGENERIC_H_INCLUDED__
#include "CDxlCom.h"
#include "CDxlConfig.h"
#include "CDxlGroup.h"
#include <threemxl/platform/io/logging/Log2.h>

#include <iostream>
#include <iomanip>

#define PACKET_RETRY_FACTOR 1

//forward declarations
class CDxlConfig;
class CDxlGroup;

/// 3mxl log entry
typedef struct
{
  uint16_t time; ///< Time since start of logging
  float pwm,     ///< PWM duty cycle, between -1 and 1
        current, ///< Current in [A]
        voltage, ///< Bus voltage in [V]
        desired, ///< Reference value (mode dependent)
        actual;  ///< Actual value (mode dependent)
} __attribute__((packed)) TMxlLogEntry;

inline std::ostream &operator<<(std::ostream &outs, const TMxlLogEntry &obj)
{
  outs << std::fixed
       << std::setw(8) << obj.time << " "
       << std::setw(8) << obj.pwm << " "
       << std::setw(8) << std::setprecision(3) << obj.current << " "
       << std::setw(8) << std::setprecision(3) << obj.voltage << " "
       << std::setw(8) << std::setprecision(3) << obj.desired << " "
       << std::setw(8) << std::setprecision(3) << obj.actual
       << std::resetiosflags(std::ios_base::floatfield);

  return outs;
}

typedef std::vector<TMxlLogEntry> TMxlLog;

/// Generic interface to both CDynamixel and C3mxl
/**
 * Some of the set* functions can also be used in a SYNC_WRITE packet fashion; use CDxlGroup for that.
 *
 * get*() functions cannot be used in a SYNC_WRITE packet fashion, since they return data.
 */
class CDxlGeneric: public CDxlCom
{
protected:
		CDxlGroup*	mpGroup;
		int			mID;
		int			mRetlevel;
		CLog2 		mLog;

public:
		CDxlGeneric():  mLog("CDxlGeneric")
		{
//			mLog.setLevel(llDebug);
//			mLog.enableConsoleOutput("false");
			mpGroup = NULL;
		}
		virtual ~CDxlGeneric() {};

		/// Read data from hardware.
		/**
		 * \param startingAddress Starting address in control table.
		 * \param dataLength Bytes to write.
		 * \param data Buffer of \c dataLength bytes.
		 */
		int 			readData(BYTE startingAddress, BYTE dataLength, BYTE *data);

		/// Write data to hardware.
		/**
		 * \param startingAddress Starting address in control table.
		 * \param dataLength Bytes to write.
		 * \param data Buffer of \c dataLength bytes.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 **/
		int				writeData(BYTE startingAddress, BYTE dataLength, BYTE *data, bool shouldSyncWrite=false);
		
		/// Execute registered command.
		int				action();
		
		/// Reset control table.
		int				reset();
		
		/// Check for motor presence.
		/** \returns DXL_SUCCESS on success, error otherwise. */
		int				ping();

		/// Get motor ID.
		/**
		 * \warning Merely returns the internally cached ID of the motor and does not request it (how could it, anyway?)
		 * \returns Motor ID.
		 */
		virtual int		getID()=0;

		/// Set group membership.
		/**
		 * \warning Don't set this manually. Use CDxlGroup::addNewDynamixel.
		 * \param group Group to set membership to.
		 */
		virtual void	setGroup(CDxlGroup* group) { mpGroup = group;}

		/// Set motor configuration.
		/**
		 * The configuration is written to the motor during init().
		 * \warning After init(), the fields of mConfig are not guaranteed to be up to date anymore!
		 */
		virtual void	setConfig(CDxlConfig* config) {};

		/// Set serial port.
		/** serialPort Serial port. */
		virtual void	setSerialPort(LxSerial* serialPort) {};

		/// Initialize this Dynamixel.
		/**
		 * \warning Make sure you call setConfig() before calling init()!
		 * \param sendConfigToMotor Send configuration to motor
		 */
		virtual int		init(bool sendConfigToMotor=true) {mLogWarningLn("init function not implemented"); return false;};

		/// Set direction convention.
		/** \param clockwiseIsPositive True if clockwise direction is positive */
		virtual void	setPositiveDirection(bool clockwiseIsPositive) {mLogWarningLn("setPositiveDirection function not implemented");};

		/// Set null angle convention.
		/** \param nullAngle Null angle in [rad] */
		virtual void	setNullAngle(double nullAngle) {mLogWarningLn("setNullAngle function not implemented");};

		/// Set reference position.
		/**
		 * \param pos Position in [rad]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setPos(double pos, bool shouldSyncWrite=false) {mLogWarningLn("setPos/1 function not implemented"); return false;};
		
		/// Set reference position and velocity.
		/**
		 * \param pos Position in [rad]
		 * \param absSpeed Speed in [rad/s]. Speed is absolute (positive in either direction). 0 speed means slowest. Negative speed means: MAXIMUM speed.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setPos(double pos, double absSpeed, bool shouldSyncWrite=false) {mLogWarningLn("setPos/2 function not implemented"); return false;};
		
		/// Set reference position, velocity and acceleration.
		/**
		 * \param pos Position in [rad]
		 * \param absSpeed Speed in [rad/s]. Speed is absolute (positive in either direction). 0 speed means slowest. Negative speed means: MAXIMUM speed.
		 * \param acceleration Acceleration in [rad/s^2].
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite=false) {mLogWarningLn("setPos/3 function not implemented"); return false;};
		
		/// Set linear reference position.
		/**
		 * \param pos Position in [m]
		 * \param absSpeed Speed in [m/s]. Speed is absolute (positive in either direction). 0 speed means slowest. Negative speed means: MAXIMUM speed.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setLinearPos(double pos, bool shouldSyncWrite=false) {mLogWarningLn("setPosLinear/1 function not implemented"); return false;};
		
		/// Set linear reference position and velocity.
		/**
		 * \param pos Position in [m]
		 * \param absSpeed Speed in [m/s]. Speed is absolute (positive in either direction). 0 speed means slowest. Negative speed means: MAXIMUM speed.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setLinearPos(double pos, double absSpeed, bool shouldSyncWrite=false) {mLogWarningLn("setPosLinear/2 function not implemented"); return false;};
		
		/// Set linear reference position, velocity and acceleration
		/**
		 * \param pos Position in [m]
		 * \param absSpeed Speed in [m/s]. Speed is absolute (positive in either direction). 0 speed means slowest. Negative speed means: MAXIMUM speed.
		 * \param acceleration Acceleration in [m/s^2].
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setLinearPos(double pos, double absSpeed, double acceleration, bool shouldSyncWrite=false) {mLogWarningLn("setPosLinear/3 function not implemented"); return false;};
		
		/// Set reference speed.
		/**
		 * \param speed Speed in [rad/s]. Speed can be positive or negative.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setSpeed(double speed, bool shouldSyncWrite=false) {mLogWarningLn("setSpeed function not implemented"); return false;};

		/// Set linear reference speed.
		/**
		 * \param speed Speed in [m/s]. Speed can be positive or negative.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setLinearSpeed(double speed, bool shouldSyncWrite=false) {mLogWarningLn("setLinearSpeed function not implemented"); return false;};

		/// Set acceleration for trajectory generation.
		/**
		 * \param acceleration Acceleration in [rad/s^2].
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setAcceleration(double acceleration, bool shouldSyncWrite=false) {mLogWarningLn("setAcceleration function not implemented"); return false;};

		/// Set linear acceleration for trajectory generation.
		/**
		 * \param acceleration Acceleration in [m/s^2].
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setLinearAcceleration(double acceleration, bool shouldSyncWrite=false) {mLogWarningLn("setLinearAcceleration function not implemented"); return false;};

		/// Set reference torque
		/**
		 * \param torque Torque in [Nm]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setTorque(double torque, bool shouldSyncWrite=false)  {mLogWarningLn("setTorque function not implemented"); return false;};

		/// Set reference current
		/**
		 * \param current Current in [A]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setCurrent(double current, bool shouldSyncWrite=false)  {mLogWarningLn("setCurrent function not implemented"); return false;};

		/// Set pulse width modulation duty cycle
		/**
		 * \param pwm PWM duty cycle, between -1 and 1.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPWM(double pwm, bool shouldSyncWrite=false)  {mLogWarningLn("setPWM function not implemented"); return false;};

		/// Set many references simultaneously.
		/**
		 * \param pos Position in [rad]
		 * \param speed Speed in [rad/s]
		 * \param torque Torque in [Nm]
		 * \param pPos Position controller P gain
		 * \param dPos Position controller D gain
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int 	setPosSpeedTorquePPosDPos(double pos, double speed, double torque, int pPos, int dPos, bool shouldSyncWrite=false) {mLogWarningLn("setPosSpeedTorquePPosDPos function not implemented"); return false;};

		/// Set PID for current control mode
		/**
		 * The error is in SI units ([A]), and the output range of the control
		 * equation is [-1, 1].
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPIDCurrent(double p, double d, double i, double i_limit, bool shouldSyncWrite=false)  {mLogWarningLn("setPIDCurrent function not implemented"); return false;};

		/// Get PID gains for current control mode
		/**
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \see setPIDCurrent
		 * */
		virtual int		getPIDCurrent(double &p, double &d, double &i, double &i_limit)  {mLogWarningLn("getPIDCurrent function not implemented"); return false;};

		/// Set PID for position control mode
		/**
		 * The error is in SI units ([rad]), and the output range of the control
		 * equation is [-1, 1].
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPIDPosition(double p, double d, double i, double i_limit, bool shouldSyncWrite=false)  {mLogWarningLn("setPIDPosition function not implemented"); return false;};

		/// Get PID gains for position control mode
		/**
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \see setPIDPosition
		 * */
		virtual int		getPIDPosition(double &p, double &d, double &i, double &i_limit)  {mLogWarningLn("getPIDPosition function not implemented"); return false;};

		/// Set PID for speed control mode
		/**
		 * The error is in SI units ([rad/s]), and the output range of the control
		 * equation is [-1, 1].
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPIDSpeed(double p, double d, double i, double i_limit, bool shouldSyncWrite=false)  {mLogWarningLn("setPIDSpeed function not implemented"); return false;};

		/// Get PID gains for speed control mode
		/**
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \see setPIDSpeed
		 * */
		virtual int		getPIDSpeed(double &p, double &d, double &i, double &i_limit)  {mLogWarningLn("getPIDSpeed function not implemented"); return false;};

		/// Set PID for torque control mode
		/**
		 * The error is in SI units ([Nm]), and the output range of the control
		 * equation is [-1, 1].
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPIDTorque(double p, double d, double i, double i_limit, bool shouldSyncWrite=false)  {mLogWarningLn("setPIDSpeed function not implemented"); return false;};

		/// Get PID gains for torque control mode
		/**
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \see setPIDTorque
		 * */
		virtual int		getPIDTorque(double &p, double &d, double &i, double &i_limit)  {mLogWarningLn("getPIDTorque function not implemented"); return false;};

		/// Set PID for energy control mode
		/**
		 * The error is in SI units ([J]), and the output range of the control
		 * equation is [-1, 1].
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setPIDEnergy(double p, double d, double i, double i_limit, bool shouldSyncWrite=false)  {mLogWarningLn("setPIDEnergy function not implemented"); return false;};

		/// Get PID gains for energy control mode
		/**
		 * \param p Proportional gain
		 * \param d Derivative gain
		 * \param i Integral gain
		 * \param i_limit Integration limit
		 * \see setPIDTorque
		 * */
		virtual int		getPIDEnergy(double &p, double &d, double &i, double &i_limit)  {mLogWarningLn("getPIDEnergy function not implemented"); return false;};

		/// Set status return level.
		/** \param returnlevel Return status packet on \li \c 0 PING \li \c 1 PING+READ \li \c 2 all commands */
		virtual int		setRetlevel(const int returnlevel) {mLogWarningLn("setRetlevel function not implemented"); return false;};

		/// Set baud rate in Dynamixel units.
		/** \param baudRateIndex Baudrate in Dynamixel BPS unit. [bits/s] = 2000000/(baudRateIndex+1). */
		virtual int		setBaudRateIndex(const BYTE baudRateIndex) {mLogWarningLn("setBaudRateIndex function not implemented"); return false;};

		/// Set baud rate in bits/s.
		/** \param baudRate Baudrate in [bit/s]. */
		virtual int		setBaudRate(const int baudRate) {mLogWarningLn("setBaudRate function not implemented"); return false;};

		/// Set return delay.
		/** Delay between transmission of Instruction Packet until the return of Status Packet.
		 * \param microsecondsReturnDelay Return delay in [us].
		 */
		virtual int		setReturnDelayTime(const int microsecondsReturnDelay) {mLogWarningLn("setReturnDelayTime function not implemented"); return false;};

		/// Set initial torque limit (EEPROM).
		/**
		 * This value is copied to ram during power-on.
		 * \param absMaxTorque Torque limi as fraction of maximum possible torque.
		 */
		virtual int		setInitialTorqueLimit(double absMaxTorque) {mLogWarningLn("setInitialTorqueLimit function not implemented"); return false;};

		/// Set operational torque limit (RAM).
		/** \param absMaxTorque Torque limit as fraction of maximum possible torque. */
		virtual int		setTorqueLimit(double absMaxTorque) {mLogWarningLn("setTorqueLimit function not implemented"); return false;};

		/// Set angle limits.
		/**
		 * \param lowerLimit Lower angle limit in [rad].
		 * \param upperLimit Upper angle limit in [rad].
		 */
		virtual int		setAngleLimits(double lowerLimit, double upperLimit) {mLogWarningLn("setAngleLimits function not implemented"); return false;};

		/// Set voltage limits.
		/**
		 * \param minVoltage Minimum voltage in [V].
		 * \param maxVoltage Maximum voltage in [V].
		 */
		virtual int		setVoltageLimits(double minVoltage, double maxVoltage) {mLogWarningLn("setVoltageLimits function not implemented"); return false;};

		/// Set temperature limit
		/** \param maxTemp Maximum temperature in [deg. C]. */
		virtual int		setTemperatureLimit(const int maxTemp) {mLogWarningLn("setTemperatureLimit function not implemented"); return false;};

		/// Set lower angle limit.
		/**
		 * \param limit Lower angle limit in [rad].
		 */
		virtual int		setAngleLowerLimit(double limit) {mLogWarningLn("setAngleLowerLimit function not implemented"); return false;};						// In [rad].

		/// Set upper angle limit.
		/**
		 * \param limit Upper angle limit in [rad].
		 */
		virtual int		setAngleUpperLimit(double limit){mLogWarningLn("setAngleUpperLimit function not implemented"); return false;};						// In [rad].

		/// Set control flexibility.
		/**
		 * \warning Uses Dynamixel internal units.
		 * \param complianceMargin Maximum error between goal position and present position in Dynamixel units.
		 * \param complianceSlope Level of Torque near the goal position. Used value = \c 2^complianceSlope.
		 */
		virtual int		setCompliance(BYTE complianceMargin, BYTE complianceSlope){mLogWarningLn("setCompliance function not implemented"); return false;};

		/// Unused
		/** \param punch Unused. */
		virtual int		setPunch(WORD punch){mLogWarningLn("setPunch function not implemented"); return false;};

		/// Set which conditions cause the alarm LED to blink.
		/** mask Condition mask. See Dynamixel documentation. */
		virtual int		setAlarmLEDMask(const BYTE mask){mLogWarningLn("setAlarmLEDMask function not implemented"); return false;};

		/// Undocumented
		/** mode Undocumented. */
		virtual int		setOperatingMode(const BYTE mode){mLogWarningLn("setOperatingMode function not implemented"); return false;};

		/// Set which conditions trigger a shutdown.
		/** mask Condition mask. See Dynamixel documentation. */
		virtual int		setAlarmShutdownMask(const BYTE mask){mLogWarningLn("setAlarmShutdownMask function not implemented"); return false;};

		/// Change hardware motor ID.
		/** Actually sets the new ID in the hardware.
		 * \param newID New ID.
		 */
		virtual int		changeID(const int newID){mLogWarningLn("changeID function not implemented"); return false;};

		/// Enable LED.
		/** \param state \c 0 to extinguish the LED, \c 1 to light it. */
		virtual int		enableLED(int state){mLogWarningLn("enableLED function not implemented"); return false;};

		/// Enable torque.
		/** \param state \c 0 to disable power, \c 1 to enable it. */
		virtual int		enableTorque(int state){mLogWarningLn("enableTorque function not implemented"); return false;};

		/// Endless turn mode ('wheel mode').
		/**
		 * Calling setEndlessTurnMode(false) will restore angle limits.
		 * \warning In this mode, there is no speed control! For safety, before enabling this mode, set the endlessTurnTorque() to zero first to avoid unwanted movements!
		 * \param enabled State.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setEndlessTurnMode(bool enabled, bool shouldSyncWrite=false){mLogWarningLn("setEndlessTurnMode function not implemented"); return false;};

		/// Torque to apply in endless turn mode.
		/**
		 * \param torqueRatio Torque as fraction of maximum possible torque.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 */
		virtual int		setEndlessTurnTorque(double torqueRatio, bool shouldSyncWrite=false){mLogWarningLn("setEndlessTurnTorque function not implemented"); return false;};

		/// Set behaviour of the 3mxl
		/**
		 * \warning Stops the motor.
		 * \param mxlMode Control mode.
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * \see \subpage 3mxlmodes.
		 * */
		virtual int		set3MxlMode(BYTE mxlMode, bool shouldSyncWrite = false){mLogWarningLn("set3MxlMode function not implemented"); return false;};

		/// Set motor direction convention
		/** \param clockwiseIsPositive True if clockwise direction is positive */
		virtual int		setPositiveDirectionMotor(bool clockwiseIsPositive){mLogWarningLn("setPositiveDirectionMotor function not implemented"); return false;};

		/// Set joint direction convention
		/** \param clockwiseIsPositive True if clockwise direction is positive */
		virtual int		setPositiveDirectionJoint(bool clockwiseIsPositive){mLogWarningLn("setPositiveDirectionJoint function not implemented"); return false;};

		/// Set motor null angle offset
		/** \param offset Offset in [rad]. */
		virtual int		setMotorOffset(double offset){mLogWarningLn("setMotorOffset function not implemented"); return false;};

		/// Set joint null angle offset
		/** \param offset Offset in [rad]. */
		virtual int		setJointOffset(double offset){mLogWarningLn("setJointOffset function not implemented"); return false;};

		/// Motor encoder counts per revolution
		/** \param encodercount Counts per revolution */
		virtual int		setEncoderCountMotor(WORD encodercount){mLogWarningLn("setEncoderCountMotor function not implemented"); return false;};

		/// Joint encoder counts per revolution
		/** \param encodercount Counts per revolution */
		virtual int		setEncoderCountJoint(WORD encodercount){mLogWarningLn("setEncoderCountJoint function not implemented"); return false;};

		/// Set motor constant
		/** \param motorconstant Motor constant in [10^-4 Nm/A]. */
		virtual int		setMotorConstant(WORD motorconstant){mLogWarningLn("setMotorConstant function not implemented"); return false;};
		
		/// Set maximum motor peak current
		/** \param current Maxmimum peak current in [A]. */
		virtual int		setMaxPeakMotorCurrent(double current){mLogWarningLn("setMaxMotorPeakCurrent function not implemented"); return false;};
		
		/// Set maximum motor continuous current
		/** \param current Maxmimum continuous current in [A]. */
		virtual int		setMaxContinuousMotorCurrent(double current){mLogWarningLn("setMaxMotorContinuousCurrent function not implemented"); return false;};
		
		/// Set motor winding time constant
		/** \param time Motor winding temperature time constant in [s]. */
		virtual int		setMotorWindingTimeConstant(double time){mLogWarningLn("setMotorWindingTimeConstant function not implemented"); return false;};
		
		/// Set motor encoder index pulse signal level
		/** \param level Low (0) or high (1) index pulse signal. */
		virtual int		setEncoderIndexLevelMotor(BYTE level){mLogWarningLn("setEncoderIndexLevelMotor function not implemented"); return false;};
		
		/// Set wheel diameter
		/** \param diameter Wheel diameter in [m]. */
		virtual int		setWheelDiameter(double diameter){mLogWarningLn("setWheelDiameter function not implemented"); return false; };

		/// Ratio between motor and joint
		/** \param gearboxratiomotor turns of motor for one joint revolution. */
		virtual int		setGearboxRatioMotor(float gearboxratiomotor){mLogWarningLn("setGearboxRatioMotor function not implemented"); return false;};

		/// Ratio between joint and joint encoder
		/** \param gearboxratiojoint turns of encoder for one joint revolution. */
		virtual int		setGearboxRatioJoint(float gearboxratiojoint){mLogWarningLn("setGearboxRatioJoint function not implemented"); return false;};

		/// What to do if watchdog triggers
		/** \param watchdogmode Unused */
		virtual int		setWatchdogMode(BYTE watchdogmode){mLogWarningLn("setWatchdogMode function not implemented"); return false;};

		/// Set watchdog interval (time)
		/** \param watchdogtime Watchdog interval = \c time * \c multiplier. */
		virtual int		setWatchdogTime(BYTE watchdogtime){mLogWarningLn("setWatchdogTime function not implemented"); return false;};

		/// Set watchdog interval (multiplier)
		/** \param watchdogmultiplier Watchdog interval = \c time * \c multiplier. */
		virtual int		setWatchdogMultiplier(BYTE watchdogmultiplier){mLogWarningLn("setWatchdogMultiplier function not implemented"); return false;};

		/// Unused
		/** \param maxcurrent Unused */
		virtual int		setMaxUninitializedMotorCurrent(WORD maxcurrent){mLogWarningLn("setMaxUninitializedMotorCurrent function not implemented"); return false;};

		/// Unused
		/** \param maxcurrent Unused */
		virtual int		setMaxMotorCurrent(WORD maxcurrent){mLogWarningLn("setMaxMotorCurrent function not implemented"); return false;};

		/// Set zero length of the spring for series elastic actuation
		/** \param parameterInRad Zero length in [rad]. */
		virtual int		setZeroLengthSpring(double parameterInRad){mLogWarningLn("setZeroLengthSpring function not implemented"); return false;};

		/// Set spring stiffness for series elastic actuation
		/** \param stiffness Spring stiffness in [Nm/rad] */
		virtual int		setSpringStiffness(double stiffness){mLogWarningLn("setSpringStiffness function not implemented"); return false;};
		
		/// Set reference energy for energy control
		/** \param energy Reference energy in [J] */
		virtual int		setReferenceEnergy(double energy){mLogWarningLn("setReferenceEnergy function not implemented"); return false;};
		
		/// Set sine frequency
		/**
		 * \note Starts the motor.
		 * \param frequency Frequency in [Hz]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * */
		virtual int		setSineFrequency(double frequency, bool shouldSyncWrite=false)  {mLogWarningLn("setSineFrequency function not implemented"); return false;};
		
		/// Set sine amplitude
		/**
		 * \warning Does not start the motor. Use setSineFrequency for that.
		 * \param amplitude Amplitude in [rad]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * \see setSineFrequency
		 * */
		virtual int		setSineAmplitude(double amplitude, bool shouldSyncWrite=false)  {mLogWarningLn("setSineAmplitude function not implemented"); return false;};
		
		/// Set sine phase angle
		/**
		 * \warning Does not start the motor. Use setSineFrequency for that.
		 * \param phase Phase in [rad]
		 * \param shouldSyncWrite Accumulate data instead of sending immediately.
		 * \see setSineFrequency
		 * */
		virtual int		setSinePhase(double phase, bool shouldSyncWrite=false)  {mLogWarningLn("setSinePhase function not implemented"); return false;};
		
		/// Set logging interval
		/**
		 * \note Call this to make sure you're logging for the right motor.
		 * \param interval logging interval in [ms].
		 */
		virtual int		setLogInterval(BYTE interval){mLogWarningLn("startLog function not implemented"); return false; };

		/// Set index in sync read chain
		/**
		 * \note Normally called by CDxlGroup.
		 * \param index index. 0 means this motor does not participate in the chain.
		 */
		virtual int		setSyncReadIndex(BYTE index){mLogWarningLn("setSyncReadIndex function not implemented"); return false; };

		/// Read position from hardware.
		/** \see presentPos(). */
		virtual int		getPos(){mLogWarningLn("getPos function not implemented"); return false;};

		/// Read linear position from hardware.
		/** \see presentLinearPos(). */
		virtual int		getLinearPos(){mLogWarningLn("getLinearPos function not implemented"); return false;};

		/// Read position and speed from hardware.
		/** \see presentPos(), presentSpeed(). */
		virtual int		getPosAndSpeed(){mLogWarningLn("getPosAndSpeed function not implemented"); return false;};

		/// Read temperature from hardware.
		/** \see presentTemp(). */
		virtual int		getTemp(){mLogWarningLn("getTemp function not implemented"); return false;};

		/// Read entire state from hardware.
		/** \see presentPos(), presentSpeed(), presentLoad(), presentVoltage(), presentTemp(). */
		virtual int		getState(){mLogWarningLn("getState function not implemented"); return false;};

		/// Read angle limits from hardware.
		/**
		 * \warning May not be called in endless turn mode!
		 * \see presentCWAngleLimit() presentCCWAngleLimit() presentAngleLowerLimit() presentAngleUpperLimit().
		 */
		virtual int		getAngleLimits(){mLogWarningLn("getAngleLimits function not implemented"); return false;};

		/// Read motor voltage from hardware.
		/** \see presentVoltage(). */
		virtual int		getVoltage(){mLogWarningLn("getVoltage function not implemented"); return false;};

		/// Read bus voltage from hardware.
		/** \see presentBusVoltage(). */
		virtual int		getBusVoltage(){mLogWarningLn("getBusVoltage function not implemented"); return false;};

		/// Read analog sensor voltages from hardware.
		/** \see presentBusVoltage() presentCurrentADCVoltage() presentAnalog1Voltage() presentAnalog2Voltage(). */
		virtual int		getSensorVoltages(){mLogWarningLn("getSensorsVoltages function not implemented"); return false; };

		/// Read current from hardware.
		/** \see presentCurrent(). */
		virtual int		getCurrent(){mLogWarningLn("getCurrent function not implemented"); return false;};

		/// Read torque from hardware.
		/** \see presentTorque(). */
		virtual int		getTorque(){mLogWarningLn("getTorque function not implemented"); return false;};

		/// Read torque, position and speed from hardware.
		/** \see presentTorque() presentPos() presentSpeed(). */
		virtual int		getTorquePosSpeed(){mLogWarningLn("getTorquePosSpeed function not implemented"); return false;};
		
		/// Read acceleration from hardware.
		/** \see presentAcceleration(). */
		virtual int		getAcceleration(){mLogWarningLn("getAcceleration function not implemented"); return false;};
		
		/// Read linear acceleration from hardware.
		/** \see presentLinearAcceleration(). */		
		virtual int		getLinearAcceleration(){mLogWarningLn("getLinearAcceleration function not implemented"); return false;};
		
		/// Read status from hardware.
		virtual int		getStatus(){mLogWarningLn("getStatus function not implemented"); return false;};
		
		/// Read logfile from hardware.
		/** \read logfile from hardware to memory. Get a pointer to this array with presentLog(). */
		virtual int		getLog(){mLogWarningLn("getLog function not implemented"); return false;};

		/// Read behavioral mode of the 3mxl.
		virtual int		get3MxlMode(){mLogWarningLn("get3MxlMode function not implemented"); return false;};
		
		/// Get cached position.
		/** \returns Position in [rad]. */
		virtual double	presentPos()=0;

		/// Get cached linear position.
		/** \returns Position in [m]. */
		virtual double	presentLinearPos()=0;

		/// Get cached speed.
		/** \returns Speed in [rad/s]. */
		virtual double	presentSpeed()=0;

		/// Get cached acceleration.
		/** \returns Acceleration in [rad/s^2]. */
		virtual double  presentAcceleration()=0;
		
		/// Get cached linear acceleration.
		/** \returns Acceleration in [m/s^2]. */
		virtual double  presentLinearAcceleration()=0;
		
		/// Get cached load value.
		/** \returns Load in [Nm]. */
		virtual double	presentLoad()=0;

		/// Get cached motor voltage.
		/** \returns Motor voltage in [V]. */
		virtual double	presentVoltage()=0;

		/// Get cached bus voltage.
		/** \returns Bus voltage in [V]. */
		virtual double	presentBusVoltage()=0;

		/// Get cached motor current ADC voltage.
		/** \returns ADC voltage in [V]. */
		virtual double	presentCurrentADCVoltage()=0;

		/// Get cached analog sensor 1 voltage.
		/** \returns Sensor voltage in [V]. */
		virtual double	presentAnalog1Voltage()=0;

		/// Get cached analog sensor 2 voltage.
		/** \returns Sensor voltage in [V]. */
		virtual double	presentAnalog2Voltage()=0;

		/// Get cached analog sensor 3 voltage.
		/** \returns Sensor voltage in [V]. */
		virtual double	presentAnalog3Voltage()=0;

		/// Get cached analog sensor 4 voltage.
		/** \returns Sensor voltage in [V]. */
		virtual double	presentAnalog4Voltage()=0;

		/// Get cached temperature.
		/** \returns Temperature in [deg. C]. */
		virtual double	presentTemp()=0;

		/// Get cached current.
		/** \returns Current in [A]. */
		virtual double	presentCurrent()=0;

		/// Get cached torque.
		/** \returns Torque in [Nm]. */
		virtual double	presentTorque()=0;

		/// Get cached status.
		/**
		 * \returns Status.
		 * \see \subpage 3mxlstatus.
		 */
		virtual int 	presentStatus(){mLogWarningLn("presentStatus function not implemented"); return 0;};

		/// Get motor initialization state.
		/** \returns Whether the motor executed the initialization procedure. */
		virtual bool	presentMotorInitState() {mLogWarningLn("presentMotorInitState function not implemented"); return 0;};

		/// Get cached clockwise angle limit.
		/**
		 * \warning Ignores the motor's null-angle and direction.
		 * \returns Raw clockwise angle limit limit.
		 */
		virtual WORD	presentCWAngleLimit()=0;

		/// Get cached counter-clockwise angle limit.
		/**
		 * \warning Ignores the motor's null-angle and direction.
		 * \returns Raw counter-clockwise angle limit limit.
		 */
		virtual WORD	presentCCWAngleLimit()=0;

		/// Get cached lower angle limit.
		/** \returns Lower angle limit in [rad]. */
		virtual double	presentAngleLowerLimit()=0;

		/// Get cached upper angle limit.
		/** \returns Upper angle limit in [rad]. */
		virtual double	presentAngleUpperLimit()=0;

		/// Get cached log string/file.
		/** \returns a vector of log entries. */
		virtual TMxlLog presentLog(){mLogWarningLn("presentLog function not implemented"); return TMxlLog();};

		/// Get behavioral mode of the 3mxl.
		/** \see \subpage 3mxlmodes. */
		virtual BYTE	present3MxlMode(){mLogWarningLn("present3MxlMode function not implemented"); return 0;};

		/// Report on this Dynamixel.
		/** Reads out the motor's complete control table.
		 * \param fOut Output file.
		 */
		virtual int	printReport(FILE* fOut){mLogWarningLn("printReport function not implemented"); return false;};

		/// Interpret data read from control table
		/**
		 * \param address starting address.
		 * \param length number of bytes read.
		 * \param data control data.
		 */
		virtual int	interpretControlData(BYTE address, BYTE length, BYTE *data){mLogWarningLn("interpretControlData function not implemented"); return false;};
};


#endif //__DXLGENERIC_H_INCLUDED__
