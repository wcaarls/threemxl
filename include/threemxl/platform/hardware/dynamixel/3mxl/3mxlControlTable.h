
#ifndef __3MXLCONTROLTABLE_H_INCLUDED__
#define __3MXLCONTROLTABLE_H_INCLUDED__

/*************************************************************************
Organisation	:	Delft University Of Technology
					Biomechanical Engineering
					Dutch Biorobotics Lab

Project			: 3MXL

Author			: Dries Hulens / Eelko van Breda / Guus Liqui Lung / Daniel Karssen

initial Date	: 09 March 2010
			  	  02 December 2010
			  	  28 January 2010

current Version	: 2.1

Filename :		: 3mxlControlTable.h

Description		: contains constants for the controltable used in the
					Dynamixelprotocol

*************************************************************************/

// original Dynamixel command instructions:
#define NO_INSTRUCTION							0x00
#define PING 									0x01
#define READ_DATA 								0x02
#define WRITE_DATA 								0x03
#define REG_WRITE 								0x04
#define ACTION 									0x05
#define RESET 									0x06
#define SYNC_WRITE 								0x83

// instructions added by GLL for 3Mxel, not fully implemented yet
#define WRITE_MOTOR_PARAMS						0xA0	// motor constant, gearboxratio etc
#define WRITE_JOINT_PARAMS						0xA1	// encoder resol, spring etc
#define WRITE_SEA_SETPOINTS						0xA2	// PD gains position control mode
#define SET_JOINT_TYPE							0xA3	// left/right hip/knee

// broadcast ID to address multiple 3Mxels
#define BROADCAST_ID							0xFE

// packet types
#define NORMAL_PACKET_TYPE						0x00
#define BROADCAST_PACKET_TYPE					0x01

/** \defgroup 3mxlmodes 3mxl control modes for address M3XL_CONTROL_MODE
 * @{ 
 */ 
#define POSITION_MODE							0	// uses one motor and encoder with position controller
#define SPEED_MODE								1	// uses one motor and encoder with speed controller
#define CURRENT_MODE							2	// uses one motor and encoder with current controller
#define TORQUE_MODE								3	// uses one motor and encoder with torque controller
#define SEA_MODE								4	// uses one motor and two encoders with SEA controller
#define PWM_MODE								5	// uses one motor and encoder with PWM controller, no PID, just PWM !!

#define INDEX_INIT								6	// reset counter on index pulse, turn motor with given torque in M3XL_DESIRED_TORQUE_L
#define EXTERNAL_INIT							7	// reset counter on external io pulse, turn motor with given torque in M3XL_DESIRED_TORQUE_L
#define ZERO_SPEED_INIT							8	// reset counter when jointspeed reaches zero (endstop), turn motor with given torque in
													//	M3XL_DESIRED_TORQUE_L
#define SEA_INIT								9	// reset counter of joint on index pulse and reset counter of motor on the next motor pulse.
													//	Turn motor with given torque in M3XL_DESIRED_TORQUE_L
#define MANUAL_INIT								10	// reset counter of joint on index pulse and reset counter of motor on the next motor pulse.
													//	manually rotate motor/joint
#define TIME_OUT_INIT							11	// reset counter of joint after a timeou value. Turn motor with given torque in M3XL_DESIRED_TORQUE_L
#define STOP_MODE								12	// do nothing, no actuated motors
#define HOME_SWITCH_AND_INDEX_INIT				13	// move to external home switch and reset counter on index in SPEED_MODE
#define CURRENT_POS_AND_SPEED_MODE				14	// uses current control based on position and motor torque
#define START_UP_MODE							15
#define SINUSOIDAL_POSITION_MODE				16
#define TEST_MODE								17	// for general testing

#define NR_OF_CONTROL_MODES						TORQUE_MODE+1
/** @} */ 

// Standard Dynamixel status errorcode bits
// They are reurned as standard dynamixel ERRR_CODE in a status packet
// and are now stored at  M3XL_DONE  = 0xA5
#define M3XL_NO_ERROR							0b00000000
#define M3XL_INSTRUCTION_ERROR					0b01000000
#define M3XL_OVERLOAD_ERROR						0b00100000
#define M3XL_CHECKSUM_ERROR						0b00010000
#define M3XL_RANGE_ERROR						0b00001000
#define M3XL_OVERHEATING_ERROR					0b00000100
#define M3XL_ANGLE_LIMIT_ERROR					0b00000010
#define M3XL_INPUT_VOLTAGE_ERROR				0b00000001

/** \defgroup 3mxlstatus 3mxl status codes for address M3XL_STATUS
 * 0x80 to 0x89 are for error states. 
 * 0x90 and higher are for non-error states. 
 * @{
 */   
#define M3XL_STATUS_EEPROM_ERROR				0x80
#define M3XL_STATUS_NOT_INITIALIZED				0x81
#define M3XL_STATUS_EM_STOP_ERROR				0x82
#define M3XL_STATUS_INIT_TIME_OUT_ERROR			0x83
#define M3XL_STATUS_MAX_POS_ERROR				0x84
#define M3XL_STATUS_MAX_TORQUE_ERROR			0x85
#define M3XL_STATUS_MAX_CURRENT_ERROR			0x86
#define M3XL_STATUS_MOTOR_STUCK_ERROR			0x87
#define M3XL_STATUS_JOINT_STUCK_ERROR			0x88
#define M3XL_STATUS_PROTOCOL_TIME_OUT_ERROR		0x89

#define M3XL_STATUS_MOVING						0X90
#define M3XL_STATUS_MOVE_DONE					0X91
#define M3XL_STATUS_INITIALIZE_BUSY				0X92
#define M3XL_STATUS_INIT_DONE					0x93
#define M3XL_STATUS_POS_MODE_EXECUTING			0x94
#define M3XL_STATUS_POS_MODE_DONE				0x95
#define M3XL_STATUS_SPEED_MODE_EXECUTING		0x96
#define M3XL_STATUS_SPEED_MODE_DONE				0x97
#define M3XL_STATUS_TORQUE_MODE_EXECUTING		0x98
#define M3XL_STATUS_TORQUE_MODE_DONE			0x99
#define M3XL_STATUS_CURRENT_MODE_EXECUTING		0x9A
#define M3XL_STATUS_CURRENT_MODE_DONE			0x9B
#define M3XL_STATUS_SEA_MODE_EXECUTING			0x9C
#define M3XL_STATUS_SEA_MODE_DONE				0x9D
#define M3XL_STATUS_PWM_MODE_EXECUTING			0x9E
#define M3XL_STATUS_PWM_MODE_DONE				0x9F
#define M3XL_STATUS_SINUSOIDAL_POS_MODE_EXECUTING     0xA0
#define M3XL_STATUS_SINUSOIDAL_POS_MODE_DONE          0xA1
#define M3XL_STATUS_IDLE_STATE					0xA0
/** @} */ 

// used for signaling when action is in progress
#define M3XL_ACTION_BUSY						0x00
#define M3XL_ACTION_DONE						0xFF

// default IDs for address M3XL_ID
#define DEFAULT_3MXL_ID_1						0x64
#define DEFAULT_3MXL_ID_2						0x65

// status return level Control table address M3XL_STATUS_RETURN_LEVEL
#define M3XL_STATUS_RETURN_NONE					0x00
#define M3XL_STATUS_RETURN_READ_DATA			0x01
#define M3XL_STATUS_RETURN_ALL					0x02	//Same as dynamixel

// 3Mxel joints types for address M3XL_JOINT_TYPE_H
#define JOINT_TYPE_WHEEL						0x01
#define JOINT_TYPE_ARM							0x02
#define JOINT_TYPE_GRIPPER						0x03
#define JOINT_TYPE_DIRECTDRIVE					0x04
#define JOINT_TYPE_SEA							0x05
#define JOINT_TYPE_LINEAR						0x06
#define JOINT_TYPE_PASSIVE						0x07
#define JOINT_TYPE_UNKNOWN						0x0A

// maximum allowable motorcurrent, determined by H-bridge
#ifdef M3XL_MINI
	#define M3XL_MAX_MOTOR_CURRENT					2.8
#else
	#define M3XL_MAX_MOTOR_CURRENT					40.0
#endif

#define M_PI									3.14159265358979323846

//***************************************************************************
// HERE ARE THE ADRESSES OF THE ELEMENTS IN THE TABLE
//***************************************************************************

#define M3XL_JOINT_TYPE_L						0x00
#define M3XL_JOINT_TYPE_H						0x01 // The high byte of M3XL_JOINT_TYPE is now used to store the actual jointtype
#define M3XL_VERSION_FIRMWARE					0x02
#define M3XL_ID									0x03
#define M3XL_STATUS_RETURN_LEVEL				0x10    //Same as dynamixel

//// <Original dynamixel table>
//#define BAUD_RATE_L							0x04
//#define RETURN_DELAY_TIME						0x05
//#define CW_ANGLE_LIMIT_L						0x06
//#define CW_ANGLE_LIMIT_H						0x07
//#define CCW_ANGLE_LIMIT_L						0x08
//#define CCW_ANGLE_LIMIT_H						0x09
//
//#define LIMIT_TEMP_HIGH						0x0B
//#define LIMIT_VOLTAGE_LOW						0x0C
//#define LIMIT_VOLTAGE_HIGH					0x0D
//#define MAX_TORQUE_L							0x0E
//#define MAX_TORQUE_H							0x0F
//#define STATUS_RETURN_LEVEL					0x10
//#define ALARM_LED								0x11
//#define ALARM_SHUTDOWN						0x12
//
//#define TORQUE_ENABLE							0x18
//#define LED									0x19
//#define CW_COMPILANCE_MARGIN					0x1A
//#define CCW_COMPILANCE_MARGIN					0x1B
//#define CW_COMPILANCE_SLOPE					0x1C
//#define CCW_COMPILANCE_SLOPE					0x1D
//#define GOAL_POSITION_L						0x1E
//#define GOAL_POSITION_H						0x1F
//#define MOVING_SPEED_L						0x20
//#define MOVING_SPEED_H						0x21
//#define TORQUE_LIMIT_L						0x22
//#define TORQUE_LIMIT_H						0x23
//#define PRESENT_POSITION_L					0x24
//#define PRESENT_POSITION_H					0x25
//#define PRESENT_SPEED_L						0x26
//#define PRESENT_SPEED_H						0x27
//#define PRESENT_LOAD_L						0x28
//#define PRESENT_LOAD_H						0x29
//#define PRESENT_VOLTAGE						0x2A
//#define PRESENT_TEMP							0x2B
//#define REGISTERED_INSTRUCTION				0x2C
//
//#define MOVING								0x2E
//#define MXL_LOCK								0x2F
//#define PUNCH_L								0x30
//#define PUNCH_H								0x31
//// </Original dynamixel table>

///////////////  <3mxl table only valid in 3mxl mode!> ///////////////////////
#define M3XL_BAUD_RATE_L						0x32    //Baudrate in baud
#define M3XL_BAUD_RATE_M						0x33
#define M3XL_BAUD_RATE_H						0x34
#define M3XL_RETURN_DELAY_TIME					0x35    //In dynamixel dimension
#define M3XL_CONTROL_MODE						0x36    //Describes behaviour of the 3mxl

//watchdog resets on arrival of new personal packet or broadcast (Not yet implemented!)
#define M3XL_WATCHDOG_MODE						0x37    //what to do of watchdog triggers
#define M3XL_WATCHDOG_TIME_MS					0x38    //time setting for watchdog = ms x multiplier
#define M3XL_WATCHDOG_TIMER_MUL					0x39

//--- <Direct drive and SEA mode parameters>
#define M3XL_MOTOR_CONSTANT_L					0x3A    //dimension 10^-4*Nm/A
#define M3XL_MOTOR_CONSTANT_H					0x3B
#define M3XL_MAX_CONTINUOUS_MOTOR_CURRENT_L		0x3C	// MAX continuous Current in 10mA
#define M3XL_MAX_CONTINUOUS_MOTOR_CURRENT_H		0x3D
#define M3XL_MAX_MOTOR_PEAK_CURRENT_L			0x3E    // stall Current in 10mA
#define M3XL_MAX_MOTOR_PEAK_CURRENT_H			0x3F
#define M3XL_MOTOR_WINDING_TIME_CONSTANT_L		0x40	// motorwinding temperature time constant in 0.01 sec
#define M3XL_MOTOR_WINDING_TIME_CONSTANT_H		0x41
#define M3XL_GEARBOX_RATIO_MOTOR_L				0x42    // ratio between motor and joint (turns of motor rev for one joint revolution), times 10
#define M3XL_GEARBOX_RATIO_MOTOR_H				0x43
#define M3XL_ENCODER_COUNT_MOTOR_L				0x44    // encoder counts quadrature
#define M3XL_ENCODER_COUNT_MOTOR_H				0x45
#define M3XL_OFFSET_MOTOR_L						0X46    // zero position of the motor in mRad
#define M3XL_OFFSET_MOTOR_H						0X47
#define M3XL_MOTOR_ENC_DIRECTION				0x48    // clockwise is positive {0,1}
#define M3XL_MOTOR_ENC_INDEX_LEVEL				0x49    // index at LOW AB or HIGH AB pulses
#define M3XL_CW_MOTOR_ANGLE_LIMIT_L				0x4A    // RW Position in 1 mRad
#define M3XL_CW_MOTOR_ANGLE_LIMIT_H				0x4B
#define M3XL_CCW_MOTOR_ANGLE_LIMIT_L			0x4C    // RW Position in 1 mRad
#define M3XL_CCW_MOTOR_ANGLE_LIMIT_H			0x4D
//--- </Direct drive and SEA mode>

//--- <Series Elastic Mode parameters> (normally not used in direct drive mode)
#define M3XL_GEARBOX_RATIO_JOINT_L				0x4E    // ratio between encoder and joint (turns of encoder rev for one joint revolution), times 10
#define M3XL_GEARBOX_RATIO_JOINT_H				0x4F
#define M3XL_ENCODER_COUNT_JOINT_L				0x50    // encoder counts quadrature
#define M3XL_ENCODER_COUNT_JOINT_H				0x51
#define M3XL_OFFSET_JOINT_L						0x52    // zero position of the joint in mRad
#define M3XL_OFFSET_JOINT_H						0x53
#define M3XL_MAX_JOINT_TORQUE_L					0x54    // RW Torque in 0,001 Nm (mNm)
#define M3XL_MAX_JOINT_TORQUE_H					0x55
#define M3XL_CW_JOINT_ANGLE_LIMIT_L				0x56    // RW Position in 1 mRad
#define M3XL_CW_JOINT_ANGLE_LIMIT_H				0x57
#define M3XL_CCW_JOINT_ANGLE_LIMIT_L			0x58    // RW Position in 1 mRad
#define M3XL_CCW_JOINT_ANGLE_LIMIT_H			0x59
#define M3XL_ZERO_LENGTH_SPRING_L				0x5A    // TODO:dimension?
#define M3XL_ZERO_LENGTH_SPRING_H				0x5B
#define M3XL_SPRING_STIFFNESS_L					0x5C    // cNm/rad
#define M3XL_SPRING_STIFFNESS_H					0x5D
#define M3XL_JOINT_ENC_DIRECTION				0x5E    // clockwiseispositive {0,1}
#define M3XL_JOINT_ENC_INDEX_LEVEL				0x5F    // index at LOW AB or HIGH AB pulses
//--- </Series Elastic Mode>

//--- <Status information>
#define M3XL_VOLTAGE_L							0x60    // RO Voltage in 10mV
#define M3XL_VOLTAGE_H							0x61
#define M3XL_CURRENT_L							0x62    // RO Current in 10mA
#define M3XL_CURRENT_H							0x63
#define M3XL_TORQUE_L							0x64    // RO Torque in 0,001 Nm (mNm)
#define M3XL_TORQUE_H							0x65
#define M3XL_ANGLE_L							0x66    // RO Angle in 1 mRad
#define M3XL_ANGLE_H							0x67
#define M3XL_ANGULAR_RATE_L                     0x68	// RO Angular Speed in 10mRad/s (cRad)/s
#define M3XL_ANGULAR_RATE_H                     0x69
#define M3XL_POSITION_32_1						0x6A     // 32 bits actual position in mm
#define M3XL_POSITION_32_2						0x6B
#define M3XL_POSITION_32_3						0x6C
#define M3XL_POSITION_32_4						0x6D
#define M3XL_SPEED_L							0x6E  	// speed in 0.1 mm/s
#define M3XL_SPEED_H							0x6F
#define M3XL_LINEAR_SPEED_L						M3XL_SPEED_L  	// speed in mm/s
#define M3XL_LINEAR_SPEED_H						M3XL_SPEED_H
//--- </Status information>

//--- <Setpoints for PID>
// the next 3 parameters MotorCurrent, Pgain and Dgain can be sent in one syncwrite packet
#define M3XL_DESIRED_CURRENT_L					0x70    // RW Current in 10mA
#define M3XL_DESIRED_CURRENT_H					0x71

// the next 5 parameters can be sent in one syncwrite packet
#define M3XL_P_CURRENT_L						0x72	// when start writing at this address
#define M3XL_P_CURRENT_H						0x73	// all PID CURRENT_MODE params should be sent, 10 bytes
#define M3XL_D_CURRENT_L						0x74
#define M3XL_D_CURRENT_H						0x75
#define M3XL_I_CURRENT_L						0x76
#define M3XL_I_CURRENT_H						0x77
#define M3XL_IL_CURRENT_L						0x78
#define M3XL_IL_CURRENT_H						0x79
#define M3XL_PID_CURRENT_SCALE_L				0x7A
#define M3XL_PID_CURRENT_SCALE_H				0x7B

#define M3XL_DESIRED_POSITION_32_1				0x7C   // RW 32 bits position in mm
#define M3XL_DESIRED_POSITION_32_2				0x7D
#define M3XL_DESIRED_POSITION_32_3				0x7E
#define M3XL_DESIRED_POSITION_32_4				0x7F

#define M3XL_DESIRED_ACCEL_L					0x80    // RW Speed in 10mRad/s (cRad)/s
#define M3XL_DESIRED_ACCEL_H					0x81
#define M3XL_DESIRED_ANGULAR_ACCEL_L			M3XL_DESIRED_ACCEL_L   // RW Speed in 10mRad/s (cRad)/s
#define M3XL_DESIRED_ANGULAR_ACCEL_H			M3XL_DESIRED_ACCEL_H

// the next 5 parameters Angle, Speed, Torque, Pgain and Dgain can be sent in one syncwrite packet
#define M3XL_DESIRED_ANGLE_L					0x82    // RW Position in 1 mRad
#define M3XL_DESIRED_ANGLE_H					0x83

// entry names changed, old ones kept for compatibility
#define M3XL_DESIRED_SPEED_L					0x84    // RW Speed in 10mRad/s (cRad)/s
#define M3XL_DESIRED_SPEED_H					0x85
#define M3XL_DESIRED_ANGULAR_RATE_L				M3XL_DESIRED_SPEED_L   // RW Speed in 10mRad/s (cRad)/s
#define M3XL_DESIRED_ANGULAR_RATE_H				M3XL_DESIRED_SPEED_H

#define M3XL_DESIRED_TORQUE_L					0x86    // RW Torque in 0,001 Nm (mNm)
#define M3XL_DESIRED_TORQUE_H					0x87

// the next 5 PID parameters can be sent in one syncwrite packet
#define M3XL_P_POSITION_L						0x88  	// when start writing at this address
#define M3XL_P_POSITION_H						0x89	// all PID POSITION_MODE params should be sent, 10 bytes
#define M3XL_D_POSITION_L						0x8A
#define M3XL_D_POSITION_H						0x8B
#define M3XL_I_POSITION_L						0x8C
#define M3XL_I_POSITION_H						0x8D
#define M3XL_IL_POSITION_L						0x8E
#define M3XL_IL_POSITION_H						0x8F
#define M3XL_PID_POSITION_SCALE_L				0x90
#define M3XL_PID_POSITION_SCALE_H				0x91

// the next 5 PID parameters can be sent in one syncwrite packet
#define M3XL_P_SPEED_L							0x92	// when start writing at this address
#define M3XL_P_SPEED_H							0x93	// all PID SPEED_MODE params should be sent, 10 bytes
#define M3XL_D_SPEED_L							0x94
#define M3XL_D_SPEED_H							0x95
#define M3XL_I_SPEED_L							0x96
#define M3XL_I_SPEED_H							0x97
#define M3XL_IL_SPEED_L							0x98
#define M3XL_IL_SPEED_H							0x99
#define M3XL_PID_SPEED_SCALE_L					0x9A
#define M3XL_PID_SPEED_SCALE_H					0x9B

// the next 5 PID parameters can be sent in one syncwrite packet
#define M3XL_P_TORQUE_L							0x9C	// when start writing at this address
#define M3XL_P_TORQUE_H							0x9D	// all PID TORQUE_MODE params should be sent, 10 bytes
#define M3XL_D_TORQUE_L							0x9E
#define M3XL_D_TORQUE_H							0x9F
#define M3XL_I_TORQUE_L							0xA0
#define M3XL_I_TORQUE_H							0xA1
#define M3XL_IL_TORQUE_L						0xA2
#define M3XL_IL_TORQUE_H						0xA3
#define M3XL_PID_TORQUE_SCALE_L					0xA4
#define M3XL_PID_TORQUE_SCALE_H					0xA5

#define M3XL_DESIRED_PWM_L						0xA6
#define M3XL_DESIRED_PWM_H						0xA7
//--- </Setpoints for PID>

#define M3XL_STATUS                           	0xA8
#define M3XL_INITIALIZED                       	0xA9

#define M3XL_STOP_PROTOCOL_HANDLER              0xAA    // quit protocolmode and switch to terminal mode

// added for linear type of joints
#define M3XL_DESIRED_LINEAR_SPEED_L				0xAB	// speed in mm/s
#define M3XL_DESIRED_LINEAR_SPEED_H				0xAC	
#define M3XL_DESIRED_LINEAR_ACCEL_L				0xAD	// accel in mm/s2
#define M3XL_DESIRED_LINEAR_ACCEL_H				0xAE
#define M3XL_MAX_JERK_L                         0xAF	// jerk in mm/s3
#define M3XL_MAX_JERK_H                         0xB0

#define LAST_MESSAGE_ADDRESS                    0xB1    // local buffering for action after regwrite
#define LAST_MESSAGE_LENGTH                     0xB2

#define M3XL_WHEEL_DIAMETER_L                   0xB3	// wheeldiameter in mm
#define M3XL_WHEEL_DIAMETER_H                   0xB4

// next entries are for Underwater robot
#define M3XL_SINUSOIDAL_FREQUENCY_L				0xB5	// in 10mHz
#define M3XL_SINUSOIDAL_FREQUENCY_H				0xB6	
#define M3XL_SINUSOIDAL_AMPLITUDE_L             0xB7	// in mRad
#define M3XL_SINUSOIDAL_AMPLITUDE_H             0xB8
#define M3XL_SINUSOIDAL_PHASE_ANGLE_L                   0xB9    // in mRad
#define M3XL_SINUSOIDAL_PHASE_ANGLE_H                   0xBA

// next entries are used to verify the index position of the motor
#define M3XL_ACQUIRE_INDEX_POSITION             0xBA
#define M3XL_INDEX_POSITION_32_1				0xBB   // RW 32 bits index position in mm
#define M3XL_INDEX_POSITION_32_2				0xBC
#define M3XL_INDEX_POSITION_32_3				0xBD
#define M3XL_INDEX_POSITION_32_4				0xBE

#define M3XL_LOG_ARRAY_SIZE						500		// also declared as LOG_ARRAY_SIZE in SystemDefs.h
#define M3XL_NR_OF_BYTES_PER_SAMPLE	 			22		// the amount of bytes in one sample
#define M3XL_NR_OF_SAMPLES_PER_BLOCK	 		5		// the amount of samples in a block
#define M3XL_NR_OF_BLOCKS						(M3XL_LOG_ARRAY_SIZE / M3XL_NR_OF_SAMPLES_PER_BLOCK)		// the amount of blocks in a log
#define M3XL_NR_OF_BYTES_PER_BLOCK				(M3XL_NR_OF_SAMPLES_PER_BLOCK * M3XL_NR_OF_BYTES_PER_SAMPLE)

#define M3XL_ENABLE_DATA_LOGGER					0xBF	// setting this to 1 enables the datalogger for that motor.
#define M3XL_LOG_DATA_INTERVAL					0xC0	// loginterval in ms												
														// the entry other motor will be set to 0!
#define M3XL_DATA_LOGGER						0xC1	// reading from this location starts reporting data by datalogger
														// which data is logged depends on the control mode
														// data is split in blocks of 5 readings of 20 bytes
														//// used for outputting logged data
														//struct dataLoggerOutputType
														//{
														//	unsigned int sampleTime;	// 16 bits, 2 bytes
														//	float PWMDutyCycle;			// 32 bits, 4 bytes
														//	float motorCurrent;			// 32 bits, 4 bytes
														//	float vBus;					// 32 bits, 4 bytes
														//	float desiredValue;			// 32 bits, 4 bytes
														//	float actualValue; 			// 32 bits, 4 bytes
														//};
														// The value written to M3XL_DATA_LOGGER specifies the block number
														// e.g. : 1   : read block 1 samples 1-5
														// e.g. : 2   : read block 2 samples 6-10
														// e.g. : 100 : read block 100 samples 496-500
#define M3XL_BUS_VOLTAGE_L						0xC2    // RO Bus Voltage in 10mV
#define M3XL_BUS_VOLTAGE_H						0xC3
#define M3XL_MOTOR_CURRENT_L						0xC4    // Motorcurrent in cA
#define M3XL_MOTOR_CURRENT_H						0xC5
#define M3XL_ANA1_VOLTAGE_L						0xC6    // RO Analog Voltage in 10mV
#define M3XL_ANA1_VOLTAGE_H						0xC7
#define M3XL_ANA2_VOLTAGE_L						0xC8    // RO Analog Voltage in 10mV
#define M3XL_ANA2_VOLTAGE_H						0xC9
#define M3XL_ANA3_VOLTAGE_L						0xCA    // RO Analog Voltage in 10mV
#define M3XL_ANA3_VOLTAGE_H						0xCB
#define M3XL_ANA4_VOLTAGE_L						0xCC    // RO Analog Voltage in 10mV
#define M3XL_ANA4_VOLTAGE_H						0xCD

// old
// #define M3XL_SYNC_READ_INDEX                    0xCA

// new on 10-12-2013
#define M3XL_SYNC_READ_INDEX						0xD0

// *** ALTERNATIVE CONTROL TABLE IDS FOR Michiel's V4 ***
// the next 5 parameters can be sent in one syncwrite packet
#define M3XL_P_ENERGY_L                                                 0xC4    // when start writing at this address
#define M3XL_P_ENERGY_H                                                 0xC5    // all PID ENERGY_MODE params should be sent, 10 bytes
#define M3XL_D_ENERGY_L                                                 0xC6
#define M3XL_D_ENERGY_H                                                 0xC7
#define M3XL_I_ENERGY_L                                                 0xC8
#define M3XL_I_ENERGY_H                                                 0xC9
#define M3XL_IL_ENERGY_L                                                0xCA
#define M3XL_IL_ENERGY_H                                                0xCB
#define M3XL_PID_ENERGY_SCALE_L                                         0xCC
#define M3XL_PID_ENERGY_SCALE_H                                         0xCD

#define M3XL_REFERENCE_ENERGY_L                                         0xCE    // RW Energy in mJ
#define M3XL_REFERENCE_ENERGY_H                                         0xCF
// *** END ALTERNATIVE CONTROL TABLE IDS FOR Michiel's V4 ***

//***************************************************************************
// END OF TABLE
//***************************************************************************

#endif //__3MXLCONTROLTABLE_H_INCLUDED__
