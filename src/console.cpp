#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <ros/ros.h>
#include <threemxl/console.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/CDynamixelROS.h>
#include <threemxl/platform/io/configuration/XMLConfiguration.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

using std::cout;
using std::endl;
using std::hex;
using std::dec;

#define DXLC_SAFE_CALL(call) \
  do { \
    int ret = call; \
    if (ret != DXL_SUCCESS) { \
      cout << "Error:" << endl << "  " << C3mxl::translateErrorCode(ret) << " (0x" << hex << ret << dec << ")" << endl; \
    } \
  } while (0)

bool DxlROSCommand::execute(ArgList args)
{
  // Sanity check
  if (args.size() != nargs_)
	return false;
  
  if (name_ == "exit")
  {
    ros::shutdown();
    return true;
  }
  else if (name_ == "id")
  {
    if (args.size() == 1)
    {
      int id = atoi(args[0].c_str());
      if (id >= 0 && id <= 255)
        console_->setMotor(id);
      else
        return false;
    }
    else
    {
      CDxlGeneric *motor = console_->getMotor();
      DxlROSConsole::MotorList &motors = console_->getMotors();
    
      cout << "Available motors:" << endl;
      for (size_t ii=0; ii < motors.size(); ++ii)
      {
        if (motors[ii] == motor)
          cout << "* ";
        else
          cout << "  ";
        cout << "id " << motors[ii]->getID() << endl;
      }
    }
  
    return true;
  }
  else if (name_ == "hb")
  {
    double freq = atof(args[0].c_str());
    if (freq < 0)
      return false;
      
    if (freq > 0)
      console_->setHeartbeatInterval(1/freq);
    else
      console_->setHeartbeatInterval(0);
      
    return true;
  }
  else if (name_ == "scan")
  {
    CDxlGeneric *motor = console_->createMotor();
    CDxlConfig config;
    std::vector<int> motors;

    for (int id=1; id < BROADCAST_ID && ros::ok(); ++id)
    {
      motor->setConfig(config.setID(id));
      if (motor->ping() == DXL_SUCCESS)
        motors.push_back(id);
        
      std::cout << "Scanning [";
      for (int ii=1; ii < BROADCAST_ID; ++ii)
        if (!(ii%15))
        {
          if (ii < id)
            cout << "=";
          else
            cout << ".";
        }
      cout << "]\r" << flush;
    }
    
    delete motor;
    
    if (motors.empty())
    {
      cout << "No motors found. Check the connection and power." << endl;
    }
    else
    {
      cout << "Motor controllers found on bus:" << endl;
      for (size_t ii=0; ii < motors.size(); ++ii)
        cout << "  id " << motors[ii] << endl;
    }
    
    return true;
  }
  
  CDxlGeneric *motor = console_->getMotor();
  if (!motor)
  {
    cout << "Error:" << endl << "  No motor specified" << endl;
    return true;
  }
  
  if (name_ == "init")
    DXLC_SAFE_CALL(motor->init(false));
  else if (name_ == "mode")
  {
    if (args[0] == "speed")
      DXLC_SAFE_CALL(motor->set3MxlMode(SPEED_MODE));
    else if (args[0] == "pos")
      DXLC_SAFE_CALL(motor->set3MxlMode(POSITION_MODE));
    else if (args[0] == "current")
      DXLC_SAFE_CALL(motor->set3MxlMode(CURRENT_MODE));
    else if (args[0] == "torque")
      DXLC_SAFE_CALL(motor->set3MxlMode(TORQUE_MODE));
    else if (args[0] == "sine")
      DXLC_SAFE_CALL(motor->set3MxlMode(SINUSOIDAL_POSITION_MODE));
    else if (args[0] == "ext_init")
      DXLC_SAFE_CALL(motor->set3MxlMode(EXTERNAL_INIT));
    else if (args[0] == "hsi_init")
      DXLC_SAFE_CALL(motor->set3MxlMode(HOME_SWITCH_AND_INDEX_INIT));
    else if (args[0] == "pwm")
      DXLC_SAFE_CALL(motor->set3MxlMode(PWM_MODE));
    else if (args[0] == "stop")
      DXLC_SAFE_CALL(motor->set3MxlMode(STOP_MODE));
    else
    {
      cout << "Unknown mode" << endl;
      return false;
    }
  }
  else if (name_ == "config")
  {
    CXMLConfiguration config_xml;
    
    if (!config_xml.loadFile(args[0]))
    {
      cout << "Couldn't read config from " << args[0] << endl;
      return false;
    }
    
    CDxlConfig config;
    if (args.size() == 2)
      config.readConfig(config_xml.root().section(args[1]));
    else
      config.readConfig(config_xml.root());
      
    if (config.mID.isSet() && config.mID != motor->getID())
      cout << "Config has different ID. Ignoring" << std::endl;
      
    config.setID(motor->getID());
    DXLC_SAFE_CALL(config.configureDynamixel(motor));
  }
  else if (name_ == "pos")
  {
    switch (args.size())
    {
      case 0:
        DXLC_SAFE_CALL(motor->getPos());
        cout << "Position:" << endl << "  " << motor->presentPos() << " rad" << endl;
        break;
      case 1:
        DXLC_SAFE_CALL(motor->setPos(atof(args[0].c_str())));
        break;
      case 2:
        DXLC_SAFE_CALL(motor->setPos(atof(args[0].c_str()), atof(args[1].c_str())));
        break;
      case 3:
        DXLC_SAFE_CALL(motor->setPos(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str())));
        break;
    }
  }
  else if (name_ == "speed")
    DXLC_SAFE_CALL(motor->setSpeed(atof(args[0].c_str())));
  else if (name_ == "accel")
  {
    switch (args.size())
    {
      case 0:
        DXLC_SAFE_CALL(motor->getAcceleration());
        cout << "Acceleration:" << endl << "  " << motor->presentAcceleration() << " rad/s^2" << endl;
        break;
      case 1:
        DXLC_SAFE_CALL(motor->setAcceleration(atof(args[0].c_str())));
        break;
    }
  }
  else if (name_ == "moffset")
    DXLC_SAFE_CALL(motor->setMotorOffset(atof(args[0].c_str())));
  else if (name_ == "joffset")
    DXLC_SAFE_CALL(motor->setJointOffset(atof(args[0].c_str())));
  else if (name_ == "jlimits")
    DXLC_SAFE_CALL(motor->setAngleLimits(atof(args[0].c_str()), atof(args[1].c_str())));
  else if (name_ == "lpos")
  {
    switch (args.size())
    {
      case 0:
        DXLC_SAFE_CALL(motor->getLinearPos());
        cout << "Position:" << endl << "  " << motor->presentLinearPos() << " m" << endl;
        break;
      case 1:
        DXLC_SAFE_CALL(motor->setLinearPos(atof(args[0].c_str())));
        break;
      case 2:
        DXLC_SAFE_CALL(motor->setLinearPos(atof(args[0].c_str()), atof(args[1].c_str())));
        break;
      case 3:
        DXLC_SAFE_CALL(motor->setLinearPos(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str())));
        break;
    }
  }
  else if (name_ == "lspeed")
    DXLC_SAFE_CALL(motor->setLinearSpeed(atof(args[0].c_str())));
  else if (name_ == "laccel")
  {
    switch (args.size())
    {
      case 0:
        DXLC_SAFE_CALL(motor->getLinearAcceleration());
        cout << "Acceleration:" << endl << "  " << motor->presentLinearAcceleration() << " m/s^2" << endl;
        break;
      case 1:
        DXLC_SAFE_CALL(motor->setLinearAcceleration(atof(args[0].c_str())));
        break;
    }
  }
  else if (name_ == "current")
    DXLC_SAFE_CALL(motor->setCurrent(atof(args[0].c_str())));
  else if (name_ == "torque")
    DXLC_SAFE_CALL(motor->setTorque(atof(args[0].c_str())));
  else if (name_ == "pwm")
    DXLC_SAFE_CALL(motor->setPWM(atof(args[0].c_str())));
  else if (name_ == "freq")
    DXLC_SAFE_CALL(motor->setSineFrequency(atof(args[0].c_str())));
  else if (name_ == "ampl")
    DXLC_SAFE_CALL(motor->setSineAmplitude(atof(args[0].c_str())));
  else if (name_ == "phase")
    DXLC_SAFE_CALL(motor->setSinePhase(atof(args[0].c_str())));
  else if (name_ == "accel")
    DXLC_SAFE_CALL(motor->setAcceleration(atof(args[0].c_str())));
  else if (name_ == "ppid")
  {
    if (args.empty())
    {
      double p, d, i, i_limit;
      DXLC_SAFE_CALL(motor->getPIDPosition(p, d, i, i_limit));
      cout << "Position gains:" << endl << "  " << std::fixed
           << "P  " << std::setw(8) << std::setprecision(3) << p << ", "
           << "D  " << std::setw(8) << std::setprecision(3) << d << ", "
           << "I  " << std::setw(8) << std::setprecision(3) << i << ", "
           << "IL " << std::setw(8) << std::setprecision(3) << i_limit << endl;
    }
    else
      DXLC_SAFE_CALL(motor->setPIDPosition(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str()), atof(args[3].c_str())));
  }
  else if (name_ == "spid")
  {
    if (args.empty())
    {
      double p, d, i, i_limit;
      DXLC_SAFE_CALL(motor->getPIDSpeed(p, d, i, i_limit));
      cout << "Speed gains:" << endl << "  " << std::fixed
           << "P  " << std::setw(8) << std::setprecision(3) << p << ", "
           << "D  " << std::setw(8) << std::setprecision(3) << d << ", "
           << "I  " << std::setw(8) << std::setprecision(3) << i << ", "
           << "IL " << std::setw(8) << std::setprecision(3) << i_limit << endl;
    }
    else
      DXLC_SAFE_CALL(motor->setPIDSpeed(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str()), atof(args[3].c_str())));
  }
  else if (name_ == "cpid")
  {
    if (args.empty())
    {
      double p, d, i, i_limit;
      DXLC_SAFE_CALL(motor->getPIDCurrent(p, d, i, i_limit));
      cout << "Current gains:" << endl << "  " << std::fixed
           << "P  " << std::setw(8) << std::setprecision(3) << p << ", "
           << "D  " << std::setw(8) << std::setprecision(3) << d << ", "
           << "I  " << std::setw(8) << std::setprecision(3) << i << ", "
           << "IL " << std::setw(8) << std::setprecision(3) << i_limit << endl;
    }
    else
      DXLC_SAFE_CALL(motor->setPIDCurrent(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str()), atof(args[3].c_str())));
  }
  else if (name_ == "tpid")
  {
    if (args.empty())
    {
      double p, d, i, i_limit;
      DXLC_SAFE_CALL(motor->getPIDTorque(p, d, i, i_limit));
      cout << "Torque gains:" << endl << "  " << std::fixed
           << "P  " << std::setw(8) << std::setprecision(3) << p << ", "
           << "D  " << std::setw(8) << std::setprecision(3) << d << ", "
           << "I  " << std::setw(8) << std::setprecision(3) << i << ", "
           << "IL " << std::setw(8) << std::setprecision(3) << i_limit << endl;
    }
    else
      DXLC_SAFE_CALL(motor->setPIDTorque(atof(args[0].c_str()), atof(args[1].c_str()), atof(args[2].c_str()), atof(args[3].c_str())));
  }
  else if (name_ == "state")
  {
    DxlROSConsole::MotorList &motors = console_->getMotors();

    cout << "State:" << endl << std::fixed;

    for (size_t ii=0; ii < motors.size(); ++ii)
    {
      if (motors[ii]->isInitialized() && motors[ii]->getID() != BROADCAST_ID)
      {
        if (motors[ii] == motor)
          cout << "* ";
        else
          cout << "  ";
        
        DXLC_SAFE_CALL(motors[ii]->getState());
        DXLC_SAFE_CALL(motors[ii]->getStatus());

        cout << "id " << motors[ii]->getID() << " "
           << std::setw(8) << std::setprecision(3) << motors[ii]->presentVoltage() << " V, "
           << std::setw(8) << std::setprecision(3) << motors[ii]->presentCurrent() << " A, "
           << std::setw(8) << std::setprecision(3) << motors[ii]->presentTorque()  << " Nm, "
           << std::setw(8) << std::setprecision(3) << motors[ii]->presentPos()     << " rad, "
           << std::setw(8) << std::setprecision(3) << motors[ii]->presentSpeed()   << " rad/s, "
           << C3mxl::translateErrorCode(motors[ii]->presentStatus()) << " (0x" << hex << motors[ii]->presentStatus() << dec << ")" << endl;
      }
    }
    
    cout << std::resetiosflags(std::ios_base::floatfield);
  }
  else if (name_ == "bus")
  {
    DXLC_SAFE_CALL(motor->getBusVoltage());
    cout << "Bus voltage:" << endl << "  " << motor->presentBusVoltage() << " V" << endl;
  }
  else if (name_ == "sensors")
  {
    DXLC_SAFE_CALL(motor->getSensorVoltages());
    cout << "Analog sensor voltages:" << endl << "  " << std::fixed
         << "Bus         " << std::setw(8) << std::setprecision(3) << motor->presentBusVoltage() << ", "
         << "Current ADC " << std::setw(8) << std::setprecision(3) << motor->presentCurrentADCVoltage() << ", "
         << "Analog 1    " << std::setw(8) << std::setprecision(3) << motor->presentAnalog1Voltage() << ", "
         << "Analog 2    " << std::setw(8) << std::setprecision(3) << motor->presentAnalog2Voltage() << ", "
         << "Analog 3    " << std::setw(8) << std::setprecision(3) << motor->presentAnalog3Voltage() << ", "
         << "Analog 4    " << std::setw(8) << std::setprecision(3) << motor->presentAnalog4Voltage() << endl;
  }
  else if (name_ == "log")
    DXLC_SAFE_CALL(motor->setLogInterval(atoi(args[0].c_str())));
  else if (name_ == "getlog")
  {
    DXLC_SAFE_CALL(motor->getLog());
    TMxlLog log = motor->presentLog();
    cout << "    Time      PWM  Current  Voltage  Desired   Actual" << endl;
    for (TMxlLog::iterator ii = log.begin(); ii != log.end(); ++ii)
      cout << (*ii) << endl;
  }
  else if (name_ == "savelog")
  {
    DXLC_SAFE_CALL(motor->getLog());
    TMxlLog log = motor->presentLog();

    std::ofstream logstream(args[0].c_str());
    logstream << "    Time      PWM  Current  Voltage  Desired   Actual" << endl;
    for (TMxlLog::iterator ii = log.begin(); ii != log.end(); ++ii)
      logstream << (*ii) << endl;
    logstream.close();
  }
  else if (name_ == "table")
  {
    BYTE data[256];
    
    // Read control table in chunks to stay within maximum message size
    for (int ii=0; ii < 256; ii += 64)
      DXLC_SAFE_CALL(motor->readData(ii, 64, &data[ii]));

    cout << "Control table:" << endl;
    for (int rr=0; rr != 16; rr++)
    {
      cout << "  " << hex << std::setfill('0');
      for (int cc=0; cc != 16; ++cc)
      {
        cout << std::setw(2) << cc*16+rr << ": " << std::setw(2) << (int)data[cc*16+rr];
        if (cc < 16-1) cout << ", ";
      }
      cout << dec << std::setfill(' ') << endl;
    }
  }
  else
  {
    cout << "Error:" << endl << "  Unknown command" << endl;
    return false;
  }

  return true;
}

void Lockable::wait(double interval)
{
  if (interval > 0)
  {
    struct timespec spec;
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    spec.tv_sec = mts.tv_sec;
    spec.tv_nsec = mts.tv_nsec;
#else
    clock_gettime(CLOCK_REALTIME, &spec);
#endif
    
    spec.tv_sec += (time_t)interval;
    spec.tv_nsec += (long)((interval-(time_t)interval)*1000000000);
    if (spec.tv_nsec >= 1000000000)
    {
      spec.tv_nsec -= 1000000000;
      spec.tv_sec++;
    }
    
    pthread_cond_timedwait(&condition_, &mutex_, &spec);
  }
  else
    pthread_cond_wait(&condition_, &mutex_);
}

CDxlGeneric *DxlROSConsole::createMotor()
{
  CDxlGeneric *motor;
  
  if (strncmp(path_, "/dev", 4))
  {
    motor = new C3mxlROS(path_);
  }
  else
  {
    motor = new C3mxl();
    motor->setSerialPort(&serial_port_);
  }

  return motor;
}

void DxlROSConsole::setMotor(int id)
{
  motor_ = NULL;
  
  // See if this motor already exists
  for (size_t ii=0; ii < motors_.size(); ++ii)
    if (motors_[ii]->getID() == id)
      motor_ = motors_[ii];
  
  // If not, construct a new motor object
  if (!motor_)
  {
    motor_ = createMotor();
  
    CDxlConfig config;
    motor_->setConfig(config.setID(id));
    motors_.push_back(motor_);
  }
}

void DxlROSConsole::init(char *path)
{
  path_ = path;
  
  if (strncmp(path_, "/dev", 4))
  {
    ROS_INFO_STREAM("Using shared_serial at " << path_);
  }
  else
  {
    ROS_INFO_STREAM("Using direct connection at " << path_);
    serial_port_.port_open(path_, LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
  }
  
  // Register commands
  commands_.push_back(DxlROSCommand(this, "exit",    0, "exit                     Quit program."));
  commands_.push_back(DxlROSCommand(this, "id",      0, "id                       Lists available motors."));
  commands_.push_back(DxlROSCommand(this, "id",      1, "id      <id>             Switches motor. id is an integer between 0 and 255."));
  commands_.push_back(DxlROSCommand(this, "hb",      1, "hb      <frequency>      Sets heartbeat frequency. 0 disables heartbeat."));

  commands_.push_back(DxlROSCommand(this, "init",    0, "init                     Initializes the motor."));
  commands_.push_back(DxlROSCommand(this, "mode",    1, "mode    <mode>           Sets the control mode. mode is one of {pos|speed|current|torque|sine|ext_init}."));
  commands_.push_back(DxlROSCommand(this, "config",  1, "config  <file>           Configure the motor with an XML file."));
  commands_.push_back(DxlROSCommand(this, "config",  2, "config  <file> <section> Configure the motor with a section from an XML file."));

  commands_.push_back(DxlROSCommand(this, "pos",     0, "pos                      Gets the current position in [rad]."));
  commands_.push_back(DxlROSCommand(this, "pos",     1, "pos     <position>       Sets the target position in [rad]."));
  commands_.push_back(DxlROSCommand(this, "pos",     2, "pos     <pos> <speed>    Sets the target position in [rad] and maximum velocity in [rad/s]."));
  commands_.push_back(DxlROSCommand(this, "pos",     3, "pos     <pos> <sp> <ac>  Sets the target position in [rad], velocity in [rad/s] and acceleration in [rad/s^2]."));
  commands_.push_back(DxlROSCommand(this, "speed",   1, "speed   <speed>          Sets the target speed in [rad/s]."));
  commands_.push_back(DxlROSCommand(this, "accel",   0, "accel                    Gets the current acceleration [rad/s^2]."));
  commands_.push_back(DxlROSCommand(this, "accel",   1, "accel   <accel>          Sets the acceleration for trajectory generation in [rad/s^2]."));
  
  commands_.push_back(DxlROSCommand(this, "lpos",    0, "lpos                     Gets the current position in [m]."));
  commands_.push_back(DxlROSCommand(this, "lpos",    1, "lpos    <position>       Sets the target position in [m]."));
  commands_.push_back(DxlROSCommand(this, "lpos",    2, "lpos    <pos> <speed>    Sets the target position in [m] and maximum velocity in [m/s]."));
  commands_.push_back(DxlROSCommand(this, "lpos",    3, "lpos    <pos> <sp> <ac>  Sets the target position in [m], velocity in [m/s] and acceleration in [m/s^2]."));
  commands_.push_back(DxlROSCommand(this, "lspeed",  1, "lspeed  <speed>          Sets the target speed in [m/s]."));
  commands_.push_back(DxlROSCommand(this, "laccel",  0, "laccel                   Gets the current acceleration in [m/s^2]."));
  commands_.push_back(DxlROSCommand(this, "laccel",  1, "laccel  <accel>          Sets the acceleration for trajectory generation in [m/s^2]."));
  
  commands_.push_back(DxlROSCommand(this, "current", 1, "current <current>        Sets the target current in [A]."));
  commands_.push_back(DxlROSCommand(this, "torque",  1, "torque  <torque>         Sets the target torque in [Nm]."));
  commands_.push_back(DxlROSCommand(this, "pwm",     1, "pwm     <dutycycle>      Sets the PWM duty cycle (between -1 and 1)."));
  commands_.push_back(DxlROSCommand(this, "freq",    1, "freq    <frequency>      Sets the target sine frequency in [Hz]."));
  commands_.push_back(DxlROSCommand(this, "ampl",    1, "ampl    <amplitude>      Sets the target sine amplitude in [rad]."));
  commands_.push_back(DxlROSCommand(this, "phase",   1, "phase   <phase>          Sets the target sine phase angle in [rad]."));

  commands_.push_back(DxlROSCommand(this, "accel",   1, "accel   <accel>          Sets the acceleration for trajectory generation in [rad/s^2]."));
  commands_.push_back(DxlROSCommand(this, "moffset", 1, "moffset <offset>         Sets the motor offset in [rad]."));
  commands_.push_back(DxlROSCommand(this, "joffset", 1, "joffset <offset>         Sets the joint offset in [rad]."));

  commands_.push_back(DxlROSCommand(this, "jlimits", 2, "jlimits <lower> <upper>  Sets the joint angle limits in [rad]."));
  
  commands_.push_back(DxlROSCommand(this, "ppid",    0, "ppid                     Gets the position PID gains."));
  commands_.push_back(DxlROSCommand(this, "ppid",    4, "ppid    <p> <d> <i> <il> Sets the position PID gains. il is the integration limit."));
  commands_.push_back(DxlROSCommand(this, "spid",    0, "spid                     Gets the speed PID gains."));
  commands_.push_back(DxlROSCommand(this, "spid",    4, "spid    <p> <d> <i> <il> Sets the speed PID gains. il is the integration limit."));
  commands_.push_back(DxlROSCommand(this, "cpid",    0, "cpid                     Gets the current PID gains."));
  commands_.push_back(DxlROSCommand(this, "cpid",    4, "cpid    <p> <d> <i> <il> Sets the current PID gains. il is the integration limit."));
  commands_.push_back(DxlROSCommand(this, "tpid",    0, "tpid                     Gets the torque PID gains."));
  commands_.push_back(DxlROSCommand(this, "tpid",    4, "tpid    <p> <d> <i> <il> Sets the torque PID gains. il is the integration limit."));

  commands_.push_back(DxlROSCommand(this, "bus",     0, "bus                      Displays the current bus voltage."));
  commands_.push_back(DxlROSCommand(this, "sensors", 0, "sensors                  Displays the current sensor voltages."));
  commands_.push_back(DxlROSCommand(this, "state",   0, "state                    Displays the current state of the motor board."));
  commands_.push_back(DxlROSCommand(this, "log",     1, "log     <interval>       Initializes logging for this motor. interval is in [ms]."));
  commands_.push_back(DxlROSCommand(this, "getlog",  0, "getlog                   Displays the log of the last motor command."));
  commands_.push_back(DxlROSCommand(this, "savelog", 1, "savelog <filename>       Saves the log of the last motor command."));
  
  commands_.push_back(DxlROSCommand(this, "table",   0, "table                    Displays the entire control table."));
  commands_.push_back(DxlROSCommand(this, "scan",    0, "scan                     Scan bus for motors."));
}

void DxlROSConsole::execute(std::string line)
{
  // Tokenize
  size_t pos = line.find(' ');
  std::string cmd = line.substr(0, pos);

  std::vector<std::string> args;
  while (pos != line.npos)
  {
    size_t newpos = line.find(' ', pos+1);
    args.push_back(line.substr(pos+1, newpos-pos-1));
    pos = newpos;
  }

  // Deal with help separately
  if (cmd == "help")
  {
    cout << "Available commands:" << endl;
    for (CommandList::iterator ii=commands_.begin(); ii != commands_.end(); ++ii)
      cout << "  " << ii->getHelp() << endl;
    cout << endl;
    cout << "Recommended initialization:" << endl;
    cout << "  id 109; init; mode torque" << endl;
    return;
  }

  // Try to find a matching command
  bool matched = false;
  for (CommandList::iterator ii=commands_.begin(); ii != commands_.end(); ++ii)
    if (ii->match(cmd, args))
    {
      matched = true;
      if (!ii->execute(args))
        cout << "Usage:" << endl << "  " << ii->getHelp() << endl;
      break;
    }

  // Try to find a loosely matching command and print help
  if (!matched)
  {
    std::stringstream ss;

    for (CommandList::iterator ii=commands_.begin(); ii != commands_.end(); ++ii)
      if (ii->matchHelp(cmd, args))
      {
        matched = true;
        ss << "  " << ii->getHelp() << endl;
      }

    if (matched)
      cout << "Usage:" << endl << ss.str();
  }

  // Command not found
  if (!matched)
    cout << "Error: " << endl << "  Unknown command '" << cmd << "'" << endl;
}

void DxlROSConsole::spin()
{
  char *buf, *home = getenv("HOME"), history_file[PATH_MAX] = {0};
  std::string line;

  // Load history
  strncat(history_file, home, PATH_MAX-1);
  strncat(history_file, "/.threemxl_console_history", PATH_MAX-1);
  read_history(history_file);
  
  cout << "3mxl console, type 'help' for help." << endl;
  while (ros::ok())
  {
    // Prompt for input
    buf = readline(">> ");
    
    if (!buf)
    {
      // Exit on EOF
      cout << "exit" << endl;
      break;
    }
    else if (*buf)
    {
      // Add to history buffer
      add_history(buf);
      line = buf;
      
      // Separate multiple commands
      std::vector<std::string> cmds;
      
      size_t pos = line.find(';');
      cmds.push_back(line.substr(0, pos));
      while (pos != line.npos)
      {
        size_t newpos = line.find(';', pos+1);
        cmds.push_back(line.substr(pos+1, newpos-pos-1));
        pos = newpos;
      }
      
      // Strip whitespace and execute commands
      for (std::vector<std::string>::iterator ii=cmds.begin(); ii != cmds.end(); ++ii)
      {
        while ((!ii->empty()) && (*ii)[0] == ' ') ii->erase(0, 1);
        while ((!ii->empty()) && (*ii)[ii->length()-1] == ' ') ii->erase(ii->length()-1, 1);
        
        lock();
        execute(*ii);
        unlock();
      }
    }
    
    // Process ROS stuff
    ros::spinOnce();
  }
  
  lock();

  // Stop the motors
  for (size_t ii=0; ii < motors_.size(); ++ii)
    motors_[ii]->set3MxlMode(STOP_MODE);
    
  unlock();
  
  // Write history
  write_history(history_file);
}

void DxlROSConsole::setHeartbeatInterval(double interval)
{
  hb_interval_ = interval;
  
  // Wake up heartbeat thread
  if (hb_interval_ > 0)
    signal();
}

void *DxlROSConsole::spin_hb(void *obj)
{
  DxlROSConsole *console = static_cast<DxlROSConsole*>(obj);
  if (!console)
  {
    ROS_ERROR("Couldn't start heartbeat thread");
    return NULL;
  }
  
  std::vector<int> status;
  
  while (ros::ok())
  {
    console->lock();
    
    // Wait for next update. This function may return prematurely, but we don't care.
    console->wait(console->hb_interval_);
    
    if (ros::ok() && console->hb_interval_ > 0)
    {
      bool error = false;
    
      // Get status of all motors
      DxlROSConsole::MotorList &motors = console->getMotors();
      for (size_t ii=0; ii < motors.size(); ++ii)
      {
        if (ii >= status.size())
          status.push_back(M3XL_STATUS_IDLE_STATE);
          
        // Skip broadcast id
        if (motors[ii]->getID() == BROADCAST_ID)
          continue; 
      
        motors[ii]->getStatus();
        
        // Compare current state to previous state to check for a new error
        if (status[ii] != motors[ii]->presentStatus() && motors[ii]->presentStatus() < 0x90 && motors[ii]->presentStatus() != DXL_SUCCESS)
          error = true;
      }
      
      // Display error if it changed
      if (error)
      {
        cout << endl << "Motor entered error state:" << endl;
      
        for (size_t ii=0; ii < motors.size(); ++ii)
          if (status[ii] != motors[ii]->presentStatus() && motors[ii]->presentStatus() < 0x90 && motors[ii]->presentStatus() != DXL_SUCCESS)
            cout << "  id " << motors[ii]->getID() << " " << motors[ii]->translateErrorCode(motors[ii]->presentStatus()) << endl;
            
        cout << ">> " << flush;
      }
      
      // Remember current error state
      for (size_t ii=0; ii < motors.size(); ++ii)
        status[ii] = motors[ii]->presentStatus();
    }
    console->unlock();
  }

  return NULL;  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_console");
  gLogFactory().setLevel(llWarning);

  char *path=NULL, default_path[] = "/dev/ttyUSB0";
  if (argc == 2)
    path = argv[1];
  else
    path = default_path;
 
  DxlROSConsole dxl_ros_console;
  
  dxl_ros_console.init(path);
  dxl_ros_console.spin();
  
  ros::shutdown();
  
  return 0;
} 
