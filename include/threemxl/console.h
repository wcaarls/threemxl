#ifndef __THREEMXL_CONSOLE_H
#define __THREEMXL_CONSOLE_H

#include <ros/ros.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>

class DxlROSConsole;

/// Motor command
class DxlROSCommand
{
  public:
	typedef std::vector<std::string> ArgList;

  protected:
	DxlROSConsole *console_; ///< Console on which to execute command.
	size_t nargs_;       ///< Number of arguments.
	std::string name_;   ///< Command name.
	std::string help_;   ///< Help information.

  public:
	DxlROSCommand(DxlROSConsole *console, std::string name, size_t nargs, std::string help) :
		console_(console), nargs_(nargs), name_(name), help_(help)
	{
	}
	
	/// Check whether a command line matches this command.
	bool match(std::string cmd, ArgList args)
	{
	  return cmd == name_ && args.size() == nargs_;
	}
	
	/// Check whether a command line loosely matches this command.
	bool matchHelp(std::string cmd, ArgList args)
	{
	  return cmd == name_;
	}

	/// Returns help information.
	std::string getHelp()
	{
	  return help_;
	}

	/// Executes the command with certain arguments.
	bool execute(ArgList args);
};

class Lockable
{
  protected:
    pthread_mutex_t mutex_; ///< Mutual exclusion lock
    pthread_cond_t condition_; ///< Condition variable

  public:
    Lockable()
    {
      pthread_mutex_init(&mutex_, NULL);
      pthread_cond_init(&condition_, NULL);
    }
    
    virtual ~Lockable()
    {
      pthread_mutex_destroy(&mutex_);
      pthread_cond_destroy(&condition_);
    }
  
    /// Lock
    void lock() { pthread_mutex_lock(&mutex_); }
    
    /// Unlock
    void unlock() { pthread_mutex_unlock(&mutex_); }
    
    /// Wait for signal
    /**
     * \param interval Time to wait in [s]. A value of 0 will cause the function to wait indefinitely.
     * \note May wake up spuriously.
     * \note Must be locked.
     */
    void wait(double interval);
    
    /// Send signal
    /* \note Must be locked. */
    void signal() { pthread_cond_signal(&condition_); }
};

/// Console appliation example for CDynamixel and CDynamixelROS
class DxlROSConsole : public Lockable
{
  public:
    typedef std::vector<DxlROSCommand> CommandList;
    typedef std::vector<CDxlGeneric*>  MotorList;
	
  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor_;   ///< Current motor interface
    MotorList motors_;     ///< Motor interface list
    LxSerial serial_port_; ///< Serial port interface
    CommandList commands_; ///< Available commands
    char *path_;           ///< Path to shared_serial node
    pthread_t hb_thread_;  ///< Heartbeat thread handle
    double hb_interval_;   ///> Heartbeat interval

  public:
    /// Constructor
    DxlROSConsole() : nh_("~"), motor_(NULL), path_(NULL), hb_interval_(0)
    {
      // Spawn heartbeat thread
      pthread_create(&hb_thread_, NULL, spin_hb, (void*)this);
    }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSConsole()
    {
      // Prod heartbeat thread. Does not actually work if ros::ok() is true
      lock();
      signal();
      unlock();
      
      // Wait for heartbeat thread to finish
      pthread_join(hb_thread_, NULL);
    
      for (size_t ii=0; ii < motors_.size(); ++ii)
        delete motors_[ii];
        
      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }
    
    /// Get current motor
    CDxlGeneric *getMotor() { return motor_; }
    
    /// Create new uninitialized motor object
    CDxlGeneric *createMotor();
    
    /// Switch motor
    /**
     * \param id Motor identifier
     * \note Creates new object if none of the current motors has this identifier.
     */
    void setMotor(int id);
    
    /// Get motor list
    MotorList &getMotors() { return motors_; }
    
    /// Set heartbeat interval
    /**
     * \param interval Interval in [s].
     * \note Console must be locked.
     */
    void setHeartbeatInterval(double interval);

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    
    /// Execute console command
    /**
     * \param cmd Command to execute
     * \note Console must be locked.
     */
    void execute(std::string cmd);
    
    /// Spin
    /** Alternatively drives the motor clockwise and counterclockwise */
    void spin();
    
    /// Heartbeat thread main function
    /** \param obj pointer to DxlROSConsole */
    static void *spin_hb(void *obj);
};    

#endif /* __THREEMXL_CONSOLE_H */
