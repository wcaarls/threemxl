#ifndef __THREEMXL_EXAMPLE_H
#define __THREEMXL_EXAMPLE_H

#include <ros/ros.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGroup.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlROSExample
{
  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor_;   ///< Motor interface
    CDxlGroup *motors_;    ///< Multiple motor interface
    LxSerial serial_port_; ///< Serial port interface

  public:
    /// Constructor
    DxlROSExample() : nh_("~"), motor_(NULL) { }
    
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSExample()
    {
      if (motor_)
        delete motor_;
      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    
    /// Spin
    /** Alternatively drives the motor clockwise and counterclockwise */
    void spin();
};    

#endif /* __THREEMXL_EXAMPLE_H */
