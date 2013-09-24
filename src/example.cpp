#include <ros/ros.h>
#include <threemxl/example.h>
#include <threemxl/C3mxlROS.h>

void DxlROSExample::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (path)
  {
    ROS_INFO("Using shared_serial");
    motor_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor_ = new C3mxl();
    
    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor_->setSerialPort(&serial_port_);
  }

  motor_->setConfig(config->setID(109));
  motor_->init(false);
  motor_->set3MxlMode(TORQUE_MODE);

  delete config;
}

void DxlROSExample::spin()
{
  ros::Rate loop_rate(1); //herz;
  int dir = -1;

  while (ros::ok())
  {
    motor_->setTorque(dir*0.005); // nM
    
    dir = -dir;
    loop_rate.sleep();
  }
  
  motor_->setTorque(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_example");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSExample dxl_ros_example;
  
  dxl_ros_example.init(path);
  dxl_ros_example.spin();
  
  return 0;   
} 
