/*
 * CDxlROSPacketHandler.cpp
 *
 *      Author: Wouter Caarls
 */

#include <threemxl/platform/hardware/dynamixel/CDxlCom.h>
#include <threemxl/CDxlROSPacketHandler.h>
#include <shared_serial/SendTo.h>
#include <shared_serial/Recv.h>
#include <shared_serial/Flush.h>

CDxlROSPacketHandler::CDxlROSPacketHandler(const char *path) :
  nh_(path), initialized_(false), socket_(0), last_error_(0)
{
}

int CDxlROSPacketHandler::init()
{ 
  if (!initialized_)
  {
    ROS_INFO("Registering service clients");
  
    sendto_service_ = nh_.serviceClient<shared_serial::SendTo>("sendto");
    sendto_service_.waitForExistence();
  
    recv_service_ = nh_.serviceClient<shared_serial::Recv>("recv");
    recv_service_.waitForExistence();

    flush_pub_ = nh_.advertise<shared_serial::Flush>("flush", 10);
  
    initialized_ = true;
  }
  
  return DXL_SUCCESS;
}

int CDxlROSPacketHandler::sendPacket(CDxlPacket *packet, bool replyExpected)
{
  if (!initialized_) init();

  int length = packet->length();
  BYTE *data = packet->data();

  shared_serial::SendTo srv;

  srv.request.socket = 0;  
  srv.request.data.resize(length);    
  for (int ii=0; ii != length; ++ii)
    srv.request.data[ii] = data[ii];
  srv.request.timeout = 1;
  
  for (int ii=0; ii != SEND_RETRY_FACTOR; ++ii)
  {
    if (sendto_service_.call(srv))
    {
      socket_ = srv.response.socket;
      return DXL_SUCCESS;
    }
    
    ROS_WARN("Error sending packet");
    usleep(1000);
  }
  
  socket_ = 0;
  last_error_ = 0;
  ROS_ERROR("Couldn't send packet");
  return DXL_PKT_SEND_ERROR;
}

int CDxlROSPacketHandler::receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds)
{
  if (!initialized_) init();

  shared_serial::Recv srv;
  
  unsigned int length = packet->length();
  BYTE *data = packet->data();
  
  srv.request.socket = socket_;
  srv.request.length = length;
  srv.request.recv_timeout = seconds+microseconds/1000000.;
  srv.request.sock_timeout = 0;
  
  if (!recv_service_.call(srv))
  {
    socket_ = 0;
    last_error_ = 0;
    
    ROS_WARN("Couldn't receive packet");
    return DXL_PKT_RECV_ERROR;
  }
  
  socket_ = 0;
  for (unsigned int ii=0; ii != srv.response.data.size(); ++ii)
    data[ii] = srv.response.data[ii];
    
  if (srv.response.data.size() != length)
  {
    last_error_ = srv.response.data.size();
    ROS_WARN("Short reply");
    return DXL_PKT_RECV_ERROR;
  }

  if (packet->calcChecksum() != packet->readChecksum())
  {
    last_error_ = srv.response.data.size();
    ROS_WARN_STREAM("Checksum error in packet: Received CS:"<< std::setw (2) << std::uppercase << std::hex << (int)packet->readChecksum() << " Calculated CS:"<< std::hex << (int)packet->calcChecksum() << std::dec);
    ROS_WARN_STREAM("Packet was " << packet->getPktString());
    
    shared_serial::Flush msg;
    msg.socket = 0;
    msg.timeout = 0;
    flush_pub_.publish(msg);
    
    return DXL_PKT_RECV_CHECKSUM_ERR;
  }
  
  return DXL_SUCCESS;
}

int CDxlROSPacketHandler::getLastError()
{
  return last_error_;
}
