#ifndef CDXLROSPACKETHANDLER_H_
#define CDXLROSPACKETHANDLER_H_

#include <ros/ros.h>
#include <threemxl/platform/hardware/dynamixel/CDxlPacketHandler.h>

class CDxlROSPacketHandler : public CDxlPacketHandler
{
    protected:
        ros::NodeHandle		nh_;
        ros::ServiceClient	sendto_service_;
        ros::ServiceClient	recv_service_;
        ros::Publisher          flush_pub_;
        
        int			initialized_;
        int			socket_;
        int			last_error_;

    public:
        CDxlROSPacketHandler(const char *path);
        int			init();
        int			sendPacket(CDxlPacket *packet, bool replyExpected);
        int			receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds);
        int			getLastError();
};

#endif /* CDXLROSPACKETHANDLER_H_ */
