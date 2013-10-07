#ifndef CDXLPACKETHANDLER_H_
#define CDXLPACKETHANDLER_H_

#include "CDxlPacket.hpp"

class CDxlPacketHandler
{
    public:
        virtual		~CDxlPacketHandler() { }
        virtual int     init() = 0;
        virtual int     sendPacket(CDxlPacket *packet, bool replyExpected) = 0;
        virtual int     receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds) = 0;
        virtual int     getLastError() = 0;
};

#endif /* CDXLPACKETHANDLER_H_ */
