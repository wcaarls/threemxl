#include "CDxlPacketHandler.h"
#include <threemxl/platform/hardware/serial/LxSerial.h>

class CDxlSerialPacketHandler : public CDxlPacketHandler
{
    protected:
	CLog2		mLog;
	LxSerial	*&mSerialPort;
	int		mLastError;

    public:
        CDxlSerialPacketHandler(LxSerial *&serialPort);
        int             init();
        int             sendPacket(CDxlPacket *packet, bool replyExpected);
        int             receivePacketWait(CDxlStatusPacket *packet, int seconds, int microseconds);
        int             getLastError();
};
