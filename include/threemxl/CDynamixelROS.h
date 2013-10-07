#ifndef CDYNAMIXELROS_H_
#define CDYNAMIXELROS_H_

#include <threemxl/platform/hardware/dynamixel/dynamixel/Dynamixel.h>
#include "CDxlROSPacketHandler.h"

/// Communication with a shared Dynamixel chain
class CDynamixelROS : public CDynamixel
{
  public:
    /// Constructor
    /** \param path path to shared_serial node */
    CDynamixelROS(const char *path)
    {
      setPacketHandler(new CDxlROSPacketHandler(path));
    }
};

#endif /* CDYNAMIXELROS_H_ */
