#ifndef C3MXLROS_H_
#define C3MXLROS_H_

#include <3mxl.h>
#include <threemxl/CDxlROSPacketHandler.h>

/// Communication with a shared 3mxl chain
class C3mxlROS : public C3mxl
{
  public:
    /// Constructor
    /** \param path path to shared_serial node */
    C3mxlROS(const char *path)
    {
      setPacketHandler(new CDxlROSPacketHandler(path));
    }
};

#endif /* C3MXLROS_H_ */
