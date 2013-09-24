#ifndef __DXL_ASSERT_H
#define __DXL_ASSERT_H

#include <ros/ros.h>

#define DXL_SAFE_CALL(call) \
  do { \
    int ret = call; \
    if (ret != DXL_SUCCESS) { \
      ROS_FATAL("DXL CALL FAILED\n\tfile = %s\n\tline = %d\n\tcall = %s\n\tmessage = %s", __FILE__, __LINE__, #call, CDxlCom::translateErrorCode(ret)); \
      ROS_ISSUE_BREAK(); \
    } \
  } while (0)

#endif /* __DXL_ASSERT_H */
