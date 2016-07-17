#ifndef ROS_H
#define ROS_H

#include "ros/node_handle.h"
#include "ChibiOSHardware.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace ros
{
    typedef NodeHandle_<ChibiOSHardware, 25, 25, 512, 512> NodeHandle;
}

#ifdef __cplusplus
}
#endif

#endif /* ROS_H */
