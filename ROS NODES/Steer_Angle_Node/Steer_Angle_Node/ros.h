#ifndef _ROS_H_
#define _ROS_H_

#include "ros_lib/ros/node_handle.h"
#include "Atmega32Hardware.h"

namespace ros
{
  typedef ros::NodeHandle_<Atmega32Hardware> NodeHandle;
}

#endif