
/* This file should not be included by anything outside of 
 * industrial_robot_client
 * It is a set of cheats to make the ROS1 -> ROS2 port easier
 */

#ifndef __IRC_INT
#define __IRC_INT

#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_INFO RCUTILS_LOG_INFO

#endif
