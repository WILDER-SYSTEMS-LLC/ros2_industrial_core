/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>

#include "industrial_utils/param_utils.h"
#include "industrial_utils/utils.h"
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"


// This is cribbed from 
// https://github.com/ros2/rclcpp/blob/rolling/rclcpp/test/rclcpp/test_logging.cpp

#include "rclcpp/logging.hpp"
rclcpp::Logger g_logger = rclcpp::get_logger("name");
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(g_logger,x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(g_logger,x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(g_logger,x)

#include <urdf/urdfdom_compatibility.h>

namespace industrial_utils
{
namespace param
{
	// Get value of named parameter if exists?
	// Then push it onto the provided listt
bool getListParam(const std::string param_name, std::vector<std::string> & list_param)
{
  bool rtn = false;
  rclcpp::Parameter value = this->get_parameter(param_name);
  rtn = (value.get_type_name() == "std::string");


  //rtn = ros::param::get(param_name, rpc_list);

  if (rtn)
  {
    std::string my_str = value.as_string();
    list_param.push_back(my_str);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param_name);
  }

  return rtn;

}

std::string vec2str(const std::vector<std::string> &vec)
{
  std::string s, delim = ", ";
  std::stringstream ss;
  std::copy(vec.begin(), vec.end(), std::ostream_iterator<std::string>(ss, delim.c_str()));
  s = ss.str();
  return "[" + s.erase(s.length()-2) + "]";
}

bool getJointNames(const std::string joint_list_param, const std::string urdf_param,
		           std::vector<std::string> & joint_names)
{
  joint_names.clear();

  // 1) Try to read explicit list of joint names
  if (// ros::param::has(joint_list_param) &&
       getListParam(joint_list_param, joint_names))
  {
    ROS_INFO_STREAM("Found user-specified joint names in '" << joint_list_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    ROS_WARN_STREAM("Unable to find user-specified joint names in '" << joint_list_param << "'");

  // 2) Try to find joint names from URDF model
  urdf::Model model;
  if ( //ros::param::has(urdf_param)
       //&& 
       model.initParam(urdf_param)
       && findChainJointNames(model.getRoot(), true, joint_names) )
  {
    ROS_INFO_STREAM("Using joint names from URDF: '" << urdf_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    ROS_WARN_STREAM("Unable to find URDF joint names in '" << urdf_param << "'");

  // 3) Raise an error
  ROS_ERROR_STREAM(
      "Cannot find user-specified joint names. Tried ROS parameter '" << joint_list_param << "'"
      << " and the URDF in '" << urdf_param << "'.");
  return false;
}

bool getJointVelocityLimits(const std::string urdf_param_name, std::map<std::string, double> &velocity_limits)
{
  urdf::Model model;
  std::map<std::string, urdf::JointSharedPtr >::iterator iter;

  if (//!ros::param::has(urdf_param_name) || 
		  !model.initParam(urdf_param_name))
    return false;
    
  velocity_limits.clear();
  for (iter=model.joints_.begin(); iter!=model.joints_.end(); ++iter)
  {
    std::string joint_name(iter->first);
    urdf::JointLimitsSharedPtr limits = iter->second->limits;
    if ( limits && (limits->velocity > 0) )
      velocity_limits.insert(std::pair<std::string,double>(joint_name,limits->velocity));
  }
  
  return true;
}

} //industrial_utils::param
} //industrial_utils
