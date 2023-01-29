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



// This is cribbed from 
// https://github.com/ros2/rclcpp/blob/rolling/rclcpp/test/rclcpp/test_logging.cpp

/* not used 
#include "rclcpp/logging.hpp"
rclcpp::Logger g_logger = rclcpp::get_logger("name");
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(g_logger,x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(g_logger,x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(g_logger,x)
*/

#include <urdf/urdfdom_compatibility.h>

namespace industrial_utils
{
namespace param
{

  // This is primarily called by getJointNames?
	// Get value of named parameter if exists?
	// Then push it onto the provided list
  // ros2 doesnt support static ros::param::get any more. We have to have
  // a node instance pointer to work with.
bool getListParam(rclcpp::Node::SharedPtr nodeptr, const std::string param_name, 
  std::vector<std::string> & list_param)
{
  bool rtn = false;
  //rtn = ros::param::get(param_name, rpc_list);

  rclcpp::Parameter value;
  rtn = nodeptr ->get_parameter(param_name, value);

  // Original had to deal with the old style param "rpc_list"
  // with mandatory list parsing for the arg you want?
  // New style just lets you directly ask for the param by name

  if (rtn)
  {
    std::string my_str = value.as_string();
    list_param.push_back(my_str);
  }
  else
  {
    RCLCPP_ERROR_STREAM(nodeptr->get_logger(),"Failed to get parameter: " << param_name);
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

bool getJointNames(rclcpp::Node::SharedPtr nodeptr, const std::string joint_list_param, 
                   const std::string urdf_param_name,
		               std::vector<std::string> & joint_names)
{
  joint_names.clear();

  // 1) Try to read explicit list of joint names
  if (// ros::param::has(joint_list_param) &&
       getListParam(nodeptr, joint_list_param, joint_names))
  {
    RCLCPP_INFO_STREAM(nodeptr->get_logger(), 
                       "Found user-specified joint names in '" << joint_list_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    RCLCPP_WARN_STREAM(nodeptr->get_logger(), 
                       "Unable to find user-specified joint names in '" << joint_list_param << "'");

  rclcpp::Parameter urdf_params;
  // 2) Try to find joint names from URDF mode
  // Note: ROS2 only has urdf::Model::initFile and initString, 
  // NOT initParam
  // So maybe this option has to go away, dunno.
  // Or maybe it needs to be the name of an URDF file instead of whatever it was before.
  urdf::Model model;
  if ( nodeptr-> get_parameter(urdf_param_name, urdf_params)
       && model.initString(urdf_params.as_string())
       && findChainJointNames(model.getRoot(), true, joint_names) )
  {
    RCLCPP_INFO_STREAM(nodeptr->get_logger(), 
                       "Using joint names from URDF: '" << urdf_params << "': " << vec2str(joint_names));
    return true;
  }
  else
    RCLCPP_WARN_STREAM(nodeptr->get_logger(), 
                       "Unable to find URDF joint names in '" << urdf_param_name << "'");

  // 3) Raise an error
  RCLCPP_ERROR_STREAM(nodeptr->get_logger(), 
                      "Cannot find user-specified joint names. Tried ROS parameter '" << joint_list_param << "'"
      << " and the URDF in '" << urdf_param_name << "'.");
  return false;
}

bool getJointVelocityLimits(rclcpp::Node::SharedPtr nodeptr, 
                            const std::string urdf_param_name, 
                            std::map<std::string, double> &velocity_limits)
{
  rclcpp::Parameter urdf_params;
  urdf::Model model;
  std::map<std::string, urdf::JointSharedPtr >::iterator iter;

  // Note: ROS2 does not have urdf:Model::initParam any more
  if (nodeptr-> get_parameter(urdf_param_name, urdf_params) || 
		  !model.initString(urdf_params.as_string()))
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
