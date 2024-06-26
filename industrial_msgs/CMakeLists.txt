cmake_minimum_required(VERSION 3.5)

project(industrial_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(industrial_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


set(msg_files
  "msg/DebugLevel.msg"
  "msg/DeviceInfo.msg"
  "msg/RobotMode.msg"
  "msg/RobotStatus.msg"
  "msg/ServiceReturnCode.msg"
  "msg/TriState.msg"
  )

set(srv_files
  "srv/CmdJointTrajectory.srv"
  "srv/GetRobotInfo.srv"
  "srv/SetDrivePower.srv"
  "srv/SetRemoteLoggerLevel.srv"
  "srv/StartMotion.srv"
  "srv/StopMotion.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories( include )
#ament_export_libraries( ${PROJECT_NAME} )



ament_package()

