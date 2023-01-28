cmake_minimum_required(VERSION 3.5)

project(industrial_robot_client)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(industrial_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_message REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

## See https://design.ros2.org/articles/actions.html
#find_package(actionlib REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

# Shouldnt this be handled by find_package?? Guess not.
include_directories(include industrial_msgs trajectory_msgs)


set(SRC_FILES 
	src/joint_relay_handler.cpp
	src/joint_trajectory_downloader.cpp
	src/joint_trajectory_interface.cpp
	#FIXME             src/joint_trajectory_streamer.cpp
	#FIXME              src/robot_state_interface.cpp
	#FIXME              src/robot_status_relay_handler.cpp
	             src/utils.cpp
	)



add_library(${PROJECT_NAME} ${SRC_FILES})

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} simple_message)
## ament_target_dependencies(${PROJECT_NAME} ${FULL_DEPENCENCIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp
	simple_message 
	std_msgs         # base
	control_msgs     # base
	industrial_msgs
	industrial_utils
	sensor_msgs      # base
	trajectory_msgs  # base
	urdf             # base
)

#
# target_compile_definitions(${PROJECT_NAME} PUBLIC ${simple_message_DEFINITIONS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  )

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
