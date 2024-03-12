cmake_minimum_required(VERSION 3.5)

project(industrial_trajectory_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(orocos_kdl REQUIRED)

ament_target_dependencies(
      class_loader
      moveit_core
      moveit_ros_planning
      pluginlib
      trajectory_msgs
      orocos_kdl
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/add_smoothing_filter.cpp
  src/n_point_filter.cpp
  src/smoothing_trajectory_filter.cpp
  src/uniform_sample_filter.cpp
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  )

install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
  )

ament_export_include_directories( include )
ament_export_libraries( ${PROJECT_NAME} )

ament_package()

