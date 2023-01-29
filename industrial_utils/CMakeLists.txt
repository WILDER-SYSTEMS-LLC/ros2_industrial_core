cmake_minimum_required(VERSION 3.5)

project(industrial_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

##find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

add_library(${PROJECT_NAME} src/param_utils.cpp src/utils.cpp)

## ROS2 totally changed how parameter passing is done.
## So just skip the param parsing utils for now. FIXME
## add_library(${PROJECT_NAME} src/utils.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp urdf)


if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest_inds_utils test/utest.cpp)
    target_link_libraries(utest_inds_utils ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()


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

ament_export_include_directories( include )
ament_export_libraries( ${PROJECT_NAME} )


ament_package()
