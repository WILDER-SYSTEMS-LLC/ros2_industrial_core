# Industrial Core

ROS2 porting effort:

https://github.com/users/ppbrown/projects/1

Currently building successfully:

    simple_message
    industrial_msgs

Building as a hackjob:

    industrial_utils (removed temporarily, not sure how to handle param parsing)

Main target to kill;

industrial_robot_client   (needs LOTS of work)

# GIT Branch

The current main branch is called melodic-devel because thats what upstream default is.
However, devel should actually happen on a ROS2-humble machine

## Building

A reminder to those new to ROS2, for steps to theoretically compile.

0. (install ROS2 packages, etc)
1. mkdir -p ~/ros/src
2. extract this repo under that src directory
3. extract repos for all the dependancies there too
    (maybe someday we'll support rosdep)
4. cd ~/ros
5. colcon build


[ROS-Industrial]: http://wiki.ros.org/Industrial
[upstream repository]: https://github.com/ros-industrial/swri-ros-pkg
