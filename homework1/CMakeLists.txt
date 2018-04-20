cmake_minimum_required(VERSION 2.8.3)
project(homework1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(homework1 src/homework1.cpp)
target_link_libraries(homework1 ${catkin_LIBRARIES})
add_dependencies(homework1 homework1_generate_messages_cpp)


