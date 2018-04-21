cmake_minimum_required(VERSION 2.8.3)
project(homework3)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package( OpenCV REQUIRED )


## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

add_executable(homework3 src/homework3.cpp)
target_link_libraries(homework3 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(homework3 homework3_generate_messages_cpp)

