cmake_minimum_required(VERSION 2.8.3)
project(rb5_ros_wrapper)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
)

add_action_files(
  DIRECTORY action
  FILES Motion.action
  FILES manipulation.action

)

add_message_files(
  FILES
  update.msg

)

add_message_files(
  FILES
  update.msg
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(rb5_ros_wrapper src/wrapper.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(rb5_ros_wrapper ${catkin_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
)
