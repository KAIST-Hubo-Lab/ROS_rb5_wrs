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

add_executable(RB5_Client_Pick src/RB5_Extended_Client.cpp)
target_link_libraries(RB5_Client_Pick ${catkin_LIBRARIES})

add_executable(RB5_Client_Fixed src/RB5_Client.cpp)
target_link_libraries(RB5_Client_Fixed ${catkin_LIBRARIES})

add_executable(RB5_Client_yujin src/RB5_Client_yujin.cpp)
target_link_libraries(RB5_Client_yujin ${catkin_LIBRARIES})

add_executable(Grasp_motion src/Grasp_motion.cpp)
target_link_libraries(Grasp_motion ${catkin_LIBRARIES})

add_executable(image_motion src/image_capture_motion.cpp)
target_link_libraries(image_motion ${catkin_LIBRARIES})


add_executable(base_navigation_rb5 src/base_navigation_rb5.cpp)
target_link_libraries(base_navigation_rb5 ${catkin_LIBRARIES})

=======

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
)
