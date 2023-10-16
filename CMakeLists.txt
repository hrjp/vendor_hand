cmake_minimum_required(VERSION 3.0.2)
project(vendor_hand)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_wrapper
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vendor_hand
#  CATKIN_DEPENDS dynamixel_wrapper roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_executable(wire_vendor src/wire_vendor.cpp)
target_link_libraries(wire_vendor ${catkin_LIBRARIES})

add_executable(wire_vendor_dual src/wire_vendor_dual.cpp)
target_link_libraries(wire_vendor_dual ${catkin_LIBRARIES})

add_executable(arm_sim src/arm_sim.cpp)
target_link_libraries(arm_sim ${catkin_LIBRARIES})

add_executable(joy_to_value src/joy_to_value.cpp)
target_link_libraries(joy_to_value ${catkin_LIBRARIES})

add_executable(target_pub src/target_pub.cpp)
target_link_libraries(target_pub ${catkin_LIBRARIES})

add_executable(arm_controller src/arm_controller.cpp)
target_link_libraries(arm_controller ${catkin_LIBRARIES})