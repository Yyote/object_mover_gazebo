cmake_minimum_required(VERSION 3.0.2)
project(object_mover_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
  message_generation
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  gazebo_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_mover_gazebo
#  CATKIN_DEPENDS gazebo_msgs roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gazebo_model_state_client src/gazebo_model_state_client.cpp)
target_link_libraries(gazebo_model_state_client ${catkin_LIBRARIES})
add_dependencies(gazebo_model_state_client object_mover_gazebo_generate_messages_cpp)

add_executable(object_mover src/object_mover.cpp)
target_link_libraries(object_mover ${catkin_LIBRARIES})
add_dependencies(object_mover object_mover_gazebo_generate_messages_cpp)
