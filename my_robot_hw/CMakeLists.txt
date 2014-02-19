cmake_minimum_required(VERSION 2.8.3)
project(my_robot_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    transmission_interface
  INCLUDE_DIRS include
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
    src/my_robot_hw.cpp
    include/my_robot_hw/my_robot_hw.h  
    )

add_executable(my_robot_hw_test src/my_robot_hw_test.cpp)
target_link_libraries(my_robot_hw_test ${PROJECT_NAME} ${catkin_LIBRARIES})
