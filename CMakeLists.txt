cmake_minimum_required(VERSION 3.0.2)
project(pdw_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES learning_pkg
 CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

add_executable(walker
  src/walker.cpp
  src/biped_robot.cpp
  )
target_link_libraries(walker ${catkin_LIBRARIES})

add_executable(plot
  src/plot_walking.cpp
  )
target_link_libraries(plot ${catkin_LIBRARIES})
