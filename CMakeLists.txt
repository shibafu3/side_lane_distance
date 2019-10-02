cmake_minimum_required(VERSION 2.8.3)
project(side_lane_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES side_lane_detector
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/nvidia/catkin_ws/devel/include 
  /home/nvidia/Documents/shibahata/include/
  /usr/include/eigen3
)

add_executable(${PROJECT_NAME}_node src/side_lane_detector_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )