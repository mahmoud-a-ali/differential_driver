cmake_minimum_required(VERSION 3.0.2)
project(differential_driver)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS tf roscpp nav_msgs geometry_msgs visualization_msgs dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/Parameters.cfg)
catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  tf
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/nodes/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}.cpp src/pd_controller.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
