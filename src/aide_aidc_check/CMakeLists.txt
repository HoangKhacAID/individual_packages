cmake_minimum_required(VERSION 3.0.2)
project(aide_aidc_check)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
)


include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(aide_aidc_check_node src/aide_aidc_check.cpp)
add_dependencies(aide_aidc_check_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aide_aidc_check_node
   ${catkin_LIBRARIES})


