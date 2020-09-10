cmake_minimum_required(VERSION 3.0.2)
project(mygrid)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(grider src/grider.cpp)
add_library(mygridtool src/mygridtool.cpp)
target_link_libraries(grider ${catkin_LIBRARIES})
target_link_libraries(grider mygridtool)
