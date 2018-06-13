cmake_minimum_required(VERSION 2.8.3)
project(vicon_twist)

set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp")

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
	DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(vicon_twist src/main.cpp)
target_link_libraries (vicon_twist ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
