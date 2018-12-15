cmake_minimum_required(VERSION 2.8.3)
project(Bag2PCD_ROStool)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(bag_to_pcd src/bag_to_pcd.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(bag_to_pcd ${catkin_LIBRARIES})
