set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.2)
project(convert_kitti_to_ros)

set(CMAKE_CXX_FLAGS "-lboost_program_options")
find_package(Boost COMPONENTS program_options filesystem REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros pcl_conversions)
find_package(PCL 1.2 REQUIRED)
catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

MESSAGE("${Boost_LIBRARIES}")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pcd2ros src/pcd2ros.cpp)
add_executable(kitti2pcd src/kitti2pcd.cpp)
target_link_libraries(pcd2ros ${catkin_LIBRARIES})
target_link_libraries(kitti2pcd  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} libboost_program_options.a)

