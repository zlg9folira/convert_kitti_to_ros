cmake_minimum_required(VERSION 3.2)
project(convert_kitti_to_ros)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-lboost_program_options")
find_package(Boost COMPONENTS program_options filesystem REQUIRED)
find_package(PCL REQUIRED)

include_directories(
${Boost_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
MESSAGE("${Boost_LIBRARIES}")
add_executable(kitti2pcd kitti2pcd.cpp)
target_link_libraries(kitti2pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES} libboost_program_options.a)
