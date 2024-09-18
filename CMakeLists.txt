cmake_minimum_required(VERSION 3.10.0)

project(rise)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree visualization)
message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION} " | PCL: " ${PCL_VERSION})

find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rospy pcl_ros pcl_conversions geodesy sensor_msgs geometry_msgs message_generation 
                image_transport cv_bridge nav_msgs)

add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")
#set(CMAKE_BUILD_TYPE "DEBUG")


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/rise.cmake)
