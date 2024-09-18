cmake_minimum_required(VERSION 3.10.0)

# Find ROS build system
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rise
  CATKIN_DEPENDS pcl_ros pcl_conversions roscpp sensor_msgs nav_msgs
  DEPENDS system_lib OpenCV
)

include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/devel/include
)

list(APPEND library_source_files
    src/estimate_vel.cpp
    src/fast_pdndt.cpp
    src/eskf_rise.cpp
    src/rise_preprocess.cpp
    src/radarpcl_view.cpp
)


list(APPEND thirdparty_libraries
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)

message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

add_library(rise_lib SHARED ${library_source_files})
target_link_libraries(rise_lib ${thirdparty_libraries} fmt::fmt OpenMP::OpenMP_CXX)
target_include_directories(rise_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
install(TARGETS rise_lib
        LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME         DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

#node
add_executable(get_gt_from_coloradar apps/get_gt_from_coloradar.cpp)
target_link_libraries(get_gt_from_coloradar rise_lib ${thirdparty_libraries})

add_executable(rise_for_serial apps/rise_for_serial.cpp)
target_link_libraries(rise_for_serial rise_lib ${thirdparty_libraries} tbb)
