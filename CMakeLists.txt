cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# For action lib
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_check_for_sse()

# find_library(PROFILER_LIBRARY NAMES profiler PATHS /usr/lib)
#Eigen required
find_package(Eigen REQUIRED)
set(Gperftools_DIR .)
find_package(Gperftools REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_boost_directories()
rosbuild_add_executable(dynamic_object_detector src/dynamic_object_detector.cpp src/kalman_simple.cpp src/pv_filter.cpp)
target_link_libraries(dynamic_object_detector ${GPERFTOOLS_TCMALLOC_AND_PROFILER})

rosbuild_add_executable(multi_object_detector src/multi_object_detector.cpp)
target_link_libraries(multi_object_detector ${GPERFTOOLS_TCMALLOC_AND_PROFILER})

rosbuild_add_executable(position_velocity_measurer src/position_velocity_measurer.cpp)
rosbuild_add_executable(fps_measurer src/fps_measurer.cpp)
rosbuild_add_executable(display_track src/display_track.cpp src/find_track_impl.cpp)
rosbuild_add_executable(find_range src/find_range.cpp)
rosbuild_add_executable(find_track_server src/find_track_server.cpp src/find_track_impl.cpp)

rosbuild_link_boost(multi_object_detector signals)
rosbuild_link_boost(dynamic_object_detector signals)
rosbuild_link_boost(position_velocity_measurer signals)
rosbuild_link_boost(fps_measurer signals)
rosbuild_link_boost(find_track_server signals)
