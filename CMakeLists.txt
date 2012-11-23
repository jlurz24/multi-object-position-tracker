cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

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
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(dynamic_object_detector src/dynamic_object_detector.cpp src/kalman_simple.cpp src/pv_filter.cpp)
rosbuild_add_executable(init src/init_action.cpp)
rosbuild_add_executable(multi_object_detector src/multi_object_detector.cpp)
rosbuild_add_executable(move_right_arm_away src/move_right_arm_away.cpp)
rosbuild_add_executable(move_left_arm_away src/move_left_arm_away.cpp)
rosbuild_link_boost(multi_object_detector signals)
rosbuild_link_boost(dynamic_object_detector signals)
#target_link_libraries(example ${PROJECT_NAME})
