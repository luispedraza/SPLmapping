cmake_minimum_required(VERSION 2.4.6)
# To use any of the ROS macros, you must first do this:
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

ADD_SUBDIRECTORY(src/Random)
ADD_SUBDIRECTORY(src/SPLM)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(SPLmapping src/SPLmapping.cpp)
target_link_libraries(SPLmapping -lnewmat -lglut MyRandom SPLM)

rosbuild_add_executable(SPLMapLoad src/SPLMapLoad.cpp)
target_link_libraries(SPLMapLoad -lnewmat -lglut MyRandom SPLM)

rosbuild_add_executable(zulumapping src/zulumapping.cpp)

#target_link_libraries(example ${PROJECT_NAME})

