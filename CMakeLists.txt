cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package( ALSA REQUIRED )
find_package( PCL REQUIRED )
find_package( PkgConfig REQUIRED )
pkg_check_modules( gsl REQUIRED gsl )

include_directories(${PCL_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(audio_noise_recognizer nodes/audio_noise_recognizer_node.cpp)
rosbuild_add_executable(kinesics_goal_state_estimation nodes/kinesics_goal_state_estimation_node.cpp)
rosbuild_add_executable(proxemics_controller nodes/proxemics_controller_node.cpp)
rosbuild_add_executable(proxemics_goal_state_estimation nodes/proxemics_goal_state_estimation_node.cpp)
rosbuild_add_executable(vocalics_controller nodes/vocalics_controller_node.cpp)
rosbuild_add_executable(vocalics_goal_state_estimation nodes/vocalics_goal_state_estimation_node.cpp)
#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(proxemics_goal_state_estimation ${gsl_LIBRARIES})
target_link_libraries(proxemics_goal_state_estimation ${PCL_LIBRARIES})
target_link_libraries(vocalics_controller ${ALSA_LIBRARIES})

#add dynamic reconfigure API
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

