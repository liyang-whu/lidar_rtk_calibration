cmake_minimum_required(VERSION 3.4 FATAL_ERROR)
project(lidar_rtk_calib_sdk)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# add_compile_options(-std=c++11) Enable C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(
    FATAL_ERROR
      "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly. Please use a different C++ compiler or report this problem to the developers."
    )
endif()

# set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to Release")

set(lidar_rtk_calib_VERSION_MAJOR 1)
set(lidar_rtk_calib_VERSION_MINOR 0)
set(lidar_rtk_calib_VERSION_PATCH 0)

configure_file("${PROJECT_SOURCE_DIR}/include/lidar_rtk_calibConfig.h.in"
               "${PROJECT_BINARY_DIR}/lidar_rtk_calibConfig.h")

include_directories("${PROJECT_BINARY_DIR}")

# ros

include_directories("${PROJECT_BINARY_DIR}" include)
execute_process(COMMAND lsb_release -rs OUTPUT_VARIABLE RELEASE_VERSIONNAME OUTPUT_STRIP_TRAILING_WHITESPACE)
message("THE RELEASE_VERSIONNAME  " ${RELEASE_VERSIONNAME})
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        std_msgs
                        geometry_msgs
                        nav_msgs
                        tf
                        cv_bridge
                        rosbag
                        roscpp
                        roslib
                        sensor_msgs)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

message("Gflags found = ${GFLAGS_FOUND}")
message("Glog found = ${GLOG_FOUND}")
set(Gflags_FOUND ${GFLAGS_FOUND})
set(Glog_FOUND ${GLOG_FOUND})

find_package(Threads QUIET)
find_package(Ceres QUIET REQUIRED)
set(GFLAGS_LIBRARY ${CMAKE_THREAD_LIBS_INIT})

catkin_package()

# PCL
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# lib name
set(LIB_PREFIX "gq_lidar_rtk_calib")
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  if("${RELEASE_VERSIONNAME}" STREQUAL "18.04")
    set(LIB_NAME "${LIB_PREFIX}_x86_64_1804")
    set(SUFFIX "_x86_64_1804")
  elseif("${RELEASE_VERSIONNAME}" STREQUAL "16.04")
    set(LIB_NAME "${LIB_PREFIX}_x86_64_1604")
    set(SUFFIX "_x86_64_1604")
  elseif("${RELEASE_VERSIONNAME}" STREQUAL "14.04")
    set(LIB_NAME "${LIB_PREFIX}_x86_64_1404")
    set(SUFFIX "_x86_64_1404")
  endif()
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  if("${RELEASE_VERSIONNAME}" STREQUAL "18.04")
    set(LIB_NAME "${LIB_PREFIX}_tx2_1804")
    set(SUFFIX "_tx2_1804")
  elseif("${RELEASE_VERSIONNAME}" STREQUAL "16.04")
    set(LIB_NAME "${LIB_PREFIX}_tx2_1604")
    set(SUFFIX "_tx2_1604")
  elseif("${RELEASE_VERSIONNAME}" STREQUAL "14.04")
    set(LIB_NAME "${LIB_PREFIX}_tx2_1404")
    set(SUFFIX "_tx2_1404")
  endif()
endif()


message("lib name " ${LIB_NAME})
# STATIC SHARED
add_library(${LIB_NAME}
            SHARED
            src/rs_lidar_rtk_calib.cpp
            include/geodetic_conv.hpp
            src/sim_rtk_mapping.cpp
            include/calib/HandEyeCalibration.cc 
    )
target_link_libraries("${LIB_NAME}"
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${GLOG_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${CERES_LIBRARIES})

add_executable(extract_rtk_pose_node src/extract_rtk_pose_node.cpp)
target_link_libraries(extract_rtk_pose_node ${PCL_LIBRARIES} ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(txt2pcd "src/txt2pcd.cpp")
target_link_libraries(txt2pcd ${PCL_LIBRARIES})

# add_executable(tf_use_quat test/tf_use_quat.cpp)
# target_link_libraries(tf_use_quat ${PCL_LIBRARIES})

# add_executable(tf_b_to_a_tool src/tf_b_to_a_tool.cpp)
# target_link_libraries(tf_b_to_a_tool ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(relative_pose_tool src/relative_pose_tool.cpp)
target_link_libraries(relative_pose_tool ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(rs_lidar_rtk_calibtraion_node src/rs_lidar_rtk_calib_node.cpp)
target_link_libraries(rs_lidar_rtk_calibtraion_node 
  ${LIB_NAME} 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES} 
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_executable(sim_rtk_mapping_node src/sim_rtk_mapping_node.cpp)
target_link_libraries(sim_rtk_mapping_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${LIB_NAME} ${OpenCV_LIBS})

# for create lib
# get_target_property(LIB_LOCATION ${LIB_NAME} LOCATION)
# add_custom_command(TARGET ${LIB_NAME} POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy ${LIB_LOCATION}
#                            ${CMAKE_CURRENT_SOURCE_DIR}/rs_lidar_rtk_calibration/lib)

# add_custom_command(TARGET ${LIB_NAME} POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/include/rs_lid_rtk_calib.h
#                            ${CMAKE_CURRENT_SOURCE_DIR}/rs_lidar_rtk_calibration/include)

# add_custom_command(TARGET ${LIB_NAME} POST_BUILD
#                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/include/sim_rtk_mapping.h
#                            ${CMAKE_CURRENT_SOURCE_DIR}/rs_lidar_rtk_calibration/include)
