#/* *****************************************************************
# *
# * map_manager
# *
# * Copyright (c) 2019
# * Institute of Mechatronic Systems,
# * Leibniz Universitaet Hannover.
# * (BSD License)
# * All rights reserved.
# *
# * http://www.imes.uni-hannover.de
# *
# * This software is distributed WITHOUT ANY WARRANTY; without
# * even the implied warranty of MERCHANTABILITY or FITNESS FOR
# * A PARTICULAR PURPOSE.
# *
# * For further information see http://www.linfo.org/bsdlicense.html
# *
# ******************************************************************/

#/**
# * @file   CMakeLists.txt
# * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
# * @date   26.07.2019
# *
# * @brief  CMakeLists for ROS-package map_manager
# */

cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs message_generation move_base_msgs actionlib tf cv_bridge image_transport pose_cov_ops)
FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS)

find_package(MRPT REQUIRED)
message(STATUS "Found MRPT: " ${MRPT_VERSION})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )

# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)

include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)

include_directories(${CSparse_INCLUDE_DIR})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  save_linkpoint_pose.msg
  start_env_detection.msg
  end_env_detection.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   roscpp std_msgs sensor_msgs geometry_msgs move_base_msgs actionlib tf
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_manager
#  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${MRPT_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSparse_INCLUDE_DIRS}
  ${SuiteSparse_INCLUDE_DIRS}

)

add_library(tutorial_slam2d_library ${G2O_LIB_TYPE}
  include/graphoptimizer/edge_se2_pointxy.cpp  include/graphoptimizer/se2.h                include/graphoptimizer/vertex_point_xy.h
  include/graphoptimizer/edge_se2.cpp    include/graphoptimizer/edge_se2_pointxy.h    include/graphoptimizer/vertex_se2.cpp
  include/graphoptimizer/edge_se2.h      include/graphoptimizer/rand.h                include/graphoptimizer/vertex_point_xy.cpp  include/graphoptimizer/vertex_se2.h
  include/graphoptimizer/parameter_se2_offset.h include/graphoptimizer/parameter_se2_offset.cpp
  include/graphoptimizer/types_tutorial_slam2d.h include/graphoptimizer/types_tutorial_slam2d.cpp
  include/graphoptimizer/simulator.h include/graphoptimizer/simulator.cpp
  include/graphoptimizer/g2o_tutorial_slam2d_api.h
)


add_executable(sm_mapping
  src/sm_mapping.cpp
  include/sm_mapping/sm_mapping.h
  src/dbdriver.cpp
  include/dbdriver/dbdriver.h
  src/linkpoint.cpp
  include/linkpoint/linkpoint.h
  src/text_input.cpp
  include/text_input/text_input.h
  src/globaltracker.cpp
  include/globaltracker/globaltracker.h
  src/lpgraph.cpp
  include/lpgraph/lpgraph.h
)
add_dependencies(sm_mapping map_manager_generate_messages_cpp)

add_executable(automated_sm_mapping
  src/automated_sm_mapping.cpp
  include/automated_sm_mapping/automated_sm_mapping.h
  src/dbdriver.cpp
  include/dbdriver/dbdriver.h
  src/linkpoint.cpp
  include/linkpoint/linkpoint.h
  src/text_input.cpp
  include/text_input/text_input.h
  src/globaltracker.cpp
  include/globaltracker/globaltracker.h
  src/lpgraph.cpp
  include/lpgraph/lpgraph.h
  src/scanutils.cpp
  include/scanutils/scanutils.h
)
add_dependencies(automated_sm_mapping map_manager_generate_messages_cpp)

add_executable(rtabmap_starter
  src/rtabmap_starter.cpp
  include/rtabmap_starter/rtabmap_starter.h
)

add_executable(environment_detector
  src/environment_detector.cpp
  include/environment_detector/environment_detector.h
  src/dbdriver.cpp
  include/dbdriver/dbdriver.h
  src/linkpoint.cpp
  include/linkpoint/linkpoint.h
  src/graphoptimizer.cpp
  include/graphoptimizer/graphoptimizer.h
  src/scanutils.cpp
  include/scanutils/scanutils.h
)
add_dependencies(environment_detector map_manager_generate_messages_cpp)

add_executable(register_velodyne
  src/register_velodyne.cpp
  include/register_velodyne/register_velodyne.h
)
add_dependencies(register_velodyne map_manager_generate_messages_cpp)

add_executable(save_map
  src/save_map.cpp
  include/save_map/save_map.h
  src/dbdriver.cpp
  include/dbdriver/dbdriver.h
  src/text_input.cpp
  include/text_input/text_input.h
  src/linkpoint.cpp
  include/linkpoint/linkpoint.h
)
add_dependencies(save_map map_manager_generate_messages_cpp)

add_executable(map_saver_starter
  src/map_saver_starter.cpp
  include/map_saver_starter/map_saver_starter.h
)

add_executable(rvizgraph
  src/rvizgraph.cpp
  include/rvizgraph/rvizgraph.h
  src/dbdriver.cpp
  include/dbdriver/dbdriver.h
  src/text_input.cpp
  include/text_input/text_input.h
  src/linkpoint.cpp
  include/linkpoint/linkpoint.h
  src/lpgraph.cpp
  include/lpgraph/lpgraph.h
)


target_link_libraries(sm_mapping
  ${catkin_LIBRARIES}
  sqlite3
  ${RTABMap_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(automated_sm_mapping
  ${catkin_LIBRARIES}
  sqlite3
  ${RTABMap_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${MRPT_LIBS}
  pose_cov_ops
)

target_link_libraries(rtabmap_starter
    ${catkin_LIBRARIES}
)

target_link_libraries(environment_detector
    ${catkin_LIBRARIES}
    sqlite3
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    ${CSparse_LIBRARIES}
    tutorial_slam2d_library
    ${SuiteSparse_LIBRARIES}
    ${G2O_LIBS}
)

target_link_libraries(register_velodyne
  ${catkin_LIBRARIES}
  ${RTABMap_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(save_map
    ${catkin_LIBRARIES}
    sqlite3
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

target_link_libraries(map_saver_starter
    ${catkin_LIBRARIES}
)

target_link_libraries(rvizgraph
    ${catkin_LIBRARIES}
    sqlite3
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)
