cmake_minimum_required(VERSION 2.8.3)
project(lidar_appearance_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    sensor_msgs
    std_msgs
    nav_msgs
    message_generation
    dynamic_reconfigure 
    message_filters
    tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
#find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

#generate_dynamic_reconfigure_options(
#    cfg/algorithmParameters.cfg
#)

catkin_package(
  	INCLUDE_DIRS include
#   	LIBRARIES ${PROJECT_NAME}_calibration
#   	CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#   	DEPENDS system_lib
#     message_runtime
)

###############
## Libraries ##
###############

include_directories(include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${libpointmatcher_INCLUDE_DIRS}
	${libnabo_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIR}
	${CERES_INCLUDE_DIRS}
)

add_library(
	${PROJECT_NAME}
	test/cal_icp.cpp	
	test/multi_cal_eval.cpp		
)

################
## Executable ##
################

add_executable(
	calib_preprocess
	src/calib_preprocess.cpp
)
target_link_libraries(
	calib_preprocess
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${CERES_LIBRARIES}
	${libpointmatcher_LIBRARIES}
)
#-----------------------------------------

add_executable(
	calib_plane_extraction
	src/calib_plane_extraction.cpp
)
target_link_libraries(
	calib_plane_extraction
	${catkin_LIBRARIES}	
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${CERES_LIBRARIES}
	${libpointmatcher_LIBRARIES}
)
#-----------------------------------------

add_executable(
	calib_icp
	src/calib_icp.cpp
)
target_link_libraries(
	calib_icp
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${CERES_LIBRARIES}
	${libpointmatcher_LIBRARIES}
)
#-----------------------------------------

add_executable(
	calib_evaluation_mse
	src/calib_evaluation_mse.cpp
)
target_link_libraries(
	calib_evaluation_mse
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${CERES_LIBRARIES}
	${libpointmatcher_LIBRARIES}
)
#-----------------------------------------

add_executable(
	simulation_data_generation
	src/simulation_data_generation.cpp
)
target_link_libraries(
	simulation_data_generation
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${CERES_LIBRARIES}
	${libpointmatcher_LIBRARIES}
)
#-----------------------------------------
