# Local properties
cmake_minimum_required(VERSION 2.8.3)
project(siehler)
add_definitions("-std=c++11")

# Find and Include ROS (catkin) components
find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    roscpp
    rospy
    calibration_common
    message_filters
    sensor_msgs
    image_transport
    pcl_conversions
    pcl_ros
    nodelet
    rosconsole
    cv_bridge
    opt_msgs
    opt_utils
    dynamic_reconfigure
    std_msgs
    rostime
    compressed_image_transport
    compressed_depth_image_transport
)
include_directories(${catkin_INCLUDE_DIRS})

# Find and Include external Libs
find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
find_package(Boost REQUIRED)
include_directories( ${BOOST_INCLUDE_DIRS} )
#find_package(Eigen3 REQUIRED)
#include_directories( ${EIGEN_INCLUDE_DIRS})
#find_package(OpenMP REQUIRED)
#find_package(CUDA REQUIRED)
#include_directories("${CUDA_INCLUDE_DIRS}")
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


# Include local
include_directories( include )

catkin_package(
    CATKIN_DEPENDS message_filters sensor_msgs image_transport roscpp pcl_ros pcl_conversions opt_utils opt_msgs 
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
)

add_library(${PROJECT_NAME}
  apps/pc_to_depth_image_node.cpp
  )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Executables
add_executable(pc_to_depth_image apps/pc_to_depth_image_node.cpp)
SET_TARGET_PROPERTIES(pc_to_depth_image PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(pc_to_depth_image ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} curses ${OpenCV_LIBRARIES})
