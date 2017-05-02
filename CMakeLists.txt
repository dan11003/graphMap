
cmake_minimum_required(VERSION 2.8.3)
project(graph_map)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++11)
endif()

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED
    geometry_msgs
                  #  pcl
    cmake_modules #not needed
    ndt_registration
    ndt_map
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    ndt_map)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED COMPONENTS)


catkin_package(
    DEPENDS Eigen libpcl-all-dev
    CATKIN_DEPENDS ndt_registration ndt_map roscpp   laser_geometry sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )


include_directories(include  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (${PROJECT_NAME}_LIB_SRCS
    src/graph_map/graph_map.cpp
    src/graph_map/map_node.cpp
    src/graph_map/map_type.cpp
    src/graph_map_fuser.cpp
    src/graph_map/reg_type.cpp
    src/ndt2d/ndt2d_map_type.cpp
    src/ndt2d/ndtd2d_reg_type.cpp
    src/ndt2d/ndt2d_map_param.cpp
    src/graphfactory.cpp
    src/visualization/graph_plot.cpp
    src/lidarUtils/lidar_utilities.cpp
    #src/graph_map_fuser_node.cpp
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_executable(test_graph test/test_graph.cpp )
add_executable(graph_map_fuser_node src/graph_map_fuser_node.cpp )



##
#
target_link_libraries(test_graph ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
target_link_libraries(graph_map_fuser_node ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

#  ${Eigen_LIBRARIES}
#)

#
