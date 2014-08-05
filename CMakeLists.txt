cmake_minimum_required(VERSION 2.8)
project(sbpl_manipulation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    angles
    geometry_msgs
    orocos_kdl
    kdl_parser
    leatherman
    message_filters
    moveit_core
    moveit_msgs
    moveit_ros_planning
    roscpp
    sbpl_geometry_utils
    shape_msgs
    sensor_msgs
    tf
    tf_conversions
    trajectory_msgs
    urdf
    visualization_msgs)
find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

# external include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${SBPL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${SBPL_LIBRARY_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

# internal include directories
include_directories(bfs3d/include)
include_directories(sbpl_arm_planner/include)
include_directories(sbpl_collision_checking/include)
include_directories(sbpl_manipulation_components/include)

add_definitions(-DROS)

# export libraries, headers, and dependencies
catkin_package(
    INCLUDE_DIRS
        bfs3d/include 
        sbpl_arm_planner/include 
        sbpl_collision_checking/include
        sbpl_manipulation_components/include
    LIBRARIES
        bfs3d
        sbpl_arm_planner
        sbpl_manipulation_components
        sbpl_collision_checking
    CATKIN_DEPENDS
        angles
        geometry_msgs
        kdl_parser
        leatherman
        moveit_core
        moveit_msgs
        moveit_ros_planning
        orocos_kdl
        tf
        tf_conversions
        trajectory_msgs
    DEPENDS 
        Boost Bullet Eigen sbpl)

add_subdirectory(bfs3d)
add_subdirectory(sbpl_arm_planner)
add_subdirectory(sbpl_collision_checking)
add_subdirectory(sbpl_manipulation_components)
