cmake_minimum_required(VERSION 2.8)
project(sbpl_arm_planner)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(Boost REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        geometry_msgs
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        sbpl_geometry_utils
        trajectory_msgs
        visualization_msgs)

find_package(Eigen REQUIRED)
find_package(sbpl REQUIRED)

set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_arm_planner
    CATKIN_DEPENDS
        geometry_msgs
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        sbpl_geometry_utils
        trajectory_msgs
        visualization_msgs
    DEPENDS
        Boost
        Eigen
        sbpl)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${sbpl_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_arm_planner
    src/bfs3d.cpp
    src/action_set.cpp
    src/arm_planner_interface.cpp
    src/bfs_heuristic.cpp
    src/euclid_dist_heuristic.cpp
    src/collision_checker.cpp
    src/manip_heuristic.cpp
    src/manip_lattice.cpp
    src/multi_frame_bfs_heuristic.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/robot_state_lattice.cpp
    src/workspace_lattice.cpp)

target_link_libraries(
    sbpl_arm_planner
    ${catkin_LIBRARIES}
    ${sbpl_LIBRARIES})

install(
    TARGETS sbpl_arm_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
