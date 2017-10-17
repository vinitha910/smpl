cmake_minimum_required(VERSION 2.8.12)
project(smpl_ros)

set(CMAKE_BUILD_TYPE Release)

add_compile_options("-std=c++11")

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

find_package(
    catkin
    REQUIRED
    COMPONENTS
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        smpl
        trajectory_msgs
        visualization_msgs)

catkin_package(
    INCLUDE_DIRS
        include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
        smpl_ros
    CATKIN_DEPENDS
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        smpl
        trajectory_msgs
        visualization_msgs
    DEPENDS
        Boost
        sbpl)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${sbpl_INCLUDE_DIRS})
include_directories(include)

add_library(
    smpl_ros
    src/debug/visualizer_ros.cpp
    src/debug/marker_conversions.cpp
    src/ros/adaptive_workspace_lattice_allocator.cpp
    src/ros/araplanner_allocator.cpp
    src/ros/bfs_heuristic_allocator.cpp
    src/ros/dijkstra_egraph_3d_heuristic_allocator.cpp
    src/ros/experience_graph_planner_allocator.cpp
    src/ros/euclid_dist_heuristic_allocator.cpp
    src/ros/joint_dist_heuristic_allocator.cpp
    src/ros/joint_dist_egraph_heuristic_allocator.cpp
    src/ros/laraplanner_allocator.cpp
    src/ros/manip_lattice_allocator.cpp
    src/ros/manip_lattice_egraph_allocator.cpp
    src/ros/mhaplanner_allocator.cpp
    src/ros/multi_frame_bfs_heuristic_allocator.cpp
    src/ros/adaptive_planner_allocator.cpp
    src/ros/planner_interface.cpp
    src/ros/propagation_distance_field.cpp
    src/ros/workspace_lattice_allocator.cpp)

target_link_libraries(smpl_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(
    DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
install(
    TARGETS smpl_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
