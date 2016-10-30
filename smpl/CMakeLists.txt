cmake_minimum_required(VERSION 2.8)
project(smpl)

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
        smpl
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

add_definitions(-DSBPL_VISUALIZE_MIN_SEVERITY=SBPL_VISUALIZE_SEVERITY_INFO)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${sbpl_INCLUDE_DIRS})
include_directories(include)

add_library(
    smpl
    src/bfs3d.cpp
    src/csv_parser.cpp
    src/planner_interface.cpp
    src/collision_checker.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/debug/visualize.cpp
    src/debug/visualizer_ros.cpp
    src/graph/action_space.cpp
    src/graph/manip_lattice.cpp
    src/graph/manip_lattice_action_space.cpp
    src/graph/robot_planning_space.cpp
    src/graph/workspace_lattice.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp)

target_link_libraries(smpl ${catkin_LIBRARIES} ${sbpl_LIBRARIES})

install(
    TARGETS smpl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
