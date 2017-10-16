cmake_minimum_required(VERSION 2.8.12)
project(smpl)

set(SMPL_ROS_BUILD OFF)
set(CMAKE_BUILD_TYPE Release)

add_compile_options("-std=c++11")

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

if(SMPL_ROS_BUILD)
    find_package(catkin
        REQUIRED
        COMPONENTS
            leatherman
            moveit_core
            moveit_msgs
            roscpp
            trajectory_msgs
            visualization_msgs)

    catkin_package(
        INCLUDE_DIRS
            include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
            smpl
            smpl_ros
        CATKIN_DEPENDS
            leatherman
            moveit_core
            moveit_msgs
            roscpp
            trajectory_msgs
            visualization_msgs
        DEPENDS
            Boost
            sbpl)
endif()

if(SMPL_ROS_BUILD)
    add_definitions(-DSMPL_CONSOLE_ROS)
    add_definitions(-DSMPL_SV_VISUALIZATION_MSGS)
endif()

add_definitions(-DSBPL_VISUALIZE_MIN_SEVERITY=SBPL_VISUALIZE_SEVERITY_INFO)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${sbpl_INCLUDE_DIRS})
if(SMPL_ROS_BUILD)
    include_directories(${catkin_INCLUDE_DIRS})
endif()
include_directories(include)

add_library(
    smpl
    src/bfs3d.cpp
    src/csv_parser.cpp
    src/collision_checker.cpp
    src/console/ansi.cpp
    src/console/console.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/debug/colors.cpp
    src/debug/marker_utils.cpp
    src/debug/visualize.cpp
    src/distance_map/chessboard_distance_map.cpp
    src/distance_map/distance_map_common.cpp
    src/distance_map/edge_euclid_distance_map.cpp
    src/distance_map/euclid_distance_map.cpp
    src/distance_map/sparse_distance_map.cpp
    src/geometry/bounding_spheres.cpp
    src/geometry/mesh_utils.cpp
    src/geometry/voxelize.cpp
    src/graph/action_space.cpp
    src/graph/adaptive_workspace_lattice.cpp
    src/graph/experience_graph.cpp
    src/graph/manip_lattice.cpp
    src/graph/manip_lattice_egraph.cpp
    src/graph/manip_lattice_action_space.cpp
    src/graph/robot_planning_space.cpp
    src/graph/workspace_lattice.cpp
    src/graph/workspace_lattice_base.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/generic_egraph_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp)

target_link_libraries(smpl ${Boost_LIBRARIES} ${sbpl_LIBRARIES} ${leatherman_LIBRARIES})

if(SMPL_ROS_BUILD)
    add_library(
        smpl_ros
        src/debug/visualizer_ros.cpp
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
    target_link_libraries(smpl_ros smpl ${catkin_LIBRARIES})
endif()

if(SMPL_ROS_BUILD)
    set(SMPL_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set(SMPL_ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(SMPL_LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(SMPL_RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    set(SMPL_INCLUDE_DESTINATION include)
    set(SMPL_ARCHIVE_DESTINATION lib)
    set(SMPL_LIBRARY_DESTINATION lib)
    set(SMPL_RUNTIME_DESTINATION bin)
endif()

install(
    TARGETS smpl
    ARCHIVE DESTINATION ${SMPL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_LIBRARY_DESTINATION}
    RUNTIME DESTINATION ${SMPL_RUNTIME_DESTINATION})
