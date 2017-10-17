cmake_minimum_required(VERSION 2.8.12)
project(smpl)

set(SMPL_ROS_BUILD OFF)
set(CMAKE_BUILD_TYPE Release)

add_compile_options("-std=c++11")

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

if(SMPL_ROS_BUILD)
    find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)

    catkin_package(
        INCLUDE_DIRS
            include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
            smpl
        CATKIN_DEPENDS
            roscpp
            visualization_msgs
        DEPENDS
            Boost
            sbpl)
endif()

add_definitions(-DSMPL_CLOCK_API=SMPL_CLOCK_CHRONO_HIGH_RESOLUTION)

#add_definitions(-DSMPL_CONSOLE_ROS)
#add_definitions(-DSMPL_CONSOLE_COLORIZE_OUTPUT=1)
#add_definitions(-DSMPL_CONSOLE_UNBUFFERED=1)
#add_definitions(-DSMPL_LOG_FILE_AND_LINE=1)

if(SMPL_ROS_BUILD)
    set(SMPL_SV_VISUALIZATION_MSGS ON)
endif()

add_definitions(-DSBPL_VISUALIZE_MIN_SEVERITY=SBPL_VISUALIZE_SEVERITY_INFO)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${sbpl_INCLUDE_DIRS})
if(SMPL_ROS_BUILD)
    include_directories(${catkin_INCLUDE_DIRS})
endif()
include_directories(include)

configure_file(
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h.in
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/smpl)

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
target_link_libraries(smpl ${Boost_LIBRARIES} ${sbpl_LIBRARIES})

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
    DIRECTORY include
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(
    TARGETS smpl
    ARCHIVE DESTINATION ${SMPL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_LIBRARY_DESTINATION}
    RUNTIME DESTINATION ${SMPL_RUNTIME_DESTINATION})
