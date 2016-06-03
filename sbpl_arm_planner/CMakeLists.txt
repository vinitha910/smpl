add_library(
    sbpl_arm_planner
    src/BFS_3D.cpp
    src/action_set.cpp
    src/arm_planner_interface.cpp
    src/bfs_heuristic.cpp
    src/collision_checker.cpp
    src/manip_heuristic.cpp
    src/manip_lattice.cpp
    src/multi_frame_bfs_heuristic.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp)

target_link_libraries(
    sbpl_arm_planner
    ${catkin_LIBRARIES}
    ${BULLET_LIBRARIES}
    sbpl)

install(
    TARGETS sbpl_arm_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

