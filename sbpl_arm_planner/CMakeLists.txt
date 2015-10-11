add_library(sbpl_arm_planner
            src/environment_robarm3d.cpp
            src/action_set.cpp
            src/planning_params.cpp
            src/sbpl_arm_planner_interface.cpp)
target_link_libraries(sbpl_arm_planner ${catkin_LIBRARIES} ${BULLET_LIBRARIES} sbpl bfs3d)

install(TARGETS sbpl_arm_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

