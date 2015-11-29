add_library(
    sbpl_manipulation_components_pr2
    src/sbpl_math.cpp
    src/orientation_solver.cpp)
    # src/pr2_kdl_robot_model.cpp
    # src/ubr1_kdl_robot_model.cpp)

target_link_libraries(sbpl_manipulation_components_pr2 ${catkin_LIBRARIES})

install(
    TARGETS sbpl_manipulation_components_pr2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

