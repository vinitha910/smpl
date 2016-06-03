add_library(sbpl_collision_checking
            src/group.cpp
            src/sbpl_collision_model.cpp
            src/collision_model_impl.cpp
            src/collision_model_config.cpp
            src/collision_world.cpp
            src/sbpl_collision_space.cpp)
target_link_libraries(sbpl_collision_checking sbpl_arm_planner ${catkin_LIBRARIES})
set_target_properties(sbpl_collision_checking PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")

add_executable(test_space src/test_collision_space.cpp)
target_link_libraries(test_space sbpl_arm_planner sbpl_collision_checking ${catkin_LIBRARIES})

install(TARGETS sbpl_collision_checking test_space
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

