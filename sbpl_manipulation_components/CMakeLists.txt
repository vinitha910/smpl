add_library(sbpl_manipulation_components
            src/robot_model.cpp
            src/kdl_robot_model.cpp
            src/occupancy_grid.cpp
            src/collision_checker.cpp
            src/post_processing.cpp)

target_link_libraries(sbpl_manipulation_components ${catkin_LIBRARIES})

add_executable(test_kdl src/test_kdl_robot_model.cpp)
target_link_libraries(test_kdl sbpl_manipulation_components)
