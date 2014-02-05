cmake_minimum_required(VERSION 2.8.3)
project(sbpl_manipulation_components_pr2)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
             COMPONENTS
                angles
                orocos_kdl
                pr2_arm_kinematics
                roscpp
                sbpl_arm_planner
                sbpl_manipulation_components
                sbpl_geometry_utils
                urdf)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
                   angles
                   orocos_kdl
                   pr2_arm_kinematics
                   roscpp
                   sbpl_arm_planner
                   sbpl_manipulation_components
                   sbpl_geometry_utils
                   urdf
               DEPENDS Boost)

add_library(${PROJECT_NAME}
            src/sbpl_math.cpp
            src/orientation_solver.cpp 
            src/pr2_kdl_robot_model.cpp
            src/ubr1_kdl_robot_model.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

