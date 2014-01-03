cmake_minimum_required(VERSION 2.8.3)
project(sbpl_arm_planner)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIR})

find_package(catkin REQUIRED
             COMPONENTS
                 angles
                 bfs3d
                 orocos_kdl
                 leatherman
                 geometry_msgs
                 moveit_msgs
                 roscpp
                 sbpl_geometry_utils
                 sbpl_manipulation_components
                 tf
                 trajectory_msgs
                 visualization_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

include_directories(include)
add_definitions(-DROS)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
                   angles
                   geometry_msgs
                   moveit_msgs
                   sbpl_manipulation_components 
                   tf
                   trajectory_msgs
               DEPENDS Boost Bullet sbpl)

add_library(${PROJECT_NAME}
            src/environment_robarm3d.cpp
            src/action_set.cpp
            src/planning_params.cpp
            src/sbpl_arm_planner_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BULLET_LIBRARIES} sbpl)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
