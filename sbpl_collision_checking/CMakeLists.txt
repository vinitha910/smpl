cmake_minimum_required(VERSION 2.8.3)
project(sbpl_collision_checking)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(catkin REQUIRED
             COMPONENTS
                 angles
                 geometry_msgs
                 orocos_kdl
                 kdl_parser
                 leatherman
                 message_filters
                 moveit_msgs
                 moveit_ros_planning
                 roscpp
                 sbpl_geometry_utils
                 sbpl_manipulation_components
                 sensor_msgs
                 tf
                 tf_conversions
                 urdf
                 visualization_msgs)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_definitions(-DROS)
include_directories(include)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
                   geometry_msgs
                   kdl_parser
                   leatherman
                   moveit_msgs
                   moveit_ros_planning
                   orocos_kdl
                   roscpp
                   sbpl_geometry_utils
                   sbpl_manipulation_components
                   tf
                   tf_conversions
                   urdf
               DEPENDS Boost)

add_library(${PROJECT_NAME}
            src/group.cpp
            src/sbpl_collision_model.cpp
            src/sbpl_collision_space.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")

add_executable(test_space src/test_collision_space.cpp)
target_link_libraries(test_space ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} test_space
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
