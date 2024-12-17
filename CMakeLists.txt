cmake_minimum_required(VERSION 3.0.2)
project(direction_indicator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    audio_compass
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        std_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        audio_compass
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/direction_indicator_node.cpp
    src/direction_calculator.cpp
    src/state_machine.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)