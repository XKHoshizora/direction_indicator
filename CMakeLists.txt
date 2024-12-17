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
    LIBRARIES direction_calculator state_machine tts_client
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

# Build libraries
add_library(direction_calculator
    src/direction_calculator.cpp
)

add_library(state_machine
    src/state_machine.cpp
)

add_library(tts_client
    src/tts_client.cpp
)

# Link libraries
target_link_libraries(direction_calculator
    ${catkin_LIBRARIES}
)

target_link_libraries(state_machine
    ${catkin_LIBRARIES}
    tts_client
)

target_link_libraries(tts_client
    ${catkin_LIBRARIES}
)

# Build executable
add_executable(${PROJECT_NAME}_node
    src/direction_indicator_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    direction_calculator
    state_machine
    tts_client
    ${catkin_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS direction_calculator state_machine tts_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" "*.hpp"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
