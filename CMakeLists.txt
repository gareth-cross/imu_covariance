cmake_minimum_required(VERSION 2.8.3)
project(imu_covariance)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

add_executable(${PROJECT_NAME}
    src/node.cpp
    src/main.cpp
    )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
