cmake_minimum_required(VERSION 2.8)
project(flex_sync_test)

add_definitions(-std=c++17 -g -Wall)

find_package(catkin REQUIRED COMPONENTS roscpp flex_sync message_filters rosbag std_msgs sensor_msgs)

catkin_package(
CATKIN_DEPENDS roscpp flex_sync message_filters rosbag std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(sync_test src/sync_test.cpp)
add_dependencies(sync_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_test ${catkin_LIBRARIES})

add_executable(live_sync_test src/live_sync_test.cpp)
add_dependencies(live_sync_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(live_sync_test ${catkin_LIBRARIES})
