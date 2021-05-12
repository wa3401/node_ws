cmake_minimum_required(VERSION 3.0.2)
project(lab7)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf2_msgs)


add_message_files(
   FILES
   test.msg
 )
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(pathFinder src/pathFinder.cpp)
add_dependencies(pathFinder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathFinder ${catkin_LIBRARIES})