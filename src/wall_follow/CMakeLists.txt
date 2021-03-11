cmake_minimum_required(VERSION 3.0.2)
project(wall_follow)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation)


# generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
#  )
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(wall_follow src/wall_follow.cpp)
add_dependencies(AEB ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_follow ${catkin_LIBRARIES})
# add dependancy for generate_messages_cpp if it doesnt work
add_dependencies(AEB ${catkin_EXPORTED_TARGETS})