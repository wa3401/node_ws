cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros)


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


add_executable(Pure src/pure.cpp)
add_dependencies(Pure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Pure ${catkin_LIBRARIES})
# add dependancy for generate_messages_cpp if it doesnt work
add_dependencies(Pure ${catkin_EXPORTED_TARGETS})
