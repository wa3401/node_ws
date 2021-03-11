cmake_minimum_required(VERSION 3.0.2)
project(lab2)

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


add_executable(AEB src/AEB.cpp)
add_dependencies(AEB ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AEB ${catkin_LIBRARIES})
# add dependancy for generate_messages_cpp if it doesnt work
add_dependencies(AEB ${catkin_EXPORTED_TARGETS})