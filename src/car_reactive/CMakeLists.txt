cmake_minimum_required(VERSION 3.0.2)
project(car_reactive)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(carReactive src/react.cpp)
add_dependencies(carReactive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(carReactive ${catkin_LIBRARIES})
# add dependancy for generate_messages_cpp if it doesnt work
add_dependencies(carReactive ${catkin_EXPORTED_TARGETS})