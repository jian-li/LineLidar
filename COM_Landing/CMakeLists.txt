cmake_minimum_required(VERSION 2.8.3)
project(COM_Landing)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  cleanBot_msg.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime)

set(SOURCE_FILES src/Serial.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include src
)
add_executable(COM_Landing src/main.cpp ${SOURCE_FILES})
add_dependencies(COM_Landing COM_Landing_generate_messages_cpp)
target_link_libraries(COM_Landing 
 ${catkin_LIBRARIES}
)

set(CMAKE_BUILD_TYPE Release)
