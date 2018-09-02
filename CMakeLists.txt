cmake_minimum_required(VERSION 2.8.3)
project(ros_tutorial_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)

add_message_files(FILES MsgTutorial.msg)

generate_messages(
    DEPENDENCIES
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    LIBRARIES ros_tutorials_topic
    CATKIN_DEPENDS roscpp std_msgs
)


add_executable(comm_tcp_server src/comm_tcp_server.cpp)
add_dependencies(comm_tcp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comm_tcp_server
  ${catkin_LIBRARIES}
)

add_executable(comm_tcp_client src/comm_tcp_client.cpp)
add_dependencies(comm_tcp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comm_tcp_client
  ${catkin_LIBRARIES}
)
