cmake_minimum_required(VERSION 2.8.3)
project(cv)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

add_message_files( 
    FILES
    blob.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

include directories(
  ${catkin_INCLUDE_DIRS}
)
