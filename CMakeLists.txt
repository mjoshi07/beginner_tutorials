cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  message_generation
  genmsg
)

## Generate services in the "srv" directory
add_service_files(
  FILES
  custom_string.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


###########
## Build ##
###########

include_directories(include  ${catkin_INCLUDE_DIRS})

# Declare a cpp executable
add_executable(talker_node src/talker.cpp)

# Specify libraries to link a library of executuable target against
target_link_libraries(talker_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(talker_node beginner_tutorials_generate_messages_cpp)

# Declare a cpp executable
add_executable(listener_node src/listener.cpp)

# Specify libraries to link a library of executuable target against
target_link_libraries(listener_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(listener_node beginner_tutorials_generate_messages_cpp)


# Add rostest gtest 
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
  test_pub
  test/test_pub.test
  test/test_pub.cpp
  )
  target_link_libraries(
    test_pub
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    test_pub
    talker_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()
