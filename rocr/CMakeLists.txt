cmake_minimum_required(VERSION 2.8.3)
project(rocr)

find_package(catkin REQUIRED COMPONENTS
  urdf
  roscpp
  geometry_msgs
  rostest
)


catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(rocr_test launch/launch_test.launch test/main.cpp test/rocr_dummy_test.cpp)
  target_link_libraries(rocr_test ${catkin_LIBRARIES})
  add_dependencies(rocr_test teleop_twist_keyboard ${catkin_EXPORTED_TARGETS})
 endif()

