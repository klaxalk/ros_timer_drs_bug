cmake_minimum_required(VERSION 3.16)
project(timer_tester)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  config/timer_tester.cfg
  )

catkin_package(
  LIBRARIES TimerTester
  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# libraries

add_library(TimerTester src/timer_tester.cpp)

add_dependencies(TimerTester
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(TimerTester
  ${catkin_LIBRARIES}
  )
