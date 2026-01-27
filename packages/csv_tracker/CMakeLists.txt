cmake_minimum_required(VERSION 3.1.2)
project(csv_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  std_srvs
  nodelet
  mrs_uav_managers
  mrs_lib
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp nodelet mrs_msgs mrs_uav_managers mrs_lib std_srvs # dynamic_reconfigure
  INCLUDE_DIRS src
  DEPENDS Eigen
  LIBRARIES CsvTracker
  )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(CsvTracker
  src/tracker_module.cpp
  )

add_dependencies(CsvTracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(CsvTracker
  ${catkin_LIBRARIES}
  )
