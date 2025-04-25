cmake_minimum_required(VERSION 3.5)
project(nexus_4wd_mecanum_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

 
include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
)

link_directories(${gazebo_dev_LIBRARY_DIRS})

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)
if(ENABLE_PROFILER)
  find_package(ignition-common3 QUIET COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()

add_library(nexus_ros_force_based_move SHARED
  src/nexus_ros_force_based_move.cpp
)

target_include_directories(nexus_ros_force_based_move PUBLIC include)

ament_target_dependencies(nexus_ros_force_based_move
  gazebo_dev
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_export_libraries(nexus_ros_force_based_move)
if(ENABLE_PROFILER)
  target_include_directories(nexus_ros_force_based_move PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(nexus_ros_force_based_move ${ignition-common3_LIBRARIES})
endif()

target_link_libraries(nexus_ros_force_based_move
  ${Boost_LIBRARIES}
  ${gazebo_dev_LIBRARIES}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)


ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    nexus_ros_force_based_move
    
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(NOT GAZEBO_VERSION VERSION_LESS 11.9)
install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
endif()
