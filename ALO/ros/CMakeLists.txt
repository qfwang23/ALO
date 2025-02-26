
cmake_minimum_required(VERSION 3.16...3.26)
project(ALO VERSION 0.3.0 LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/ALO/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/ALO ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)
endif()

if("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "KISS-ICP ROS 1 wrapper will be compiled")
  find_package(
    catkin REQUIRED
    COMPONENTS geometry_msgs
               nav_msgs
               sensor_msgs
               geometry_msgs
               roscpp
               rosbag
               std_msgs
               tf2
               tf2_ros)
  catkin_package()

  # ROS 1 node
  add_executable(odometry_node ros1/OdometryServer.cpp)
  target_compile_features(odometry_node PUBLIC cxx_std_20)
  target_include_directories(odometry_node PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(odometry_node ALO::pipeline ${catkin_LIBRARIES})
  install(TARGETS odometry_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  message(STATUS "KISS-ICP ROS 2 wrapper will be compiled")

  find_package(ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  rclcpp_components_register_node(odometry_component PLUGIN "kiss_icp_ros::OdometryServer" EXECUTABLE odometry_node)

  install(TARGETS odometry_component LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

  ament_package()

else()
  message(FATAL_ERROR "catkin or colcon not found KISS-ICP-ROS disabled")
endif()
