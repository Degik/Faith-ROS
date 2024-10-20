cmake_minimum_required(VERSION 3.8)
project(faith_ros)

# C++ 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${TIFF_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/class
)

# Build C++ nodes

## Node master
add_executable(master
  src/master.cpp
  src/class/VideoPublisher.cpp
)
ament_target_dependencies(master
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(master
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
  ${TIFF_LIBRARIES}
)
install(TARGETS master DESTINATION lib/${PROJECT_NAME})

## Node camera
add_executable(camera
  src/camera.cpp
  src/class/VideoPublisher.cpp
)
ament_target_dependencies(camera
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(camera
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
  ${TIFF_LIBRARIES}
)
install(TARGETS camera DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(
  PROGRAMS src/py/inference.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inference
)

ament_package()
