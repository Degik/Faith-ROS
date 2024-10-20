cmake_minimum_required(VERSION 3.8)
project(ros_test1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)          # Python
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Node master
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
link_directories(/usr/lib/x86_64-linux-gnu)

# Node camera
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

# Python - node inference
install(
  PROGRAMS src/py/inference.py
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})