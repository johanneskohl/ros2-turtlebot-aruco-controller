cmake_minimum_required(VERSION 3.8)
project(turtlebot_aruco_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(TAC_DEBUG "Enable or disable debug build" OFF)
if (TAC_DEBUG)
  set(_debug_definition "TAC_DEBUG")
else()
  set(_debug_definition "")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(main)
target_link_libraries(main
  PUBLIC
    ${OpenCV_LIBRARIES}
)
ament_target_dependencies(main
  PUBLIC
    rclcpp rclcpp_lifecycle geometry_msgs
)
target_compile_definitions(main
  PUBLIC
    ${_debug_definition}
)
target_compile_options(main
  PUBLIC
    -Wall -Wextra -Wpedantic -Werror
)

add_executable(make-marker)
target_link_libraries(make-marker
  PUBLIC
    ${OpenCV_LIBRARIES}
)
target_compile_options(make-marker
  PUBLIC
    -Wall -Wextra -Wpedantic -Werror
)

add_subdirectory(src)

install(
  TARGETS main make-marker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
