cmake_minimum_required(VERSION 3.8)
project(heron_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(heron_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/controller.cpp src/force_compensator.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  control_toolbox
  nav_msgs
  geometry_msgs
  heron_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)
set_target_properties(${PROJECT_NAME}_node
                      PROPERTIES OUTPUT_NAME controller
                      PREFIX "")

ament_export_include_directories(include)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
