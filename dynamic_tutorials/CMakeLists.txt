cmake_minimum_required(VERSION 3.5)
project(dynamic_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)

add_executable(${PROJECT_NAME}_node src/server.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()