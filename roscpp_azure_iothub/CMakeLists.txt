# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(roscpp_azure_iothub)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if (WIN32)
  find_package(azure_c_shared_utility REQUIRED)
  find_package(azure_iot_sdks REQUIRED)
  find_package(umqtt REQUIRED)
  find_package(uamqp REQUIRED)  
else (WIN32)
  set(IOTHUB_CLIENT_INCLUDES ".." "/usr/include/azureiot" "/usr/include/azureiot/inc" "./parson")
  add_library(parson STATIC
    parson/parson.c
    parson/parson.h
  )
endif()

add_executable(${PROJECT_NAME}_node src/ros_azure_iothub_cpp_node.cpp)

include_directories(include ${IOTHUB_CLIENT_INCLUDES})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  std_msgs
)

if (WIN32)
  target_link_libraries(${PROJECT_NAME}_node
    iothub_client
    iothub_client_mqtt_ws_transport
  )
else (WIN32)
  target_link_libraries(${PROJECT_NAME}_node
    iothub_client
    iothub_client_mqtt_transport
    umqtt
    aziotsharedutil
    pthread
    curl
    ssl
    crypto
    m
    uuid
    parson
    prov_auth_client
    hsm_security_client
    uhttp
  )
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

#############
##  Test   ##
#############
#if(WIN32)
#  if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest_gtest(${PROJECT_NAME}_testnode test/testiothub.launch test/testiothub.cpp)
#    target_link_libraries(${PROJECT_NAME}_testnode iothub_service_client ${catkin_LIBRARIES} )
#  endif()
#endif()

ament_package()
