# Samples for the Azure IoT Hub connector

These samples demostrate how to use the various features of Microsoft Azure IoT Hub service to relay telemetry messages or dynamic reconfiguration commands in ROS nodes.

## build
colcon --log-level debug build --merge-install --event-handlers console_cohesion+ --cmake-target install --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

## List of samples

* [IOT Hub connector sample in C++](./roscpp_azure_iothub/)

* [Dynamic reconfiguration sample](./dynamic_tutorials/)
