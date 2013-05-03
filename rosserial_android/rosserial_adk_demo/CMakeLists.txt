cmake_minimum_required(VERSION 2.4.6)
project(ADK_DEMO C CXX)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

include_directories(src/AndroidAccessory src/USB_Host_Shield)
file(GLOB ADK_SRC src/AndroidAccessory/AndroidAccessory.cpp src/USB_Host_Shield/*.cpp)

set(CHIP_TYPE mega2560)
#====================================================================#
#  pubsub                                                            #
#====================================================================#

set(pubsub_BOARD ${CHIP_TYPE})               # Arduino Target board
set(pubsub_SRCS src/pubsub.cpp          
					${ADK_SRC} )  
set(pubsub_PORT /dev/ttyUSB0)            # Serial upload port
generate_ros_firmware(pubsub)

#====================================================================#
#  ADK Demo Firmware                                                     #
#====================================================================#
set(adk_demo_BOARD ${CHIP_TYPE})               # Arduino Target board
set(adk_demo_SRCS src/adk_demo.cpp          
					${ADK_SRC} )  
set(adk_demo_PORT /dev/ttyUSB0)            # Serial upload port
generate_ros_firmware(adk_demo)

