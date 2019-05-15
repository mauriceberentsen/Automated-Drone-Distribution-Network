cmake_minimum_required(VERSION 2.8.3)
project(drone_meshnetwork_simulation)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-std=c++14 -Wall -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  gazebo_ros 
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Location.msg
   NRF24.msg
   DroneInfo.msg
   NodeDebugInfo.msg
   RequestGatewayDroneFlight.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AreaScan.srv
   CasusRequest.srv
   PowerSwitch.srv
   RequestDroneFlight.srv
   RequestGPS.srv
   WirelessMessage.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

#############################
## Declare ROS executables ##
#############################

add_executable(DroneManager src/ros/DroneManagerService/DroneManagerService.cpp src/ros/DroneManagerService/DroneManager.cpp)
target_link_libraries(DroneManager ${catkin_LIBRARIES})
add_dependencies(DroneManager drone_meshnetwork_simulation_gencpp drone_meshnetwork_simulation_generate_messages_cpp)

add_executable(WirelessSignalSimulator src/ros/WirelessSimulation/WirelessSignalSimulatorService.cpp src/ros/WirelessSimulation/WirelessSignalSimulator.cpp src/ros/WirelessSimulation/Node.cpp)
target_link_libraries(WirelessSignalSimulator ${catkin_LIBRARIES})
add_dependencies(WirelessSignalSimulator drone_meshnetwork_simulation_gencpp drone_meshnetwork_simulation_generate_messages_cpp)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES drone_meshnetwork_simulation
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
   DEPENDS roscpp gazebo_ros
)

#############
## Gazebo  ##
#############

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(MeshnetworkRouter SHARED       src/gazebo/Arduino/VirtualArduinoRouter.cpp
                                           src/Communication/Meshnetwork/MeshnetworkComponent.cpp
                                           src/Communication/Meshnetwork/MeshnetworkRouter.cpp
                                           src/Communication/RoutingTechnique/HybridLMRoutingProtocol.cpp
                                           src/Communication/Messages/Message.cpp
                                           src/ros/RosDroneEngineConnector.cpp
                                           src/ros/WirelessSimulation/VirtualNRF24.cpp)
target_link_libraries(MeshnetworkRouter ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(MeshnetworkRouter drone_meshnetwork_simulation_generate_messages_cpp)

add_library(MeshnetworkGateway SHARED src/gazebo/Arduino/VirtualArduinoGateway.cpp
                                      src/Communication/Meshnetwork/MeshnetworkComponent.cpp
                                      src/Communication/Meshnetwork/MeshnetworkGateway.cpp
                                      src/Communication/RoutingTechnique/HybridLMRoutingProtocol.cpp
                                      src/Communication/Messages/Message.cpp
                                      src/ros/WirelessSimulation/VirtualNRF24.cpp
                                      src/ros/RosDroneEngineConnector.cpp
                                      src/ros/RosInternetMock.cpp)
target_link_libraries(MeshnetworkGateway ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(MeshnetworkGateway drone_meshnetwork_simulation_generate_messages_cpp)

add_library(DroneEngine SHARED src/gazebo/DroneSimulation/DroneEngine.cpp)
target_link_libraries(DroneEngine ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(DroneEngine drone_meshnetwork_simulation_generate_messages_cpp)

add_library(DroneFactory SHARED src/gazebo/DroneFactory.cpp
                                src/gazebo/DroneSimulation/VirtualDrone.cpp)
target_link_libraries(DroneFactory  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(DroneFactory drone_meshnetwork_simulation_generate_messages_cpp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-HybridLMRoutingProtocol-test src/Communication/RoutingTechnique/tests/utestChildTableTree.cpp
                                                     src/Communication/RoutingTechnique/HybridLMRoutingProtocol.cpp)

catkin_add_gtest(${PROJECT_NAME}-GatewayDrone-test src/Communication/Messages/Tests/utestMessage.cpp
                                                     src/Communication/Messages/Message.cpp
)
# catkin_add_gtest(${PROJECT_NAME}-MeshRouter-test src/Communication/Meshnetwork/tests/utestMeshRouter.cpp)

# if(TARGET ${PROJECT_NAME}-MeshRouter-test)
#   target_link_libraries(${PROJECT_NAME}-MeshRouter-test WirelessSignalSimulator MeshnetworkRouter ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
# endif()




