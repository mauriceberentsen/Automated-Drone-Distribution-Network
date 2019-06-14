cmake_minimum_required(VERSION 3.5.1)
project(DroneMeshnetwork)

set(CMAKE_CXX_STANDARD 11)

############################
##      SOURCE FILES      ##
############################
add_executable(${PROJECT_NAME}_router RaspberryRouter.cpp 
				      ../Communication/Messages/Message.cpp 
				      ../Communication/Meshnetwork/MeshnetworkComponent.cpp 
				      ../Communication/Meshnetwork/MeshnetworkRouter.cpp 
				      ../Communication/RoutingTechnique/HybridLMRoutingProtocol.cpp
 			          ../Drone/MockDroneEngine.cpp 
 				      ../NRF24/NRF24HighLevelInterface.cpp 
				      ../NRF24/NRF24LowLevelInterface.cpp)

add_executable(${PROJECT_NAME}_gateway RaspberryGateway.cpp 
				      ../Communication/Messages/Message.cpp 
				      ../Communication/Meshnetwork/MeshnetworkComponent.cpp 
				      ../Communication/Meshnetwork/MeshnetworkRouter.cpp 
				      ../Communication/RoutingTechnique/HybridLMRoutingProtocol.cpp
 			          ../Drone/MockDroneEngine.cpp 
 				      ../NRF24/NRF24HighLevelInterface.cpp 
					  ../NRF24/NRF24LowLevelInterface.cpp
					  ../RESTGateway/InternetGateway/InternetGateway.cpp)

#####################################
##      HEADERS SEARCH PATHS       ##
##################################### 
set(PISTACHE_INCLUDE_DIR "libs/pistache/include")
set(RF24_INCLUDE_DIR "/usr/local/include/RF24/..")
set(NORMAL_LIB "/usr/local/lib")

set(HEADER_SEARCH_PATHS  ${NORMAL_LIB} ${PISTACHE_INCLUDE_DIR} ${RF24_INCLUDE_DIR})

#####################################
##      LIBRARY SEARCH PATHS       ##
#####################################
set(PISTACHE_LIBRARY "pistache")
set(EXTRA_LIBRARY "-pthread -lssl -lrf24")

set(LIBRARIES_SEARCH_PATHS  ${EXTRA_LIBRARY})

#######################################
##      ADDING HEADERS LIBRARY       ##
#######################################
include_directories(${HEADER_SEARCH_PATHS})
target_link_libraries(${PROJECT_NAME}_router ${LIBRARIES_SEARCH_PATHS})

#g++ -Ofast -Wall -pthread  -I. -I.. -L  -lrf24 -o Router

