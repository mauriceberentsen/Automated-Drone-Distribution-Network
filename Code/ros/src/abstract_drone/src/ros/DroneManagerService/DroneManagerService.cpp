/**
 * @file DroneManagerService.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file booting a service to manage drones
 * @version 1.0
 * @date 2019-04-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "abstract_drone/RequestDroneFlight.h"

#include "DroneManager.hpp"

int main( int argc, char **argv )
{
 ros::init( argc, argv, "DroneManager" );
 std::shared_ptr< ros::NodeHandle > rosNode;
 std::string gatewayName = "/gateway";
 rosNode.reset( new ros::NodeHandle( "DroneManager" ) );
 ros::DroneManagerService::DroneManager DM( rosNode, gatewayName );
 ros::ServiceServer sendToLocationService = rosNode->advertiseService(
     "requestDroneMovement",
     &ros::DroneManagerService::DroneManager::RequestMovement, &DM );
 ros::ServiceServer setToCasusService = rosNode->advertiseService(
     "SetToCasus", &ros::DroneManagerService::DroneManager::setDronesToCasus,
     &DM );
 ROS_INFO( "Drone manager online" );
 ros::spin( );

 return 0;
}