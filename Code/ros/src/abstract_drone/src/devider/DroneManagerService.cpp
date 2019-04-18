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
 DroneManagerService::DroneManager DM( rosNode, gatewayName );
 ros::ServiceServer sendToLocationService = rosNode->advertiseService(
     "requestDroneMovement",
     &DroneManagerService::DroneManager::RequestMovement, &DM );
 ros::ServiceServer setToCasusService = rosNode->advertiseService(
     "SetToCasus", &DroneManagerService::DroneManager::setDronesToCasus, &DM );
 ROS_INFO( "Drone manager online" );
 ros::spin( );

 return 0;
}