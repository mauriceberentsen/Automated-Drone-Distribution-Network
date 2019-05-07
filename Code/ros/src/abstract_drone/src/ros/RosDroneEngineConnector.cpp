/**
 * @file RosDroneEngineConnector.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for RosDroneEngineConnector
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "abstract_drone/Location.h"
#include "abstract_drone/RequestGPS.h"

#include "RosDroneEngineConnector.hpp"
namespace ros
{
namespace Drone
{
 RosDroneEngineConnector::RosDroneEngineConnector( uint16_t droneID )
     : ID( droneID )
 {
  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "node", ros::init_options::NoSigintHandler );
  }
  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset( new ros::NodeHandle( "droneEngineConnector" ) );
  std::string connectedEngine =
      "/Drones/" + std::to_string( this->ID ) + "/goal";
  std::string gps_ServiceName =
      "/Drones/" + std::to_string( this->ID ) + "/gps";

  this->droneEngine = this->rosNode->advertise< abstract_drone::Location >(
      connectedEngine, 100 );
  ROS_INFO( "Loaded DroneEngineconnector connected to topicname:[%s]",
            gps_ServiceName.c_str( ) );
  // Connect to the GPS service
  this->GPSLink = this->rosNode->serviceClient< abstract_drone::RequestGPS >(
      gps_ServiceName.c_str( ) );
 }

 void RosDroneEngineConnector::setGoal( const float latitude,
                                        const float longitude,
                                        const float height )
 {
  abstract_drone::Location msg;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.height = height;

  droneEngine.publish( msg );
 }

 const ignition::math::Vector3< float > RosDroneEngineConnector::getLocation( )
 {
  abstract_drone::RequestGPS GPS;
  {
   if ( this->GPSLink.call( GPS ) ) {
    ignition::math::Vector3< float > loc(
        GPS.response.longitude, GPS.response.latitude, GPS.response.height );
    return loc;
   } else {
    ignition::math::Vector3< float > loc( 6, 6, 6 );
    // TODO Come up with some beter implementation when no gps service is
    // avaiable
    return loc;
   }
  }
 }

 RosDroneEngineConnector::~RosDroneEngineConnector( )
 {
 }
}  // namespace Drone
}  // namespace ros
