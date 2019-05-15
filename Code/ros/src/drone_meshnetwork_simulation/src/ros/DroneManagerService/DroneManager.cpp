/**
 * @file DroneManager.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for DroneManagerService
 * @version 1.0
 * @date 2019-04-05
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "drone_meshnetwork_simulation/RequestGatewayDroneFlight.h"
#include "DroneManager.hpp"
namespace ros
{
namespace DroneManagerService
{
 DroneManager::DroneManager( std::shared_ptr< ros::NodeHandle > &_rosNode,
                             std::string &GatewayTopicName )
     : nodeHandle( _rosNode ), GatewayTopic( GatewayTopicName )
 {
  internet = this->nodeHandle->advertise<
      drone_meshnetwork_simulation::RequestGatewayDroneFlight >( GatewayTopic,
                                                                 100 );
 }
 DroneManager::~DroneManager( ){};

 bool DroneManager::RequestMovement( uint8_t ID, float latitude,
                                     float longitude, uint16_t height )
 {
  drone_meshnetwork_simulation::RequestGatewayDroneFlight msg;
  msg.ID = ID;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.height = height;
  while ( internet.getNumSubscribers( ) == 0 ) {
   internet.publish( msg );
  }
  internet.publish( msg );
  return true;
 }

 bool DroneManager::RequestMovement(
     drone_meshnetwork_simulation::RequestDroneFlight::Request &req,
     drone_meshnetwork_simulation::RequestDroneFlight::Response &res )
 {
  drone_meshnetwork_simulation::RequestGatewayDroneFlight msg;
  msg.ID = req.ID;
  msg.latitude = req.latitude;
  msg.longitude = req.longitude;
  msg.height = req.height;
  while ( internet.getNumSubscribers( ) == 0 ) {
   internet.publish( msg );
  }
  internet.publish( msg );
  return true;
 }

 bool DroneManager::setDronesToCasus(
     drone_meshnetwork_simulation::CasusRequest::Request &req,
     drone_meshnetwork_simulation::CasusRequest::Response &res )
 {
  switch ( req.caseID ) {
   case 1:
    return RequestMovement( 1, -13, 24 );

    break;
   case 2:
    return RequestMovement( 2, -11, 20 );

    break;
   case 3:
    return RequestMovement( 3, -12, 14 );

    break;
   case 4:
    return RequestMovement( 4, -7, 16 );

    break;
   case 5:
    return RequestMovement( 5, -12, 7 );

    break;
   case 6:
    return RequestMovement( 6, -7, 12 );

    break;
   case 7:
    return RequestMovement( 7, -9, 5 );

    break;
   case 8:
    return RequestMovement( 8, -5, 10 );

    break;
   case 9:
    return RequestMovement( 9, -5, 5 );

    break;
   case 10:
    return RequestMovement( 10, 4, 9 );

    break;
   case 11:
    return RequestMovement( 11, 5, 5 );

    break;
   case 12:
    return RequestMovement( 12, 7, 13 );

    break;
   case 13:
    return RequestMovement( 13, 9, 7 );

    break;
   case 14:
    return RequestMovement( 14, 8, 20 );

    break;
   case 15:
    return RequestMovement( 15, 5, 25 );

    break;
   case 16:
    return RequestMovement( 16, 11, 25 );

    break;
   case 17:
    return RequestMovement( 17, 8, 27 );

    break;
   case 100:
    for ( int i = 1; i < 11; i++ ) {
     for ( int j = 0; j < 10; j++ ) {
      RequestMovement( i + j, i * 5, j * 5 );
     }
    }
    return true;

    break;
   case 101:
    RequestMovement( 1, -3, 4 );
    RequestMovement( 2, 3, 4 );
    RequestMovement( 3, -3, 10 );
    RequestMovement( 4, 3, 10 );
    RequestMovement( 5, -3, 16 );
    RequestMovement( 6, 3, 16 );
    return true;

    break;
       case 102:
    RequestMovement( 1, -3, 4 );
    RequestMovement( 2, 3, 4 );
    RequestMovement( 3, 9, 4 );

    RequestMovement( 4, -3, 10 );
    RequestMovement( 5, 3, 10 );
    RequestMovement( 6, 9, 10 );

    RequestMovement( 7, -3, 16 );
    RequestMovement( 8, 3, 16 );
    RequestMovement( 9, 9, 16 );

    RequestMovement( 10, -3, 22 );
    RequestMovement( 11, 3, 22 );
    RequestMovement( 12, 9, 22 );
    return true;

    break;
   case 0:
    return RequestMovement( 1, -13, 24 )

           && RequestMovement( 2, -11, 20 )

           && RequestMovement( 3, -12, 14 )

           && RequestMovement( 4, -7, 16 )

           && RequestMovement( 5, -12, 7 )

           && RequestMovement( 6, -7, 12 )

           && RequestMovement( 7, -9, 5 )

           && RequestMovement( 8, -5, 10 )

           && RequestMovement( 9, -5, 5 )

           && RequestMovement( 10, 4, 9 )

           && RequestMovement( 11, 5, 5 )

           && RequestMovement( 12, 7, 13 )

           && RequestMovement( 13, 9, 7 )

           && RequestMovement( 14, 8, 20 )

           && RequestMovement( 15, 5, 25 )

           && RequestMovement( 16, 11, 25 )

           && RequestMovement( 17, 8, 26.5 );
    break;
   default:
    for ( int i = 1; i < req.caseID + 1; i++ ) {
     int row, col;
     row = i % 2;
     col = std::floor( i / 2 );
     ROS_INFO( "REQUEST FOR %d", i );
     RequestMovement( i, row * 5, col * 5 );
    }
    return true;
  }
 }
}  // namespace DroneManagerService
}  // namespace ros