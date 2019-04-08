#include "DroneManager.hpp"
#include "abstract_drone/RequestGatewayDroneFlight.h"

DroneManager::DroneManager( std::shared_ptr< ros::NodeHandle > _rosNode,
                            std::string GatewayTopicName )
    : nodeHandle( _rosNode ), GatewayTopic( GatewayTopicName )
{
 rosPub =
     this->nodeHandle->advertise< abstract_drone::RequestGatewayDroneFlight >(
         GatewayTopic, 100 );
}
DroneManager::~DroneManager( ){};

bool DroneManager::RequestMovement( uint8_t ID, float longitude, float latitude,
                                    uint16_t height )
{
 abstract_drone::RequestGatewayDroneFlight msg;
 msg.ID = ID;
 msg.longitude = longitude;
 msg.latitude = latitude;
 msg.height = height;
 rosPub.publish( msg );
}

bool DroneManager::RequestMovement(
    abstract_drone::RequestDroneFlight::Request &req,
    abstract_drone::RequestDroneFlight::Response &res )
{
 abstract_drone::RequestGatewayDroneFlight msg;
 msg.ID = req.ID;
 msg.longitude = req.longitude;
 msg.latitude = req.latitude;
 msg.height = req.height;
 rosPub.publish( msg );
 return true;
}

bool DroneManager::setDronesToCasus(
    abstract_drone::RequestDroneFlight::Request &req,
    abstract_drone::RequestDroneFlight::Response &res )
{
 switch ( req.ID ) {
  case 1:
   RequestMovement( 1, -13, 24 );

   break;
  case 2:
   RequestMovement( 2, -11, 20 );

   break;
  case 3:
   RequestMovement( 3, -12, 14 );

   break;
  case 4:
   RequestMovement( 4, -7, 16 );

   break;
  case 5:
   RequestMovement( 5, -12, 7 );

   break;
  case 6:
   RequestMovement( 6, -7, 12 );

   break;
  case 7:
   RequestMovement( 7, -9, 5 );

   break;
  case 8:
   RequestMovement( 8, -5, 10 );

   break;
  case 9:
   RequestMovement( 9, -5, 5 );

   break;
  case 10:
   RequestMovement( 10, 4, 9 );

   break;
  case 11:
   RequestMovement( 11, 5, 5 );

   break;
  case 12:
   RequestMovement( 12, 7, 13 );

   break;
  case 13:
   RequestMovement( 13, 9, 7 );

   break;
  case 14:
   RequestMovement( 14, 8, 20 );

   break;
  case 15:
   RequestMovement( 15, 5, 25 );

   break;
  case 16:
   RequestMovement( 16, 11, 25 );

   break;
  case 17:
   RequestMovement( 17, 8, 27 );

   break;

  default:
   RequestMovement( 1, -13, 24 );

   RequestMovement( 2, -11, 20 );

   RequestMovement( 3, -12, 14 );

   RequestMovement( 4, -7, 16 );

   RequestMovement( 5, -12, 7 );

   RequestMovement( 6, -7, 12 );

   RequestMovement( 7, -9, 5 );

   RequestMovement( 8, -5, 10 );

   RequestMovement( 9, -5, 5 );

   RequestMovement( 10, 4, 9 );

   RequestMovement( 11, 5, 5 );

   RequestMovement( 12, 7, 13 );

   RequestMovement( 13, 9, 7 );

   RequestMovement( 14, 8, 20 );

   RequestMovement( 15, 5, 25 );

   RequestMovement( 16, 11, 25 );

   RequestMovement( 17, 8, 27 );
   break;
 }
 return true;
}