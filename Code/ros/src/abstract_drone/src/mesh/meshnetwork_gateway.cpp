#include "meshnetwork_gateway.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 void MeshnetworkGateway::Init( )
 {
  connectedToGateway = true;
  isGateway = true;
  prefferedGateWay = this->nodeID;

  ros::SubscribeOptions so = ros::SubscribeOptions::create<
      abstract_drone::RequestGatewayDroneFlight >(
      "/gateway", 1000,
      boost::bind( &MeshnetworkGateway::gatewayQueue, this, _1 ),
      ros::VoidPtr( ), &this->rosQueue );
  this->gatewaySub = this->rosNode->subscribe( so );
  common::Time::Sleep( 0.01 );

  routerTech->startRouting( );
 }

 void MeshnetworkGateway::gatewayQueue(
     const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg )
 {
  sendGoalToDrone( _msg->ID, _msg->longitude, _msg->latitude, _msg->height );
 }

 void MeshnetworkGateway::processIntroduction(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
 }

 void MeshnetworkGateway::CheckConnection( )
 {
  while ( this->rosNode->ok( ) ) {
   common::Time::Sleep( CheckConnectionTime );  // check every 10 seconds
   routerTech->maintainRouting( );
  }
 }

 void MeshnetworkGateway::ProcessHeartbeat(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::HeartbeatMessage msg( _msg->payload.data( ) );
  // ROS_INFO("RECIEVED A HEARTBEAT FROM %u", msg.getID( ));
  sendHeartbeat( msg.getID( ) );
 }

 void MeshnetworkGateway::processMovementNegotiationMessage(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  // Our gateway stay where he is. Maybe in the future we need to move the
  // gateway?
 }
}  // namespace Meshnetwork
}  // namespace gazebo