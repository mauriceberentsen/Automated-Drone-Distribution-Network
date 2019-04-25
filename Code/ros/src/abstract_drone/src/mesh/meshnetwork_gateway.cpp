/**
 * @file meshnetwork_gateway.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for the MeshnetworkGateway
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "meshnetwork_gateway.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 MeshnetworkGateway::MeshnetworkGateway( ){

 };
 void MeshnetworkGateway::Init( )
 {
  connectedToGateway = true;
  prefferedGateWay = this->nodeID;

  //   ros::SubscribeOptions so = ros::SubscribeOptions::create<
  //       abstract_drone::RequestGatewayDroneFlight >(
  //       "/gateway", 1000,
  //       boost::bind( &MeshnetworkGateway::gatewayQueue, this, _1 ),
  //       ros::VoidPtr( ), &this->rosQueue );
  //   this->gatewaySub = this->rosNode->subscribe( so );
  common::Time::Sleep( 0.01 );

  routerTech->startRouting( );
 }

 //  void MeshnetworkGateway::gatewayQueue(
 //      const abstract_drone::RequestGatewayDroneFlightConstPtr& _msg )
 //  {
 //   sendGoalToDrone( _msg->ID, _msg->longitude, _msg->latitude, _msg->height
 //   );
 //  }

 void MeshnetworkGateway::processIntroduction( const uint8_t* message )
 {
 }

 void MeshnetworkGateway::CheckConnection( )
 {
  while ( true ) {                              // this->rosNode->ok( ) ) {
   common::Time::Sleep( CheckConnectionTime );  // check every 10 seconds
   routerTech->maintainRouting( );
  }
 }

 void MeshnetworkGateway::lostConnection( )
 {
 }

 void MeshnetworkGateway::ProcessHeartbeat( const uint8_t* message )
 {
  Messages::HeartbeatMessage msg( message );
  // ROS_INFO("RECIEVED A HEARTBEAT FROM %u", msg.getID( ));
  sendHeartbeat( msg.getID( ) );
 }

 void MeshnetworkGateway::processMovementNegotiationMessage(
     const uint8_t* message )
 {
 }
}  // namespace Meshnetwork
}  // namespace gazebo