/**
 * @file MeshnetworkGateway.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for the MeshnetworkGateway
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "MeshnetworkGateway.hpp"

namespace Communication
{
namespace Meshnetwork
{
 MeshnetworkGateway::MeshnetworkGateway( const uint8_t node,
                                         const uint8_t drone,
                                         bool developermode )
     : MeshnetworkComponent( node, drone, developermode )
     , internet( new ros::RosInternetMock( *this ) )
 {
 }

 void MeshnetworkGateway::Init( )
 {
  connectedToGateway = true;
  prefferedGateWay = this->nodeID;
  std::this_thread::sleep_for( std::chrono::microseconds( initTime ) );
  internet->connect( );
  routerTech->startRouting( );
 }

 void MeshnetworkGateway::processIntroduction( const uint8_t* message )
 {
 }

 void MeshnetworkGateway::CheckConnection( )
 {
  while ( true ) {  // this->rosNode->ok( ) ) {
   std::this_thread::sleep_for(
       std::chrono::seconds( CheckConnectionTime ) );  // check every 10 seconds
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
}  // namespace Communication