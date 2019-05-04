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
     , internet( new ros::Internet::RosInternetMock( *this ) )
 {
 }

 void MeshnetworkGateway::Init( )
 {
  internet->connect( );
  // Since we are a gateway this is always true. In the future we could base
  // this on the fact if we are connected to the internet
  connectedToGateway = true;
  // Would make no sense to connect to another gateway
  prefferedGateWay = this->nodeID;
  // Give the InternetComponent some time to boot
  // Begin with routing nearby nodes
  routerTech->startRouting( );
 }

 void MeshnetworkGateway::processIntroduction( const uint8_t* message )
 {
 }

 void MeshnetworkGateway::CheckConnection( )
 {
  while ( true ) {
   std::this_thread::sleep_for(
       std::chrono::seconds( CheckConnectionTime ) );  // check every 10 seconds
   routerTech->maintainRouting( );
  }
 }

 void MeshnetworkGateway::lostConnection( )
 {
  // We cant lose connection since we are the connection
 }

 void MeshnetworkGateway::ProcessHeartbeat( const uint8_t* message )
 {
  Messages::HeartbeatMessage msg( message );
  sendHeartbeat( msg.getCreator( ) );
 }

 void MeshnetworkGateway::processMovementNegotiationMessage(
     const uint8_t* message )
 {
  // In the current implementation gateways dont move
 }
 void MeshnetworkGateway::SendGoalRequestToDrone( const uint8_t ID,
                                                  const float latitude,
                                                  const float longitude,
                                                  const uint16_t height )
 {
  SendGoalToDrone( ID, latitude, longitude, height );
 }
}  // namespace Meshnetwork
}  // namespace Communication