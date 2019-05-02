/**
 * @file MeshnetworkRouter.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for MeshnetworkRouter
 * @version 1.0
 * @date 2019-04-19
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <thread>  // std::this_thread::sleep_for
#include <chrono>  // std::chrono::seconds

#include "MeshnetworkRouter.hpp"

namespace Communication
{
namespace Meshnetwork
{
 MeshnetworkRouter::MeshnetworkRouter( const uint8_t node, const uint8_t drone,
                                       bool developermode )
     : MeshnetworkComponent( node, drone, developermode )
 {
 }
 // Called after load
 void MeshnetworkRouter::Init( )
 {
  std::this_thread::sleep_for( std::chrono::microseconds( initTime ) );
  routerTech->startRouting( );
 }

 void MeshnetworkRouter::processIntroduction( const uint8_t *message )
 {
  Messages::IntroduceMessage introduce( message );
  if ( introduce.getKnowGateway( ) &&
       introduce.getHopsUntilGateway( ) < this->hopsFromGatewayAway &&
       introduce.getCreator( ) != lastGoodKnownLocation.getCreator( ) ) {
   requestLocation( introduce.getCreator( ) );
  }
 }

 void MeshnetworkRouter::ProcessHeartbeat( const uint8_t *message )
 {
  Messages::HeartbeatMessage msg( message );
  // if sender is gateway flip bool and return
  if ( msg.getIsGateway( ) ) {
   prefferedGateWay = msg.getPrefferedGateway( );
   connectedToGateway = true;
   hopsFromGatewayAway =
       msg.getHops( ) + 1;  // count our own hop towards gateway
   return;
  }
  // if sender doesn't know gateway but we do
  if ( !msg.getKnowGateway( ) && connectedToGateway ) {
   // send a heartbeat back to inform him we know a gateway
   sendHeartbeat( msg.getCreator( ) );
  }
  // if sender knows gateway and we don't
  else if ( msg.getKnowGateway( ) && !connectedToGateway ) {
   routerTech->OtherCanCommunicateWithNode( msg.getCreator( ),
                                            msg.getPrefferedGateway( ) );
   // Set his gateway as prefferedGateWay and try to contact that
   // gateway
   prefferedGateWay = msg.getPrefferedGateway( );
   sendHeartbeatToGateway( );
  }
 }

 void MeshnetworkRouter::CheckConnection( )
 {
  while ( true )  // {this->rosNode->ok( ) )
  {
   if ( !communication->On( ) ) {
    connectedToGateway = false;
    continue;
   }
   std::this_thread::sleep_for(
       std::chrono::seconds( CheckConnectionTime ) );  // check every 10
   // seconds
   routerTech->maintainRouting( );
   searchOtherNodesInRange( );  // maybe there is someone close
   if ( connectedToGateway ) {
    if ( !knowPrefferedGatewayLocation ) {
     requestLocation( prefferedGateWay );
    }
    sendHeartbeatToGateway( );
   } else
    lostConnection( );
  }
 }

 void MeshnetworkRouter::sendHeartbeatToGateway( )
 {
  if ( !sendHeartbeat( prefferedGateWay ) ) {}
  // this we be flipped back by response of the gateway
  connectedToGateway = false;
 }
 void MeshnetworkRouter::lostConnection( )
 {
  if ( connectedToGateway ) {
   timerStarted = false;
   this->lastTimeOnline = std::clock( );
   return;
  }
  if ( !timerStarted ) {
   routerTech->cantCommunicateWithNode( prefferedGateWay );
   informAboutMissingChild( this->nodeID, prefferedGateWay );
   this->lastTimeOnline = std::clock( );
   timerStarted = true;
  } else if ( ( std::clock( ) - this->lastTimeOnline ) /
                  ( double )CLOCKS_PER_SEC >
              timeUntilConnectionLost ) {
   StartEmergencyProtocol( );
   timerStarted = false;
  }
 }

 void MeshnetworkRouter::StartEmergencyProtocol( )
 {
  if ( connectedToGateway )  // in case we meanwhile found a connection
  {
   return;
  }
  if ( routerTech->empty( ) ) {
   sendGoalToEngine( lastGoodKnownLocation );
   routerTech->NodeMovedLocation( );
   lastGoodKnownLocation = prefferedGateWayLocation;
  } else {
   // start negotiation about who needs to move
   startMovementNegotiation( );
  }
 }

 void MeshnetworkRouter::startMovementNegotiation( )
 {
  static float myDistance;
  // Find out which who the greatest distance from the GATEWAY. He shall be
  // choosen to replace the missing node.
  // Before we go to action we first make a list of everybody near that is
  // disconnected.

  // First find out how far away you are
  if ( routerTech->getDirectionToNode( lastGoodKnownLocation.getCreator( ) ) ==
       UINT8_MAX ) {
   myDistance = distanceBetweenMeAndLocation( prefferedGateWayLocation );
  } else {
   myDistance = -1;
  }

  SafeAddToNegotiationList( std::make_pair( myDistance, this->nodeID ) );

  // add ourself to the list
  // tell others how far away we are
  informOthersAboutCost( myDistance );
  while ( negotiationList.size( ) - 1 < routerTech->getAmountOfChildren( ) ) {
   // wait for others to respond
   continue;
  }
  // now it is time for action

  if ( myDistance >= 0 && negotiationList.rbegin( )->second == this->nodeID ) {
   sendGoalToEngine( lastGoodKnownLocation );
   routerTech->NodeMovedLocation( );
   lastGoodKnownLocation = prefferedGateWayLocation;
   negotiationList.clear( );
  } else {
   negotiationList.clear( );
  }
 }
 void MeshnetworkRouter::SafeAddToNegotiationList(
     const std::pair< float, uint8_t > &val )
 {
  mtx.lock( );
  negotiationList.insert( val );
  mtx.unlock( );
 }

 void MeshnetworkRouter::informOthersAboutCost( float cost )
 {
  for ( auto &other : routerTech->getSetOfChildren( ) ) {
   uint8_t towards = routerTech->getDirectionToNode( other );
   if ( towards == UINT8_MAX ) { continue; }

   Messages::MovementNegotiationMessage MovNegotiationMsg(
       this->nodeID, this->nodeID, towards, towards, cost );
   uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
   MovNegotiationMsg.toPayload( buffer );
   SendMessage( buffer, towards );
  }
 }

 void MeshnetworkRouter::processMovementNegotiationMessage(
     const uint8_t *message )
 {
  Messages::MovementNegotiationMessage msg( message );
  SafeAddToNegotiationList(
      std::make_pair( msg.getCost( ), msg.getCreator( ) ) );
 }
}  // namespace Meshnetwork
}  // namespace Communication