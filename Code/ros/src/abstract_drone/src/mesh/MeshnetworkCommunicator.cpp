/**
 * @file MeshnetworkCommunicator.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for MeshnetworkCommunicator
 * @version 1.0
 * @date 2019-04-19
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <thread>  // std::this_thread::sleep_for
#include <chrono>  // std::chrono::seconds

#include "MeshnetworkCommunicator.hpp"
namespace Communication
{
namespace Meshnetwork
{
 MeshnetworkCommunicator::MeshnetworkCommunicator( const uint8_t node,
                                                   const uint8_t drone,
                                                   bool developermode )
     : MeshnetworkComponent( node, drone, developermode )
 {
 }
 // Called after load
 void MeshnetworkCommunicator::Init( )
 {
  std::this_thread::sleep_for( std::chrono::microseconds( initTime ) );
  routerTech->startRouting( );
 }

 void MeshnetworkCommunicator::processIntroduction( const uint8_t *message )
 {
  Messages::IntroduceMessage introduce( message );
  if ( introduce.getKnowGateway( ) &&
       introduce.getHopsUntilGateway( ) < this->hopsFromGatewayAway &&
       introduce.getID( ) != lastGoodKnownLocation.getID( ) ) {
   requestLocation( introduce.getID( ) );
  }
 }

 void MeshnetworkCommunicator::ProcessHeartbeat( const uint8_t *message )
 {
  Messages::HeartbeatMessage msg( message );
  msg.toString( ).c_str( );
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
   sendHeartbeat( msg.getID( ) );
  }
  // if sender knows gateway and we don't
  else if ( msg.getKnowGateway( ) && !connectedToGateway ) {
   routerTech->OtherCanCommunicateWithNode( msg.getID( ),
                                            msg.getPrefferedGateway( ) );
   // Set his gateway as prefferedGateWay and try to contact that
   // gateway
   prefferedGateWay = msg.getPrefferedGateway( );
   sendHeartbeatToGateway( );
  }
 }

 void MeshnetworkCommunicator::CheckConnection( )
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

 void MeshnetworkCommunicator::sendHeartbeatToGateway( )
 {
  if ( !sendHeartbeat( prefferedGateWay ) ) {}
  // this we be flipped back by response of the gateway
  connectedToGateway = false;
 }
 void MeshnetworkCommunicator::lostConnection( )
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

 void MeshnetworkCommunicator::StartEmergencyProtocol( )
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

 void MeshnetworkCommunicator::startMovementNegotiation( )
 {
  static float myDistance;
  // Find out which who the greatest distance from the GATEWAY. He shall be
  // choosen to replace the missing node.
  // Before we go to action we first make a list of everybody near that is
  // disconnected.

  // First find out how far away you are
  if ( routerTech->getDirectionToNode( lastGoodKnownLocation.getID( ) ) ==
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
 void MeshnetworkCommunicator::SafeAddToNegotiationList(
     const std::pair< float, uint8_t > &val )
 {
  mtx.lock( );
  negotiationList.insert( val );
  mtx.unlock( );
 }

 void MeshnetworkCommunicator::informOthersAboutCost( float cost )
 {
  for ( auto &other : routerTech->getSetOfChildren( ) ) {
   uint8_t towards = routerTech->getDirectionToNode( other );
   if ( towards == UINT8_MAX ) { continue; }

   Messages::MovementNegotiationMessage MovNegotiationMsg(
       this->nodeID, this->nodeID, towards, towards, cost );
   uint8_t buffer[Messages::MAX_PAYLOAD];
   MovNegotiationMsg.toPayload( buffer );
   SendMessage( buffer, towards );
  }
 }

 void MeshnetworkCommunicator::processMovementNegotiationMessage(
     const uint8_t *message )
 {
  Messages::MovementNegotiationMessage msg( message );
  SafeAddToNegotiationList( std::make_pair( msg.getCost( ), msg.getID( ) ) );
 }
}  // namespace Meshnetwork
}  // namespace Communication