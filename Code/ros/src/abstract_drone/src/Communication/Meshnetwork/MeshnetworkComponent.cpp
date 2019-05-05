/**
 * @file MeshnetworkComponent.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file of abstract class MeshnetworkComponent
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <math.h>
#include <thread>
#include <iostream>

#include "MeshnetworkComponent.hpp"

namespace Communication
{
namespace Meshnetwork
{
 MeshnetworkComponent::MeshnetworkComponent( const uint8_t node,
                                             const uint8_t drone,
                                             bool developermode )
     : nodeID( node )
     , droneID( drone )
     , debug( developermode )
     , routerTech( new RoutingTechnique::ChildTableTree( *this ) )
     , communication(
           new ros::WirelessSimulation::VirtualNRF24( *this, *this ) )
     , droneEngine( new ros::Drone::RosDroneEngineConnector( drone ) )
 {
  this->communication->StartAntenna( );
  if ( debug ) { this->communication->DebugingMode( true ); }
  this->checkConnectionThread =
      std::thread( std::bind( &MeshnetworkComponent::CheckConnection, this ) );
 }

 void MeshnetworkComponent::OnMsg( const uint8_t *message )
 {
  ( message[Messages::FORWARD] == this->nodeID ) ? processMessage( message )
                                                 : forwardMessage( message );
  routerTech->OtherCanCommunicateWithNode( message[Messages::FROM],
                                           message[Messages::CREATOR] );
 }

 const uint8_t MeshnetworkComponent::getNodeID( ) const
 {
  return nodeID;
 }

 const bool MeshnetworkComponent::getConnectedToGateway( ) const
 {
  return connectedToGateway;
 }
 const uint32_t MeshnetworkComponent::getTotalMessageSent( ) const
 {
  return totalMessageSent;
 }
 const uint8_t MeshnetworkComponent::getPrefferedGateway( ) const
 {
  return prefferedGateWay;
 }

 const uint8_t MeshnetworkComponent::getHopsFromGatewayAway( ) const
 {
  return hopsFromGatewayAway;
 }

 const uint8_t MeshnetworkComponent::getLastGoodKnownLocationID( ) const
 {
  return lastGoodKnownLocation.getCreator( );
 }

 const uint8_t MeshnetworkComponent::getRouterTechTableSize( ) const
 {
  return routerTech->getTableSize( );
 }

 void MeshnetworkComponent::forwardMessage( const uint8_t *message )
 {
  const uint8_t other =
      routerTech->getDirectionToNode( message[Messages::FORWARD] );
  if ( other == UINT8_MAX ) { return; }

  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  for ( int i = 0; i < Messages::MAX_PAYLOAD; i++ ) {
   buffer[i] = message[i];
  }
  buffer[Messages::FROM] = this->nodeID;
  buffer[Messages::TO] = other;
  if ( buffer[Messages::TYPE] == Messages::HEARTBEAT ) {
   Messages::HeartbeatMessage msg( buffer );
   msg.makeHop( );
   if ( msg.getHops( ) > UINT8_MAX - 1 ) return;
   msg.toPayload( buffer );
  }
  SendMessage( buffer, other );
 }

 void MeshnetworkComponent::processMessage( const uint8_t *message )
 {
  const uint8_t msgType = message[Messages::TYPE];
  switch ( msgType ) {
   case Messages::LOCATION:
    processLocation( message );
    break;
   case Messages::MISSING:
    ( message );
    break;
   case Messages::MOVE_TO_LOCATION:
    processSendGoalToEngine( message );
    break;
   case Messages::PRESENT:
    processIntroduction( message );
    break;
   case Messages::HEARTBEAT:
    ProcessHeartbeat( message );
    break;
   case Messages::REQUESTLOCATION:
    processRequestLocation( message );
    break;
   case Messages::MOVEMENT_NEGOTIATION:
    processMovementNegotiationMessage( message );
    break;
   default:
    break;
  }
 }
 /*public*/ void MeshnetworkComponent::processLocation( const uint8_t *message )
 {
  Messages::LocationMessage msg( message );
  if ( msg.getCreator( ) == prefferedGateWay ) {
   prefferedGateWayLocation = msg;
   knowPrefferedGatewayLocation = true;
  } else {
   lastGoodKnownLocation = msg;
  }
 }

 void MeshnetworkComponent::requestLocation( const uint8_t other )
 {
  uint8_t towards = routerTech->getDirectionToNode( other );
  Messages::Message msg( this->nodeID, this->nodeID, Messages::REQUESTLOCATION,
                         towards, other );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  msg.toPayload( buffer );
  SendMessage( buffer, other );
 }

 void MeshnetworkComponent::processMissing( const uint8_t *message )
 {
  Messages::MissingMessage msg( message );
  if ( routerTech->OtherCantCommunicateWithNode( msg.getCreator( ),
                                                 msg.getMissing( ) ) > 0 )
   informAboutMissingChild( msg.getCreator( ), msg.getMissing( ) );
 }

 void MeshnetworkComponent::processSendGoalToEngine( const uint8_t *message )
 {
  Messages::LocationMessage locmsg( message );

  droneEngine->setGoal( locmsg.getLongitude( ), locmsg.getLatitude( ),
                        locmsg.getHeight( ) );
 }

 void MeshnetworkComponent::sendGoalToEngine(
     const Messages::LocationMessage &_msg )
 {
  droneEngine->setGoal( _msg.getLongitude( ), _msg.getLatitude( ),
                        _msg.getHeight( ) );
 }

 /*public*/ bool MeshnetworkComponent::sendHeartbeat( uint8_t other )
 {
  uint8_t towards = routerTech->getDirectionToNode( other );
  if ( towards == UINT8_MAX ) return false;
  // create a Message to introduce yourself to others
  Messages::HeartbeatMessage heartbeat( this->nodeID, this->nodeID, towards,
                                        other, connectedToGateway,
                                        prefferedGateWay );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  heartbeat.toPayload( buffer );
  return SendMessage( buffer, towards );
 }

 void MeshnetworkComponent::SendGoalToDrone( const uint8_t ID,
                                             const float latitude,
                                             const float longitude,
                                             const uint16_t height )
 {
  uint8_t towards = routerTech->getDirectionToNode( ID );
  Messages::GoToLocationMessage goToLocationMsg(
      this->nodeID, this->nodeID, towards, ID, latitude, longitude, height );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  goToLocationMsg.toPayload( buffer );
  SendMessage( buffer, towards );
 }

 void MeshnetworkComponent::informAboutMissingChild( uint8_t parent,
                                                     uint8_t missing )
 {
  for ( auto &chi1d : routerTech->getSetOfChildren( ) ) {
   if ( parent == chi1d ) continue;
   uint8_t towards = routerTech->getDirectionToNode( chi1d );
   if ( towards == UINT8_MAX ) continue;
   Messages::MissingMessage missingMessage( this->nodeID, this->nodeID, towards,
                                            chi1d, missing );
   uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
   missingMessage.toPayload( buffer );
   SendMessage( buffer, towards );
  }
 }

 /*public*/ void MeshnetworkComponent::searchOtherNodesInRange( )
 {
  Messages::IntroduceMessage introduce( this->nodeID, this->nodeID, 0, 0,
                                        this->hopsFromGatewayAway,
                                        this->connectedToGateway );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  introduce.toPayload( buffer );

  communication->BroadcastMessage( buffer );
 }

 void MeshnetworkComponent::processRequestLocation( const uint8_t *payload )
 {
  Messages::Message msg( payload );
  sendLocation( msg.getCreator( ) );
 }

 void MeshnetworkComponent::sendLocation( const uint8_t other )
 {
  const ignition::math::Vector3< float > location = droneEngine->getLocation( );
  uint8_t towards = routerTech->getDirectionToNode( other );
  Messages::LocationMessage msg( this->nodeID, this->nodeID, towards, other,
                                 location.X( ), location.Y( ), location.Z( ),
                                 0 );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  msg.toPayload( buffer );
  SendMessage( buffer, other );
 }

 float MeshnetworkComponent::distanceBetweenMeAndLocation(
     const Messages::LocationMessage &A )
 {
  const ignition::math::Vector3< float > location = droneEngine->getLocation( );

  float a, b, c;
  a = A.getLongitude( ) - location.X( );
  b = A.getLatitude( ) - location.Y( );
  c = A.getHeight( ) - location.Z( );
  // return the square root of (a^2 + b^2 + c^2)
  return std::sqrt( std::pow( a, 2 ) + std::pow( b, 2 ) + std::pow( c, 2 ) );
 }

 /*public*/ void MeshnetworkComponent::IntroduceNode( const uint8_t other )
 {
  // create a Message to introduce yourself to others
  Messages::IntroduceMessage introduce( this->nodeID, this->nodeID, other,
                                        other, this->hopsFromGatewayAway,
                                        this->connectedToGateway );
  uint8_t buffer[Messages::MAX_PAYLOAD] = {0};
  introduce.toPayload( buffer );
  SendMessage( buffer, other );
 }

 bool MeshnetworkComponent::SendMessage( const uint8_t *message,
                                         const uint8_t to )
 {
  if ( communication->SendMessageTo( message ) ) {
   routerTech->canCommunicateWithNode( to );
   ++totalMessageSent;
   return true;
  } else if ( routerTech->cantCommunicateWithNode( to ) > 0 )
   informAboutMissingChild( this->nodeID, to );
  return false;
 }
}  // namespace Meshnetwork
}  // namespace Communication