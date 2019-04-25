/**
 * @file meshnetwork_component.cpp
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

#include "meshnetwork_component.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 /*public*/ MeshnetworkComponent::MeshnetworkComponent( )
     : routerTech( new RoutingTechnique::ChildTableTree( *this ) )
     , communication( new Wireless::VirtualNRF24( *this ) )
 {
 }

 /*protected*/ void MeshnetworkComponent::Load( physics::ModelPtr _parent,
                                                sdf::ElementPtr _sdf )
 {
  // Store the pointer to the model
  this->model = _parent;
  this->nodeID = UINT8_MAX;    // unkown nodes are number UINT8_MAX
  this->droneID = UINT16_MAX;  // Drones need unique id's to connect the motor

  if ( _sdf->HasElement( "nodeID" ) ) {
   this->nodeID = _sdf->Get< int >( "nodeID" );
  }
  if ( _sdf->HasElement( "DroneID" ) ) {
   this->droneID = _sdf->Get< int >( "DroneID" );
  } else {
   ROS_ERROR(
       "Drone com being used without a droneid, set up using the tag \
    <DroneID> unique <DroneID> \n  Restart ussing droneID's else drone \
    movement will be a mess" );
  }
  if ( _sdf->HasElement( "Debug" ) ) {
   this->debug = _sdf->Get< bool >( "Debug" );
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node

  this->droneEngine.reset( new ros::rosDroneEngineConnector( this->droneID ) );
  this->communication->StartAntenna( );
  this->checkConnectionThread =
      std::thread( std::bind( &MeshnetworkComponent::CheckConnection, this ) );
  if ( debug ) {
   this->communication->DebugingMode( true );
   ROS_INFO( "Started publishing debug info for %s",
             this->model->GetName( ).c_str( ) );
  }
  ROS_INFO( "Loaded MeshnetworkComponent Plugin with parent...%s",
            this->model->GetName( ).c_str( ) );
 }

 void MeshnetworkComponent::OnMsg( const abstract_drone::NRF24ConstPtr &_msg )
 {
  ( _msg->forward == this->nodeID ) ? processMessage( _msg )
                                    : forwardMessage( _msg );
  routerTech->OtherCanCommunicateWithNode( _msg->from, _msg->payload[0] );
  // TODO send ack message back
 }

 uint8_t MeshnetworkComponent::getNodeID( )
 {
  return this->nodeID;
 }

 void MeshnetworkComponent::forwardMessage(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  abstract_drone::WirelessMessage WM;
  WM.request.message = *( _msg );
  if ( _msg->payload[1] == Messages::HEARTBEAT ) {
   Messages::HeartbeatMessage msg( _msg->payload.data( ) );
   msg.makeHop( );
   if ( msg.getHops( ) > UINT8_MAX ) return;

   msg.toPayload( WM.request.message.payload.data( ) );
  }

  uint8_t other = routerTech->getDirectionToNode( _msg->forward );
  if ( other == UINT8_MAX ) return;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = other;
  SendMessage( WM.request.message.payload.data( ), other );
 }

 void MeshnetworkComponent::processMessage(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  const uint8_t msgType = _msg->payload[1];
  ROS_INFO( "NODE[%u] recieved messagetype %u", nodeID, msgType );
  switch ( msgType ) {
   case Messages::LOCATION:
    processLocation( _msg );
    break;
   case Messages::MISSING:
    processMissing( _msg );
    break;
   case Messages::MOVE_TO_LOCATION:
    processSendGoalToEngine( _msg );
    break;
   case Messages::PRESENT:
    processIntroduction( _msg );
    break;
   case Messages::HEARTBEAT:
    ProcessHeartbeat( _msg );
    break;
   case Messages::REQUESTLOCATION:
    // processRequestLocation( _msg );
    break;
   case Messages::MOVEMENT_NEGOTIATION:
    processMovementNegotiationMessage( _msg );
    break;
   default:
    ROS_WARN( "%s UNKOWN message recieved %u", this->model->GetName( ).c_str( ),
              msgType );
    break;
  }
 }
 /*public*/ void MeshnetworkComponent::processLocation(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::LocationMessage msg( _msg->payload.data( ) );
  if ( msg.getID( ) == prefferedGateWay ) {
   prefferedGateWayLocation = msg;
   knowPrefferedGatewayLocation = true;
  } else {
   lastGoodKnownLocation = msg;
  }
 }

 void MeshnetworkComponent::requestLocation( const uint8_t other )
 {
  Messages::Message msg( this->nodeID, Messages::REQUESTLOCATION, other,
                         other );
  uint8_t buffer[32];
  msg.toPayload( buffer );
  SendMessage( buffer, other );
 }

 void MeshnetworkComponent::processMissing(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::MissingMessage msg( _msg->payload.data( ) );
  if ( routerTech->OtherCantCommunicateWithNode( msg.getID( ),
                                                 msg.getMissing( ) ) > 0 )
   informAboutMissingChild( msg.getID( ), msg.getMissing( ) );
 }

 void MeshnetworkComponent::processSendGoalToEngine(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::LocationMessage locmsg( _msg->payload.data( ) );

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
  Messages::HeartbeatMessage heartbeat( this->nodeID, towards, other,
                                        connectedToGateway, prefferedGateWay );
  uint8_t buffer[32];
  heartbeat.toPayload( buffer );
  return SendMessage( buffer, towards );
 }

 void MeshnetworkComponent::sendGoalToDrone( const uint8_t ID,
                                             const float longitude,
                                             const float latitude,
                                             const uint16_t height )
 {
  uint8_t other = routerTech->getDirectionToNode( ID );
  Messages::GoToLocationMessage GTLmsg( this->nodeID, other, ID, longitude,
                                        latitude, height );
  uint8_t buffer[32];
  GTLmsg.toPayload( buffer );
  SendMessage( buffer, other );
 }

 void MeshnetworkComponent::informAboutMissingChild( uint8_t parent,
                                                     uint8_t missing )
 {
  for ( auto &chi1d : routerTech->getSetOfChildren( ) ) {
   if ( parent == chi1d ) continue;
   uint8_t other = routerTech->getDirectionToNode( chi1d );
   if ( other == UINT8_MAX ) continue;

   Messages::MissingMessage missingMSG( this->nodeID, other, chi1d, missing );
   uint8_t buffer[32];
   missingMSG.toPayload( buffer );
   SendMessage( buffer, other );
  }
 }

 /*public*/ void MeshnetworkComponent::searchOtherNodesInRange( )
 {
  Messages::IntroduceMessage introduce(
      this->nodeID, 0, 0, this->hopsFromGatewayAway, this->connectedToGateway );
  uint8_t buffer[32];
  introduce.toPayload( buffer );

  communication->BroadcastMessage( buffer );
 }

 void MeshnetworkComponent::processRequestLocation( const uint8_t *payload )
 {
  Messages::Message msg( payload );
  sendLocation( msg.getID( ) );
 }

 void MeshnetworkComponent::sendLocation( const uint8_t other )
 {
  const ignition::math::Vector3< float > location = droneEngine->getLocation( );

  Messages::LocationMessage msg( this->nodeID, other, other, location.X( ),
                                 location.Y( ), location.Z( ), 0 );
  uint8_t buffer[32];
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
  return std::sqrt( std::pow( a, 2 ) + std::pow( b, 2 ) + std::pow( c, 2 ) );
 }

 /*public*/ void MeshnetworkComponent::IntroduceNode( const uint8_t other )
 {
  // create a Message to introduce yourself to others
  Messages::IntroduceMessage introduce( this->nodeID, other, other,
                                        this->hopsFromGatewayAway,
                                        this->connectedToGateway );
  uint8_t buffer[32];
  introduce.toPayload( buffer );
  SendMessage( buffer, other );
 }

 bool MeshnetworkComponent::SendMessage( uint8_t *message, uint8_t to )
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

}  // namespace gazebo
