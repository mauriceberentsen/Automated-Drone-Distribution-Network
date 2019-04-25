/**
 * @file meshnetwork_communicator.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for MeshnetworkCommunicator
 * @version 1.0
 * @date 2019-04-19
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "meshnetwork_communicator.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 MeshnetworkCommunicator::MeshnetworkCommunicator( )
 {
 }
 // Called after load
 void MeshnetworkCommunicator::Init( )
 {
  common::Time::Sleep( initTime );
  routerTech->startRouting( );
 }

 void MeshnetworkCommunicator::processIntroduction(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::IntroduceMessage introduce( _msg->payload.data( ) );
  if ( introduce.getKnowGateway( ) &&
       introduce.getHopsUntilGateway( ) < this->hopsFromGatewayAway &&
       introduce.getID( ) != lastGoodKnownLocation.getID( ) ) {
   requestLocation( introduce.getID( ) );
  }
 }

 void MeshnetworkCommunicator::ProcessHeartbeat(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::HeartbeatMessage msg( _msg->payload.data( ) );
  ROS_WARN( "[%s] {%s}", this->model->GetName( ).c_str( ),
            msg.toString( ).c_str( ) );
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
   // Set his gateway as prefferedGateWay and try to contact that gateway
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
   common::Time::Sleep( CheckConnectionTime );  // check every 10 seconds
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
   return;
  }
  if ( !timerStarted ) {
   routerTech->cantCommunicateWithNode( prefferedGateWay );
   informAboutMissingChild( this->nodeID, prefferedGateWay );
   this->lastTimeOnline = this->model->GetWorld( )->SimTime( );
   timerStarted = true;
  } else if ( lastTimeOnline <
              this->model->GetWorld( )->SimTime( ) - timeUntilConnectionLost ) {
   ROS_WARN( "%f seconds since no connection", timeUntilConnectionLost );
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

  ROS_INFO( "LOST AS A GROUP LETS PICK SOMEONE TO MOVE" );
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

  for ( auto &i : negotiationList ) {
   std::cout << this->model->GetName( ) + " " << ( int )i.second << "--"
             << i.first << std::endl;
  }
  ROS_INFO( "%s Node %u has the greatest cost",
            this->model->GetName( ).c_str( ),
            negotiationList.rbegin( )->second );
  if ( myDistance >= 0 && negotiationList.rbegin( )->second == this->nodeID ) {
   ROS_INFO( "%s: i'm the one who need to move ",
             this->model->GetName( ).c_str( ) );
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
       this->nodeID, towards, towards, cost );
   uint8_t buffer[32];
   MovNegotiationMsg.toPayload( buffer );
   SendMessage( buffer, towards );
   ++totalMessageSent;
  }
 }

 void MeshnetworkCommunicator::processMovementNegotiationMessage(
     const abstract_drone::NRF24ConstPtr &_msg )
 {
  Messages::MovementNegotiationMessage msg( _msg->payload.data( ) );
  SafeAddToNegotiationList( std::make_pair( msg.getCost( ), msg.getID( ) ) );
 }
}  // namespace Meshnetwork
}  // namespace gazebo