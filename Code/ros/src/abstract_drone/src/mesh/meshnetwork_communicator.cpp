#include "meshnetwork_communicator.hpp"

namespace gazebo
{
// Called by the world update start event
void MeshnetworkCommunicator::OnUpdate( )
{
 if ( !this->init )  // we want this to happen once after all nodes are loaded
 {
  searchOtherNodesInRange( );
  init = true;
 }
}

void MeshnetworkCommunicator::processIntroduction(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 IntroduceMessage introduce( _msg->payload.data( ) );
 if ( introduce.getKnowGateway( ) &&
      introduce.getHopsUntilsGateway( ) < this->hopsFromGatewayAway &&
      introduce.getID( ) != lastGoodKnownLocation.getID( ) ) {
  requestLocation( introduce.getID( ) );
 }
}

void MeshnetworkCommunicator::processMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 const uint8_t msgType = _msg->payload[1];
 switch ( msgType ) {
  case LOCATION:
   processLocation( _msg );
   break;
  case REQUESTLOCATION:
   processRequestLocation( _msg );
   break;
  case SIGNON:
   ROS_WARN( "%s SIGNON message recieved", this->model->GetName( ).c_str( ) );
   break;
  case GIVEID:
   ROS_WARN( "GIVEID message recieved" );
   reassignID( _msg->payload[2] );
   break;
  case PRESENT:
   processIntroduction( _msg );
   break;
  case DECEASED:
   processDeceased( _msg );
   break;
  case HEARTBEAT:
   processHeartbeat( _msg );
   break;
  case MOVE_TO_LOCATION:
   ROS_WARN( "MOVE_TO_LOCATION message recieved" );
   sendGoalToEngine( _msg );
   break;
  case MOVEMENT_NEGOTIATION:
   ROS_WARN( "%s MOVEMENT_NEGOTIATION message recieved %u",
             this->model->GetName( ).c_str( ) );
   processMovementNegotiationMessage( _msg );
   break;
  default:
   ROS_WARN( "%s UNKOWN message recieved %u", this->model->GetName( ).c_str( ),
             msgType );
   break;
 }
}
void MeshnetworkCommunicator::processHeartbeat(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 HeartbeatMessage msg( _msg->payload.data( ) );
 // if sender is gateway flip bool and return
 if ( msg.getIsGateway( ) ) {
  prefferedGateWay = msg.getPrefferedGateway( );
  connectedToGateway = true;
  hopsFromGatewayAway = msg.hops + 1;  // count our own hop towards gateway
  return;
 }
 // if sender doesn't know gateway but we do tell him
 if ( !msg.getKnowGateway( ) && connectedToGateway ) {
  sendHeartbeat( msg.getID( ) );
 }
 // if sender knows gateway and we don't try pinging gateway again
 else if ( msg.getKnowGateway( ) && !connectedToGateway ) {
  NodeTable.proofOfAvailability( msg.getID( ), msg.getPrefferedGateway( ) );
  // aks him where he lives in case we lose him
  // requestLocation( msg.getID( ) );
  prefferedGateWay = msg.getPrefferedGateway( );
  sendHeartbeatToGateway( );
 }
}

void MeshnetworkCommunicator::CheckConnection( )
{
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 10 );  // check every 10 seconds
  if ( !this->on ) {
   connectedToGateway = false;
   continue;
  }
  for ( auto &node : NodeTable.getFamily( ) ) {
   sendHeartbeat( node.first );
  }
  searchOtherNodesInRange( );  // maybe there is someone close
  if ( connectedToGateway ) {
   if ( !knowPrefferedGatewayLocation ) { requestLocation( prefferedGateWay ); }
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
  NodeTable.proofOfMissing( prefferedGateWay, prefferedGateWay );
  informAboutMissingChild( this->NodeID, prefferedGateWay );
  this->lastTimeOnline = this->model->GetWorld( )->SimTime( );
  timerStarted = true;
 } else if ( lastTimeOnline < this->model->GetWorld( )->SimTime( ) - 30 ) {
  ROS_WARN( "30 seconds since no connection" );
  startEmergencyProtocol( );
  timerStarted = false;
 }
}

void MeshnetworkCommunicator::startEmergencyProtocol( )
{
 if ( connectedToGateway )  // in case we meanwhile found a connectetion
 {
  negotiationList.clear( );
  return;
 }
 if ( NodeTable.empty( ) ) {
  sendGoalToEngine( lastGoodKnownLocation );
  NodeTable.getFamily( ).clear( );
  lastGoodKnownLocation = prefferedGateWayLocation;
 } else {
  // start negotiation about who needs to move
  startMovementNegotiation( );
 }
}

void MeshnetworkCommunicator::startMovementNegotiation( )
{
 // Find out which who the greatest distance from the GATEWAY. He shall be
 // choosen to replace the missing node
 // before we go to action we first make a list of everybody disconnected
 ROS_INFO( "LOST AS A GROUP LETS PICK SOMEONE TO MOVE" );
 // First find out how far away you are
 float myDistance = -1;
 if ( NodeTable.getDirectionToNode( lastGoodKnownLocation.getID( ) ) == 255 ) {
  myDistance = distanceBetweenMeAndLocation( prefferedGateWayLocation );
 }
 // add ourself to the list
 // tell others how far away we are
 informOthersAboutDistance( myDistance );
 if ( negotiationList.size( ) < NodeTable.getFamily( ).size( ) ) {
  // wait another round
  return;
 }
 // now it is time for action

 negotiationList.insert( std::make_pair( myDistance, this->NodeID ) );
 for ( auto &i : negotiationList ) {
  std::cout << this->model->GetName( ) + " " << ( int )i.second << "--"
            << i.first << std::endl;
 }
 ROS_INFO( "%s Node %u has the greatest cost", this->model->GetName( ).c_str( ),
           negotiationList.rbegin( )->second );
 if ( myDistance >= 0 && negotiationList.rbegin( )->second == this->NodeID ) {
  ROS_INFO( "%s: i'm the one who need to move ",
            this->model->GetName( ).c_str( ) );
  sendGoalToEngine( lastGoodKnownLocation );
  NodeTable.getFamily( ).clear( );

  lastGoodKnownLocation = prefferedGateWayLocation;
 }
 negotiationList.clear( );
}

void MeshnetworkCommunicator::informOthersAboutDistance( float distance )
{
 MovementNegotiationMessage NegoMSG( this->NodeID, distance );
 abstract_drone::WirelessMessage WM;
 abstract_drone::NRF24 nrfmsg;
 for ( auto &other : NodeTable.getFamily( ) ) {
  uint8_t towards = NodeTable.getDirectionToNode( other.first );
  if ( towards == 255 ) continue;
  WM.request.from = this->NodeID;
  WM.request.to = towards;
  nrfmsg.from = this->NodeID;
  nrfmsg.to = towards;
  nrfmsg.forward = other.first;
  nrfmsg.ack = 0;
  NegoMSG.toPayload( nrfmsg.payload.data( ) );
  WM.request.message = nrfmsg;
  ++totalMessageSent;
  if ( publishService.call( WM ) ) {
   if ( !WM.response.succes ) {
    NodeTable.proofOfMissing( towards, towards );
   } else {
    NodeTable.proofOfAvailability( towards, towards );
   }
  } else {
   NodeTable.proofOfMissing( towards, towards );
  }
 }
}

void MeshnetworkCommunicator::processMovementNegotiationMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 MovementNegotiationMessage msg( _msg->payload.data( ) );
 negotiationList.insert( std::make_pair( msg.getDistance( ), msg.getID( ) ) );
}

}  // namespace gazebo