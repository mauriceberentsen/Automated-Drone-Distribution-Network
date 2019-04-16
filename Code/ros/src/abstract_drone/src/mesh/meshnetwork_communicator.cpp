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
   // reassignID( _msg->payload[2] );
   break;
  case PRESENT:
   processIntroduction( _msg );
   break;
  case DECEASED:
   processDeceased( _msg );
   break;
  case HEARTBEAT:
   ProcessHeartbeat( _msg );
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
void MeshnetworkCommunicator::ProcessHeartbeat(
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
  nodeTable.proofOfAvailability( msg.getID( ), msg.getPrefferedGateway( ) );
  // aks him where he lives in case we lose him
  // requestLocation( msg.getID( ) );
  prefferedGateWay = msg.getPrefferedGateway( );
  sendHeartbeatToGateway( );
 }
}

void MeshnetworkCommunicator::CheckConnection( )
{
 while ( this->rosNode->ok( ) ) {
  if ( !this->on ) {
   connectedToGateway = false;
   continue;
  }
  common::Time::Sleep( 10 );  // check every 10 seconds
  for ( auto &node : nodeTable.getFamily( ) ) {
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
  nodeTable.proofOfMissing( prefferedGateWay, prefferedGateWay );
  informAboutMissingChild( this->nodeID, prefferedGateWay );
  this->lastTimeOnline = this->model->GetWorld( )->SimTime( );
  timerStarted = true;
 } else if ( lastTimeOnline < this->model->GetWorld( )->SimTime( ) - 30 ) {
  ROS_WARN( "30 seconds since no connection" );
  StartEmergencyProtocol( );
  timerStarted = false;
 }
}

void MeshnetworkCommunicator::StartEmergencyProtocol( )
{
 if ( connectedToGateway )  // in case we meanwhile found a connectetion
 {
  negotiationList.clear( );
  return;
 }
 if ( nodeTable.empty( ) ) {
  sendGoalToEngine( lastGoodKnownLocation );
  nodeTable.getFamily( ).clear( );
  lastGoodKnownLocation = prefferedGateWayLocation;
 } else {
  // start negotiation about who needs to move
  startMovementNegotiation( );
 }
}

void MeshnetworkCommunicator::startMovementNegotiation( )
{
 static bool waiting = false;
 static float myDistance;
 // Find out which who the greatest distance from the GATEWAY. He shall be
 // choosen to replace the missing node
 // before we go to action we first make a list of everybody disconnected

 ROS_INFO( "LOST AS A GROUP LETS PICK SOMEONE TO MOVE" );
 // First find out how far away you are
 if ( nodeTable.getDirectionToNode( lastGoodKnownLocation.getID( ) ) == 255 ) {
  myDistance = distanceBetweenMeAndLocation( prefferedGateWayLocation );
 } else {
  myDistance = -1;
 }

 // add ourself to the list
 // tell others how far away we are
 informOthersAboutDistance( myDistance );
 if ( negotiationList.size( ) < nodeTable.getFamily( ).size( ) ) {
  // wait another round
  return;
 }
 if ( !waiting ) {
  negotiationList.insert( std::make_pair( myDistance, this->nodeID ) );
  waiting = true;
 }
 waiting = false;
 // now it is time for action

 for ( auto &i : negotiationList ) {
  std::cout << this->model->GetName( ) + " " << ( int )i.second << "--"
            << i.first << std::endl;
 }
 ROS_INFO( "%s Node %u has the greatest cost", this->model->GetName( ).c_str( ),
           negotiationList.rbegin( )->second );
 if ( myDistance >= 0 && negotiationList.rbegin( )->second == this->nodeID ) {
  ROS_INFO( "%s: i'm the one who need to move ",
            this->model->GetName( ).c_str( ) );
  sendGoalToEngine( lastGoodKnownLocation );
  nodeTable.getFamily( ).clear( );

  lastGoodKnownLocation = prefferedGateWayLocation;
  negotiationList.clear( );
 } else {
  negotiationList.clear( );
 }
}

void MeshnetworkCommunicator::informOthersAboutDistance( float distance )
{
 MovementNegotiationMessage NegoMSG( this->nodeID, distance );
 abstract_drone::WirelessMessage WM;
 for ( auto &other : nodeTable.getFamily( ) ) {
  uint8_t towards = nodeTable.getDirectionToNode( other.first );
  if ( towards == 255 ) continue;
  WM.request.from = this->nodeID;
  WM.request.to = towards;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = towards;
  WM.request.message.forward = other.first;
  WM.request.message.ack = 0;
  NegoMSG.toPayload( WM.request.message.payload.data( ) );
  ++totalMessageSent;
  sendMessage( WM );
 }
}

void MeshnetworkCommunicator::processMovementNegotiationMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 MovementNegotiationMessage msg( _msg->payload.data( ) );
 negotiationList.insert( std::make_pair( msg.getDistance( ), msg.getID( ) ) );
}

}  // namespace gazebo