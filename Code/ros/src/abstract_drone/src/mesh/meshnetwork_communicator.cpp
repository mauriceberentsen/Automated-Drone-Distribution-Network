#include "meshnetwork_communicator.hpp"

namespace gazebo
{
// Called by the world update start event
void MeshnetworkCommunicator::OnUpdate( )
{
 if ( !this->init )  // we want this to happen once after all nodes are loaded
 {
  std::random_device rd;
  std::mt19937 mt( rd( ) );
  std::uniform_real_distribution< double > dist( 10.0, 20.0 );

  // common::Time::Sleep( dist( mt ) );
  searchOtherNodesInRange( );
  // simulate startup time of a nrf node
  // TODO research how long a nrf takes to start;
  // search a connection route;
  // scan for nodes Nodes near
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
  ROS_INFO( "SOMEONE CLOSER WHERE IS HE FROM?" );
  requestLocation( introduce.getID( ) );
 }
 //  ROS_INFO( "%s recieved IntroduceMessage %s", this->model->GetName( ).c_str(
 //  ),
 //            introduce.toString( ).c_str( ) );
 //  auto from = connectedNodes.find( introduce.getID( ) );
 //  if ( introduce.getHopsUntilsGateway( ) <
 //       HopsUntilGateway )  // minus one since we are only interested in
 //       shorter
 //                           // paths not alternatives
 //  {
 //   HopsUntilGateway = introduce.getHopsUntilsGateway( ) + 1;
 //  }
 //  if ( from != connectedNodes.end( ) )  // A known node update the hops
 //  {
 //   connectedNodes.insert( std::pair< uint8_t, uint8_t >(
 //       introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
 //  } else  // A new node lets add him and send back a response
 //  {
 //   connectedNodes.insert( std::pair< uint8_t, uint8_t >(
 //       introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
 //   IntroduceNode( introduce.getID( ) );
 //  }
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
  hopsFromGatewayAway = msg.hops;
  return;
 }
 // if sender doesn't know gateway but we do tell him
 if ( !msg.getKnowGateway( ) && connectedToGateway ) {
  sendHeartbeat( msg.getID( ) );
 }
 // if sender knows gateway and we don't try pinging gateway again
 else if ( msg.getKnowGateway( ) && !connectedToGateway ) {
  NodeTable.proofOfLive( msg.getID( ), msg.getPrefferedGateway( ) );
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
 if ( !sendHeartbeat( prefferedGateWay ) ) {
  // ROS_INFO( "NO WAY FOUND TOWARDS GATEWAY" );
 }
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
  NodeTable.proofOfDeceased( prefferedGateWay, prefferedGateWay );
  informAboutDeceasedChild( this->NodeID, prefferedGateWay );
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
 if ( NodeTable.empty( ) )
  sendGoalToEngine( lastGoodKnownLocation );
 else {
  // start negotiation about who needs to move

  startMovementNegotiation( );
 }
}

void MeshnetworkCommunicator::startMovementNegotiation( )
{
 // Find out which who the greatest distance from the GATEWAY. He shall be
 // choosen to replace the missing node
 // before we go to action we first make a list of everybody disconnected
 while ( negotiationList.size( ) <= NodeTable.getFamily( ).size( ) ) {
  ROS_INFO( "LOST AS A GROUP LETS BUILD PICK SOMEONE TO MOVE" );
  // First find out how far away you are
  float myDistance = distanceBetweenMeAndLocation( prefferedGateWayLocation );
  // add ourself to the list
  negotiationList.insert( std::make_pair( myDistance, this->NodeID ) );
  // tell others how far away we are
  informOthersAboutDistance( myDistance );
  // next time here we go to action
 }
 // now it is time for action

 for ( auto &i : negotiationList ) {
  std::cout << this->model->GetName( ) + " " << ( int )i.second << "--"
            << i.first << std::endl;
 }
 ROS_INFO( "%s Node %u has the greatest distance",
           this->model->GetName( ).c_str( ),
           negotiationList.rbegin( )->second );
 if ( negotiationList.rbegin( )->second == this->NodeID ) {
  ROS_INFO( "%s lets move this boi", this->model->GetName( ).c_str( ) );
  sendGoalToEngine( lastGoodKnownLocation );

  negotiationList.clear( );
 }
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
    NodeTable.proofOfDeceased( towards, towards );
   } else {
    NodeTable.proofOfLive( towards, towards );
   }
  } else {
   NodeTable.proofOfDeceased( towards, towards );
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