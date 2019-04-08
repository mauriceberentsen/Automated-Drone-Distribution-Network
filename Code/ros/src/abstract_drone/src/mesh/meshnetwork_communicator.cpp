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
 //  IntroduceMessage introduce( _msg->payload.data( ) );
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
 ROS_WARN( "ProcessMessage with type %d", _msg->payload[1] );
 const uint8_t msgType = _msg->payload[1];
 switch ( msgType ) {
  case NOTDEFINED:
   ROS_WARN( "Not defined message recieved" );
   ROS_WARN( "Payload %s", _msg->payload );
   break;
  case LOCATION:
   ROS_WARN( "Location message recieved" );
   break;
  case SIGNON:
   ROS_WARN( "%s SIGNON message recieved", this->model->GetName( ).c_str( ) );
   break;
  case GIVEID:
   ROS_WARN( "GIVEID message recieved" );
   reassignID( _msg->payload[2] );
   break;
  case PRESENT:
   ROS_INFO( "%s Recieved PRESENT", this->model->GetName( ).c_str( ) );
   processIntroduction( _msg );
   break;
  case DECEASED:
   ROS_WARN( "%s DECEASED message recieved", this->model->GetName( ).c_str( ) );
   processDeceased( _msg );
   break;
  case HEARTBEAT:
   ROS_WARN( "%s HEARTBEAT message recieved",
             this->model->GetName( ).c_str( ) );
   processHeartbeat( _msg );
   break;
  case MOVE_TO_LOCATION:
   ROS_WARN( "MOVE_TO_LOCATION message recieved" );
   sendGoalToEngine( _msg );
   break;
  default:
   ROS_WARN( "UNKNOWN message recieved" );
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
  return;
 }
 // if sender doesn't know gateway but we do tell him
 if ( !msg.getKnowGateway( ) && connectedToGateway ) {
  sendHeartbeat( msg.getID( ) );
 }
 // if sender knows gateway and we don't try pinging gateway again
 else if ( msg.getKnowGateway( ) && !connectedToGateway ) {
  NodeTable.proofOfLive( msg.getID( ), msg.getPrefferedGateway( ) );
  prefferedGateWay = msg.getPrefferedGateway( );
  sendHeartbeatToGateway( );
 }
}

void MeshnetworkCommunicator::CheckConnection( )
{
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 10 );  // check every 10 seconds
  if(!this->on) 
  {
      connectedToGateway = false;
      continue;
  }
  for ( auto &node : NodeTable.getFamily( ) ) {
   sendHeartbeat( node.first );
  }
  searchOtherNodesInRange( );  // maybe there is someone close
  if ( connectedToGateway )
   sendHeartbeatToGateway( );
  else
   lostConnection( );
 }
}

void MeshnetworkCommunicator::sendHeartbeatToGateway( )
{
 if(!sendHeartbeat( prefferedGateWay ))
 {
    ROS_INFO("NO WAY FOUND TOWARDS GATEWAY");
 }
 connectedToGateway = false;
}
void MeshnetworkCommunicator::lostConnection( )
{
    NodeTable.proofOfDeceased( prefferedGateWay, prefferedGateWay );
    informAboutDeceasedChild(this->NodeID, prefferedGateWay);
}

}  // namespace gazebo