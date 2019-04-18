#include "meshnetwork_gateway.hpp"

namespace gazebo
{
MeshnetworkGateway::MeshnetworkGateway( )
{
}

void MeshnetworkGateway::OnUpdate( )
{
 if ( !init ) {
  connectedToGateway = true;
  isGateway = true;
  prefferedGateWay = this->nodeID;

  ros::SubscribeOptions so = ros::SubscribeOptions::create<
      abstract_drone::RequestGatewayDroneFlight >(
      "/gateway", 1000,
      boost::bind( &MeshnetworkGateway::gatewayQueue, this, _1 ),
      ros::VoidPtr( ), &this->rosQueue );
  this->gatewaySub = this->rosNode->subscribe( so );

  init = true;
  searchOtherNodesInRange( );
 }
}

void MeshnetworkGateway::gatewayQueue(
    const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg )
{
 sendGoalToDrone( _msg->ID, _msg->longitude, _msg->latitude, _msg->height );
}

void MeshnetworkGateway::processIntroduction(
    const abstract_drone::NRF24ConstPtr &_msg )
{
}

void MeshnetworkGateway::CheckConnection( )
{
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 10 );  // check every 10 seconds
  // if ( !this->on ) continue;
  for ( auto &node : nodeTable.getSetOfChildren( ) )
   sendHeartbeat( node );
 }
}

void MeshnetworkGateway::processMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 // ROS_WARN( "ProcessMessage with type %d", _msg->payload[1] );
 const uint8_t msgType = _msg->payload[1];
 switch ( msgType ) {
  case Messages::NOTDEFINED:
   ROS_WARN( "Not defined message recieved" );
   ROS_WARN( "Payload %s", _msg->payload );
   break;
  case Messages::LOCATION:
   ROS_WARN( "Location message recieved" );
   break;
  case Messages::PRESENT:
   break;
  case Messages::REQUESTLOCATION:
   processRequestLocation( _msg );
   break;
  case Messages::MISSING:
   ROS_WARN( "%s MISSING message recieved", this->model->GetName( ).c_str( ) );
   processMissing( _msg );
   break;
  case Messages::HEARTBEAT:
   // ROS_WARN( "HEARTBEAT message recieved" );
   ProcessHeartbeat( _msg );
   break;
  case Messages::MOVE_TO_LOCATION:
   ROS_WARN( "MOVE_TO_LOCATION message recieved" );
   sendGoalToEngine( _msg );
   break;
  default:
   ROS_WARN( "UNKNOWN message recieved" );
   break;
 }
}

void MeshnetworkGateway::ProcessHeartbeat(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Messages::HeartbeatMessage msg( _msg->payload.data( ) );
 // ROS_INFO("RECIEVED A HEARTBEAT FROM %u", msg.getID( ));
 sendHeartbeat( msg.getID( ) );
}

}  // namespace gazebo