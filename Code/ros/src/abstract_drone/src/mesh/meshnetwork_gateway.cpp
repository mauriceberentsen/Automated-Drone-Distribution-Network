#include "meshnetwork_gateway.hpp"

namespace gazebo
{
MeshnetworkGateway::MeshnetworkGateway( )
{
 shortestPathToGatewayID = 0;
 HopsUntilGateway = 0;
}
void MeshnetworkGateway::OnUpdate( )
{
 // Will probally stay empty
}

void MeshnetworkGateway::processIntroduction(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 IntroduceMessage introduce( _msg->payload.data( ) );
 ROS_INFO( "%s recieved IntroduceMessage %s", this->model->GetName( ).c_str( ),
           introduce.toString( ).c_str( ) );
 auto from = connectedNodes.find( introduce.getID( ) );
 if ( from != connectedNodes.end( ) ) {  // A known node update the hops
  ROS_INFO( "NODE %d UPDATE", introduce.getID( ) );
  connectedNodes.insert( std::pair< uint8_t, uint8_t >(
      introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
 } else  // A new node lets add him and send back a response
 {
  ROS_INFO( "NEW NODE %d ADDED", introduce.getID( ) );
  connectedNodes.insert( std::pair< uint8_t, uint8_t >(
      introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
  IntroduceNode( introduce.getID( ) );
 }
}

void MeshnetworkGateway::processMessage(
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
  case PRESENT:
   ROS_INFO( "%s Recieved PRESENT", this->model->GetName( ).c_str( ) );
   processIntroduction( _msg );
   break;
  case SIGNON:
   ROS_WARN( "%s SIGNON message recieved", this->model->GetName( ).c_str( ) );
   _msg->from == 255 ? handOutNewID( _msg ) : registerNode( _msg );
   break;
  case HEARTBEAT:
   ROS_WARN( "HEARTBEAT message recieved" );
   processHeartbeat( _msg );
   break;
  case GIVEID:
   ROS_WARN( "GIVEID message recieved" );
   break;
  default:
   ROS_WARN( "UNKNOWN message recieved" );
   break;
 }
}
void MeshnetworkGateway::processHeartbeat(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Message msg( _msg->payload.data( ) );
 ROS_WARN( "%s heartbeat message recieved: %s", msg.toString( ).c_str( ),
           this->model->GetName( ).c_str( ) );
}

void MeshnetworkGateway::registerNode(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 ROS_WARN( "NODE REGISTERED" );
}

void MeshnetworkGateway::handOutNewID(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 ROS_WARN( "ASSIGNING NEW ID: not working" );
}

}  // namespace gazebo