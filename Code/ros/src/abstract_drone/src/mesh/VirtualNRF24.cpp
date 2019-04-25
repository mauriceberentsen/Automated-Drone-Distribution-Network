#include "abstract_drone/AreaScan.h"
#include "abstract_drone/nodeInfo.h"

#include "VirtualNRF24.hpp"
#include "meshnetwork_component.hpp"

namespace Wireless
{
VirtualNRF24::VirtualNRF24( gazebo::Meshnetwork::MeshnetworkComponent& MC )
    : meshnetworkComponent( MC )
{
}

VirtualNRF24::~VirtualNRF24( )
{
}

void VirtualNRF24::StartAntenna( )
{
 // Initialize ros, if it has not already been initialized.
 if ( !ros::isInitialized( ) ) {
  int argc = 0;
  char** argv = NULL;
  ros::init( argc, argv, "node", ros::init_options::NoSigintHandler );
 }
 this->rosNode.reset( new ros::NodeHandle( "VirtualNRF24" ) );
 Node_TopicName =
     "/Node/" + std::to_string( meshnetworkComponent.getNodeID( ) );
 ROS_INFO( "Start virtual antenna[%s]", Node_TopicName.c_str( ) );

 std::string powerSwitch = "/Nodes/" +
                           std::to_string( meshnetworkComponent.getNodeID( ) ) +
                           "/powerSwitch";

 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::NRF24 >(
         Node_TopicName, 1000, boost::bind( &VirtualNRF24::OnRosMsg, this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );

 this->rosSub = this->rosNode->subscribe( so );
 this->rosPub = this->rosNode->advertise< abstract_drone::nodeInfo >(
     WirelessSignalSimulatorName, 100 );

 // Setup  area scanner
 this->areaScanner = this->rosNode->serviceClient< abstract_drone::AreaScan >(
     "/SignalSimulator/othersInRange" );

 // Setup communication
 this->publishService =
     this->rosNode->serviceClient< abstract_drone::WirelessMessage >(
         "/SignalSimulator/message" );

 // Give the node an ON/OFF button
 this->switchPowerService = this->rosNode->advertiseService(
     powerSwitch, &VirtualNRF24::switchPower, this );

 // Spin up the queue helper thread.
 this->rosQueueThread =
     std::thread( std::bind( &VirtualNRF24::QueueThread, this ) );

 // inform the WirelessSignalSimulator
 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = meshnetworkComponent.getNodeID( );
 nodeinf.sub = Node_TopicName;
 nodeinf.on = true;
 rosPub.publish( nodeinf );
}
void VirtualNRF24::StopAntenna( )
{
}

const bool VirtualNRF24::On( )
{
 return on;
}

bool VirtualNRF24::SendMessageTo( const uint8_t* msg )
{
 abstract_drone::WirelessMessage WM;
 WM.request.message.from = msg[Messages::FROM];
 WM.request.message.to = msg[Messages::TO];
 WM.request.message.forward = msg[Messages::FORWARD];
 WM.request.message.ack = 0;
 for ( int i = 0; i < 32; i++ ) {
  WM.request.message.payload[i] = msg[i];
 }

 if ( publishService.call( WM ) ) {
  if ( WM.response.succes ) {
   return true;
  } else {
   return false;
  }
 } else {
  ROS_ERROR( "SIGNAL SIMULATOR NOT AVAILABLE" );
  return false;
 }
}

void VirtualNRF24::BroadcastMessage( const uint8_t* msg )
{
 abstract_drone::AreaScan scanMsg;
 scanMsg.request.id = meshnetworkComponent.getNodeID( );
 if ( areaScanner.call( scanMsg ) ) {
  if ( scanMsg.response.near.empty( ) ) {
   ROS_WARN( "Node %d found none", meshnetworkComponent.getNodeID( ) );
   return;
  }

  for ( const uint8_t& n : scanMsg.response.near ) {
   uint8_t message[32];

   for ( int i = 0; i < 32; i++ ) {
    message[i] = msg[i];
   }

   message[Messages::TO] = n;
   message[Messages::FORWARD] = n;

   SendMessageTo( message );
  }
 } else {
  ROS_ERROR( "Failed to call service othersInRange" );
 }
}

void VirtualNRF24::DebugingMode( const bool on )
{
 std::string DebugInfoName =
     "/Nodes/" + std::to_string( meshnetworkComponent.getNodeID( ) ) + "/debug";

 this->nodeDebugTopic =
     this->rosNode->advertise< abstract_drone::NodeDebugInfo >( DebugInfoName,
                                                                100 );
 this->NodeDebugInfoThread =
     std::thread( std::bind( &VirtualNRF24::publishDebugInfo, this ) );
}

void VirtualNRF24::OnRosMsg( const abstract_drone::NRF24ConstPtr& _msg )
{
 meshnetworkComponent.OnMsg( _msg->payload.data( ) );
}

bool VirtualNRF24::switchPower( std_srvs::TriggerRequest& request,
                                std_srvs::TriggerResponse& response )
{
 this->on = !this->on;
 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = this->meshnetworkComponent.nodeID;
 nodeinf.on = this->on;
 rosPub.publish( nodeinf );
 return true;
}

void VirtualNRF24::QueueThread( )
{
 static const double timeout = 0.01;
 while ( this->rosNode->ok( ) ) {
  this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
 }
}

void VirtualNRF24::publishDebugInfo( )
{
 abstract_drone::NodeDebugInfo msg;
 while ( this->rosNode->ok( ) ) {
  gazebo::common::Time::Sleep( 1 );  // 1hz refresh is enough
  msg.nodeID = meshnetworkComponent.nodeID;
  msg.ConnectedWithGateway = meshnetworkComponent.connectedToGateway;
  msg.familySize = meshnetworkComponent.routerTech->getTableSize( );
  msg.totalMessages = meshnetworkComponent.totalMessageSent;
  msg.connectedNodes = meshnetworkComponent.routerTech->getAmountOfChildren( );
  msg.prefferedGateWay = meshnetworkComponent.prefferedGateWay;
  msg.on = this->on;
  msg.hops = meshnetworkComponent.hopsFromGatewayAway;
  msg.prefLoc = meshnetworkComponent.lastGoodKnownLocation.getID( );
  nodeDebugTopic.publish( msg );
 }
}

}  // namespace Wireless
