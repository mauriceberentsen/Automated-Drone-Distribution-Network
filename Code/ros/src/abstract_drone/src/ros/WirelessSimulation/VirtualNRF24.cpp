/**
 * @file VirtualNRF24.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for the Virtual NRF24
 * @version 1.0
 * @date 2019-04-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "abstract_drone/AreaScan.h"
#include "abstract_drone/DroneInfo.h"

#include "VirtualNRF24.hpp"
#include "../../Communication/Meshnetwork/MeshnetworkComponent.hpp"

using namespace Communication::Messages;

namespace ros
{
namespace WirelessSimulation
{
 VirtualNRF24::VirtualNRF24( )
     : meshnetworkComponent( nullptr ), debuginfo( nullptr )
 {
 }

 VirtualNRF24::~VirtualNRF24( )
 {
 }

 void VirtualNRF24::StartAntenna( Communication::Wireless::IMeshNetwork* MC )
 {
  meshnetworkComponent = MC;
  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char** argv = NULL;
   ros::init( argc, argv, "node", ros::init_options::NoSigintHandler );
  }
  this->rosNode.reset( new ros::NodeHandle( "VirtualNRF24" ) );
  Node_TopicName =
      "/Node/" + std::to_string( meshnetworkComponent->getNodeID( ) );
  ROS_INFO( "Start virtual antenna[%s]", Node_TopicName.c_str( ) );

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create< abstract_drone::NRF24 >(
          Node_TopicName, 1000,
          boost::bind( &VirtualNRF24::OnRosMsg, this, _1 ), ros::VoidPtr( ),
          &this->rosQueue );

  this->rosSub = this->rosNode->subscribe( so );

  this->rosPub = this->rosNode->advertise< abstract_drone::DroneInfo >(
      WirelessSignalSimulatorName, 100 );

  // inform the WirelessSignalSimulator
  abstract_drone::DroneInfo nodeinf;
  nodeinf.nodeID = meshnetworkComponent->getNodeID( );
  nodeinf.sub = Node_TopicName;
  nodeinf.on = true;
  while ( rosPub.getNumSubscribers( ) == 0 ) {
   std::this_thread::sleep_for( std::chrono::microseconds( 1 ) );
  }
  rosPub.publish( nodeinf );

  // Setup  area scanner
  this->areaScanner = this->rosNode->serviceClient< abstract_drone::AreaScan >(
      "/SignalSimulator/othersInRange" );

  // Setup communication
  this->publishService =
      this->rosNode->serviceClient< abstract_drone::WirelessMessage >(
          "/SignalSimulator/message" );

  std::string powerSwitch =
      "/Nodes/" + std::to_string( meshnetworkComponent->getNodeID( ) ) +
      "/powerSwitch";
  // Give the node an ON/OFF button
  this->switchPowerService = this->rosNode->advertiseService(
      powerSwitch, &VirtualNRF24::switchPower, this );

  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread( std::bind( &VirtualNRF24::QueueThread, this ) );
 }

 void VirtualNRF24::StopAntenna( )
 {
  rosNode->shutdown( );
 }

 const bool VirtualNRF24::On( )
 {
  return on;
 }

 bool VirtualNRF24::SendMessageTo( const uint8_t* msg )
 {
  abstract_drone::WirelessMessage WM;
  WM.request.from = msg[FROM];
  WM.request.to = msg[TO];
  for ( int i = 0; i < MAX_PAYLOAD; i++ ) {
   WM.request.message.payload[i] = msg[i];
  }

  if ( publishService.call( WM ) ) {
   if ( WM.response.succes ) {
    return true;
   } else {
    return false;
   }
  } else {
   //    ROS_ERROR( "SIGNAL SIMULATOR NOT AVAILABLE" );
   return false;
  }
 }

 void VirtualNRF24::BroadcastMessage( const uint8_t* msg )
 {
  abstract_drone::AreaScan scanMsg;
  scanMsg.request.id = meshnetworkComponent->getNodeID( );
  if ( areaScanner.call( scanMsg ) ) {
   if ( scanMsg.response.near.empty( ) ) {
    ROS_WARN( "Node %d found none", meshnetworkComponent->getNodeID( ) );
    return;
   }

   for ( const uint8_t& n : scanMsg.response.near ) {
    uint8_t message[MAX_PAYLOAD];

    for ( int i = 0; i < MAX_PAYLOAD; i++ ) {
     message[i] = msg[i];
    }

    message[TO] = n;

    SendMessageTo( message );
   }
  } else {
   //    ROS_ERROR( "Failed to call service othersInRange" );
  }
 }

 void VirtualNRF24::DebugingMode(
     Communication::Wireless::IMeshDebugInfo* debug, const bool on )
 {
  debuginfo = debug;
  std::string DebugInfoName =
      "/Nodes/" + std::to_string( meshnetworkComponent->getNodeID( ) ) +
      "/debug";

  this->nodeDebugTopic =
      this->rosNode->advertise< abstract_drone::NodeDebugInfo >( DebugInfoName,
                                                                 100 );
  this->NodeDebugInfoThread =
      std::thread( std::bind( &VirtualNRF24::publishDebugInfo, this ) );
 }

 void VirtualNRF24::OnRosMsg( const abstract_drone::NRF24ConstPtr& _msg )
 {
  meshnetworkComponent->OnMsg( _msg->payload.data( ) );
 }

 bool VirtualNRF24::switchPower( abstract_drone::PowerSwitchRequest& request,
                                 abstract_drone::PowerSwitchResponse& response )
 {
  this->on = request.power;
  abstract_drone::DroneInfo nodeinfo;
  nodeinfo.nodeID = this->meshnetworkComponent->getNodeID( );
  nodeinfo.on = this->on;
  rosPub.publish( nodeinfo );
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
   std::this_thread::sleep_for(
       std::chrono::seconds( 1 ) );  // 1hz refresh is enough
   msg.nodeID = debuginfo->getNodeID( );
   msg.ConnectedWithGateway = debuginfo->getConnectedToGateway( );
   msg.familySize = debuginfo->getRouterTechTableSize( );
   msg.totalMessages = debuginfo->getTotalMessageSent( );
   msg.prefferedGateWay = debuginfo->getPrefferedGateway( );
   msg.on = this->on;
   msg.hops = debuginfo->getHopsFromGatewayAway( );
   msg.prefLoc = debuginfo->getLastGoodKnownLocationID( );
   nodeDebugTopic.publish( msg );
  }
 }
}  // namespace WirelessSimulation
}  // namespace ros
