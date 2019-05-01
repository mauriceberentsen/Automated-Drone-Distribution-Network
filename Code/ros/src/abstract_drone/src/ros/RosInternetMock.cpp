/**
 * @file RosInternetMock.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for RosInternetMock
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
// gateway header
#include "../Communication/Meshnetwork/MeshnetworkGateway.hpp"
// local header
#include "RosInternetMock.hpp"

namespace ros
{
namespace Internet
{
 RosInternetMock::RosInternetMock(
     Communication::Meshnetwork::MeshnetworkGateway &MG )
     : meshnetworkGateway( MG )
 {
  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "RosInternetMock",
              ros::init_options::NoSigintHandler );
  }
  this->rosNode.reset( new ros::NodeHandle( "droneEngineConnector" ) );

  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread( std::bind( &RosInternetMock::QueueThread, this ) );
 }

 RosInternetMock::~RosInternetMock( )
 {
  disconnect( );
 }

 void RosInternetMock::disconnect( )
 {
  this->gatewaySub.shutdown( );
 }

 void RosInternetMock::connect( )
 {
  ros::SubscribeOptions so = ros::SubscribeOptions::create<
      abstract_drone::RequestGatewayDroneFlight >(
      "/gateway", 1000, boost::bind( &RosInternetMock::gatewayQueue, this, _1 ),
      ros::VoidPtr( ), &this->rosQueue );

  this->gatewaySub = this->rosNode->subscribe( so );
  ROS_INFO( "gateway connected to virtual internet" );
 }
 void RosInternetMock::QueueThread( )
 {
  static const double timeout = 0.01;
  while ( this->rosNode->ok( ) ) {
   this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
  }
 }

 void RosInternetMock::gatewayQueue(
     const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg )
 {
  meshnetworkGateway.sendGoalToDrone( _msg->ID, _msg->latitude, _msg->longitude,
                                      _msg->height );
 }

}  // namespace Internet
}  // namespace ros