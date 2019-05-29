/**
 * @file VirtualRaspberryPiGatewayWithServer.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Sourcefile for virtual RaspberryPi gateway
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "VirtualRaspberryPiGatewayWithServer.hpp"

namespace gazebo
{
namespace RaspberryPiSimulation
{
 VirtualRaspberryPiGatewayWithServer::VirtualRaspberryPiGatewayWithServer( )
     : nodeID( 0 )
     , droneID( 0 )
     , debug( false )
     , meshnetworkGateway( nullptr )
     , routing( nullptr )
     , engine( nullptr )
     , NRF24( nullptr )
     , internet( nullptr )
 {
 }

 VirtualRaspberryPiGatewayWithServer::~VirtualRaspberryPiGatewayWithServer( )
 {
 }
 // cppcheck-suppress unusedFunction gazebo uses this function
 void VirtualRaspberryPiGatewayWithServer::Load( physics::ModelPtr _parent,
                                   sdf::ElementPtr _sdf )
 {
  // Store the pointer to the model
  this->model = _parent;
  if ( _sdf->HasElement( "nodeID" ) ) {
   this->nodeID = _sdf->Get< int >( "nodeID" );
  }
  if ( _sdf->HasElement( "DroneID" ) ) {
   this->droneID = _sdf->Get< int >( "DroneID" );
  } else {
   ROS_ERROR(
       "Drone com being used without a droneid, set up using the tag \
    <DroneID> unique <DroneID> \n  Restart using droneID's else drone \
    movement will be a mess" );
  }
  if ( _sdf->HasElement( "Debug" ) ) {
   this->debug = _sdf->Get< bool >( "Debug" );
  }
  StartSoftware( );
 }

 void VirtualRaspberryPiGatewayWithServer::StartSoftware( )
 {
  routing = new Communication::RoutingTechnique::HybridLMRoutingProtocol( );
  engine = new ros::Drone::RosDroneEngineConnector( droneID );
  NRF24 = new ros::WirelessSimulation::VirtualNRF24( );
  internet = new InternetGateway( );
  
  meshnetworkGateway = new Communication::Meshnetwork::MeshnetworkGateway(
      nodeID, droneID, debug, routing, engine, NRF24, internet );
  meshnetworkGateway->Init( );
 }


}  // namespace RaspberryPiSimulation

GZ_REGISTER_MODEL_PLUGIN( RaspberryPiSimulation::VirtualRaspberryPiGatewayWithServer )
}  // namespace gazebo