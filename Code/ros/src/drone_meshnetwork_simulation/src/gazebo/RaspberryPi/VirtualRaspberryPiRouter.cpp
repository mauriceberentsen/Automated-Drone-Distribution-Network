/**
 * @file VirtualRaspberryPiRouter.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for VirtualRaspberryPiRouter
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "VirtualRaspberryPiRouter.hpp"

namespace gazebo
{
namespace RaspberryPiSimulation
{
 VirtualRaspberryPiRouter::VirtualRaspberryPiRouter( )
     : nodeID( 0 )
     , droneID( 0 )
     , debug( false )
     , meshnetworkRouter( nullptr )
     , routing( nullptr )
     , engine( nullptr )
     , NRF24( nullptr )
 {
 }

 VirtualRaspberryPiRouter::~VirtualRaspberryPiRouter( )
 {
 }

 void VirtualRaspberryPiRouter::Load( physics::ModelPtr _parent,
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
    <DroneID> unique <DroneID> \n  Restart ussing droneID's else drone \
    movement will be a mess" );
  }
  if ( _sdf->HasElement( "Debug" ) ) {
   this->debug = _sdf->Get< bool >( "Debug" );
  }
  setup( );
 }

 void VirtualRaspberryPiRouter::setup( )
 {
  routing = new Communication::RoutingTechnique::HybridLMRoutingProtocol( );
  engine = new ros::Drone::RosDroneEngineConnector( droneID );
  NRF24 = new ros::WirelessSimulation::VirtualNRF24( );
  meshnetworkRouter = new Communication::Meshnetwork::MeshnetworkRouter(
      nodeID, droneID, debug, routing, engine, NRF24 );
  meshnetworkRouter->Init( );
 }

 void VirtualRaspberryPiRouter::loop( )
 {
 }

}  // namespace RaspberryPiSimulation

GZ_REGISTER_MODEL_PLUGIN( RaspberryPiSimulation::VirtualRaspberryPiRouter )
}  // namespace gazebo
