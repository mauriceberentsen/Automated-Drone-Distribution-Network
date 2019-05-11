/**
 * @file VirtualArduinoRouter.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for VirtualArduinoRouter
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "VirtualArduinoRouter.hpp"

namespace gazebo
{
namespace ArduinoSimulation
{
 VirtualArduinoRouter::VirtualArduinoRouter( )
 {
 }

 VirtualArduinoRouter::~VirtualArduinoRouter( )
 {
 }

 void VirtualArduinoRouter::Load( physics::ModelPtr _parent,
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

 void VirtualArduinoRouter::setup( )
 {
  routing = new Communication::RoutingTechnique::HybridLMRoutingProtocol( );
  engine = new ros::Drone::RosDroneEngineConnector( droneID );
  NRF24 = new ros::WirelessSimulation::VirtualNRF24( );
  meshnetworkRouter = new Communication::Meshnetwork::MeshnetworkRouter(
      nodeID, droneID, debug, routing, engine, NRF24 );
  meshnetworkRouter->Init( );
 }

 void VirtualArduinoRouter::loop( )
 {
 }

}  // namespace ArduinoSimulation

GZ_REGISTER_MODEL_PLUGIN( ArduinoSimulation::VirtualArduinoRouter )
}  // namespace gazebo
