/**
 * @file VirtualArduinoGateway.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Sourcefile for virtual arduino gateway
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "VirtualArduinoGateway.hpp"

namespace gazebo
{
namespace ArduinoSimulation
{
 VirtualArduinoGateway::VirtualArduinoGateway( )
 {
 }

 VirtualArduinoGateway::~VirtualArduinoGateway( )
 {
 }

 void VirtualArduinoGateway::Load( physics::ModelPtr _parent,
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

 //  communication::RoutingTechnique::ChildTableTree CTT;
 //  ros::Drone::RosDroneEngineConnector RDEC;
 //  ros::Internet::RosInternetMock RIM;
 //  ros::WirelessSimulation::VirtualNRF24 NRF24;

 void VirtualArduinoGateway::setup( )
 {
  CTT = new Communication::RoutingTechnique::ChildTableTree( );
  RDEC = new ros::Drone::RosDroneEngineConnector( droneID );
  NRF24 = new ros::WirelessSimulation::VirtualNRF24( );
  RIM = new ros::Internet::RosInternetMock( );
  meshnetworkGateway = new Communication::Meshnetwork::MeshnetworkGateway(
      nodeID, droneID, debug, CTT, RDEC, NRF24, RIM );
  meshnetworkGateway->Init( );
 }

 void VirtualArduinoGateway::loop( )
 {
 }

}  // namespace ArduinoSimulation

GZ_REGISTER_MODEL_PLUGIN( ArduinoSimulation::VirtualArduinoGateway )
}  // namespace gazebo