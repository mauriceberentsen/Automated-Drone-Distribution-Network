#include "VirtualArduinoGateway.hpp"

namespace gazebo
{
namespace Arduino
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
  meshnetworkGateway = new Communication::Meshnetwork::MeshnetworkGateway(
      nodeID, droneID, debug );
  setup( );
 }

 void VirtualArduinoGateway::setup( )
 {
  meshnetworkGateway->Init( );
 }

 void VirtualArduinoGateway::loop( )
 {
 }

}  // namespace Arduino

GZ_REGISTER_MODEL_PLUGIN( Arduino::VirtualArduinoGateway )
}  // namespace gazebo