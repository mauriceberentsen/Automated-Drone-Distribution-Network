#include "VirtualArduinoCommunicator.hpp"

namespace gazebo
{
namespace Arduino
{
 VirtualArduinoCommunicator::VirtualArduinoCommunicator( )
 {
 }

 VirtualArduinoCommunicator::~VirtualArduinoCommunicator( )
 {
 }

 void VirtualArduinoCommunicator::Load( physics::ModelPtr _parent,
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
  meshnetworkCommunicator =
      new Communication::Meshnetwork::MeshnetworkCommunicator( nodeID, droneID,
                                                               debug );
  setup( );
 }

 void VirtualArduinoCommunicator::setup( )
 {
  meshnetworkCommunicator->Init( );
 }

 void VirtualArduinoCommunicator::loop( )
 {
 }

}  // namespace Arduino

GZ_REGISTER_MODEL_PLUGIN( Arduino::VirtualArduinoCommunicator )
}  // namespace gazebo
