#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "Drone.hpp"

namespace gazebo
{
namespace DroneSimulation
{
 class DroneFactory : public WorldPlugin
 {
 public:
  void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf )
  {

   if ( _sdf->HasElement( "amountOfGatewayDrones" ) ) {
    int amountOfGateways = _sdf->Get< int >( "amountOfGatewayDrones" );
    for ( int i = 0; i < amountOfGateways; i++ ) {
     // the new class will inject sdf information needed in Gazebo
     new GatewayDrone( 1, i, 0, _parent );
    }
   }

   if ( _sdf->HasElement( "amountOfRouterDrones" ) ) {
    int amountOfRouters = _sdf->Get< int >( "amountOfRouterDrones" );
    for ( int i = 0; i < amountOfRouters; i++ ) {
     // the new class will inject sdf information needed in Gazebo
     new RouterDrone( i, 0, 0, _parent );
    }
   }
  }
 };
}  // namespace DroneSimulation

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( DroneSimulation::DroneFactory )
}  // namespace gazebo
