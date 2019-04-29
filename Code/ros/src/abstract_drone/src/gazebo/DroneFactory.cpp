/**
 * @file DroneFactory.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for the drone DroneFactory
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "DroneFactory.hpp"
#include "DroneSimulation/VirtualDrone.hpp"

namespace gazebo
{
void DroneFactory::Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf )
{
 if ( _sdf->HasElement( "amountOfGatewayDrones" ) ) {
  int amountOfGateways = _sdf->Get< int >( "amountOfGatewayDrones" );
  for ( int i = 0; i < amountOfGateways; i++ ) {
   // the new class will inject sdf information needed in Gazebo
   new DroneSimulation::GatewayDrone( 1, i, 0, _parent );
  }
 }

 if ( _sdf->HasElement( "amountOfRouterDrones" ) ) {
  int amountOfRouters = _sdf->Get< int >( "amountOfRouterDrones" );
  for ( int i = 0; i < amountOfRouters; i++ ) {
   // the new class will inject sdf information needed in Gazebo
   new DroneSimulation::RouterDrone( i, 0, 0, _parent );
  }
 }
}
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( DroneFactory )
}  // namespace gazebo
