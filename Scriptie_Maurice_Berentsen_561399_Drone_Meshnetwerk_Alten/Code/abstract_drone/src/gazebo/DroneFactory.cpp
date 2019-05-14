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
 bool debug = false;

 if ( _sdf->HasElement( "Debug" ) ) { debug = _sdf->Get< bool >( "Debug" ); }

 if ( _sdf->HasElement( "amountOfGatewayDrones" ) ) {
  int amountOfGateways = _sdf->Get< int >( "amountOfGatewayDrones" );
  for ( int i = 0; i < amountOfGateways; i++ ) {
   // the new class will inject sdf information needed in Gazebo
   new DroneSimulation::GatewayDrone( -1, i, 0, _parent, debug );
  }
 }

 int gridsize = 6;

 if ( _sdf->HasElement( "gridsize" ) ) {
  gridsize = _sdf->Get< int >( "gridsize" );
 }

 if ( _sdf->HasElement( "amountOfRouterDrones" ) ) {
  int amountOfRouters = _sdf->Get< int >( "amountOfRouterDrones" );
  for ( int i = 0; i < amountOfRouters; i++ ) {
   // the new class will inject sdf information needed in Gazebo
   int row, col;
   row = i % gridsize;
   col = std::floor( i / gridsize );
   new DroneSimulation::RouterDrone( row, col, 0, _parent, debug );
  }
 }
}
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( DroneFactory )
}  // namespace gazebo
