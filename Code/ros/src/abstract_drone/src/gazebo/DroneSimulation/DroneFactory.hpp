/**
 * @file DroneFactory.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the drone DroneFactory
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef DRONEFACTORY
#define DRONEFACTORY

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
namespace DroneSimulation
{
 class DroneFactory : public WorldPlugin
 {
 public:
  /**
   * @brief This function is called when loading the plugin into gazebo. It's
   * created the amount of Gateways given in the SDF <amountOfGatewayDrones> and
   * the amount of routers given in <amountOfRouterDrones>
   *
   * @param _parent pointer to the World this plugin lives in
   * @param _sdf pointer to the SDF of this plugin
   */
  void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf );

 protected:
 private:
 };
}  // namespace DroneSimulation
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( DroneSimulation::DroneFactory )
}  // namespace gazebo
#endif  // DRONEFACTORY
