/**
 * @file IVirtualRaspberryPi.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file of interface Virtual RaspberryPi
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALRASPBERRYPI
#define VIRTUALRASPBERRYPI

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
namespace RaspberryPiSimulation
{
 class IVirtualRaspberryPi : public ModelPlugin
 {
  /**
   * @brief Should be called one time at start up
   *
   */
  virtual void setup( ) = 0;
  /**
   * @brief Should be filled with code that loops
   *
   */
  virtual void loop( ) = 0;
 };

}  // namespace RaspberryPiSimulation
}  // namespace gazebo
#endif  // VIRTUALRASPBERRYPI
