/**
 * @file IVirtualArduino.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file of interface Virtual Arduino
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALARDUINO
#define VIRTUALARDUINO

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
namespace ArduinoSimulation
{
 class IVirtualArduino : public ModelPlugin
 {
  /**
   * @brief Should be called at start up
   *
   */
  virtual void setup( ) = 0;
  /**
   * @brief Should be filled with code that loops
   *
   */
  virtual void loop( ) = 0;
 };

}  // namespace ArduinoSimulation
}  // namespace gazebo
#endif  // VIRTUALARDUINO
