/**
 * @file VirtualDrone.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for all drone classes
 * @version 1.0
 * @date 2019-04-18
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALDRONE
#define VIRTUALDRONE

#include <sstream>
#include <iostream>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
namespace DroneSimulation
{
 class VirtualDrone
 {
 protected:
  /**
   * @brief Construct an abstract Drone

    * @param _x X coordinate in the world
    * @param _y Y coordinate in the world
    * @param _z Z coordinate in the world
    * @param _parent The world to place the drone in
    * @param debug debugging on or off
    */
  explicit VirtualDrone( const float _x, const float _y, const float _z,
                         physics::WorldPtr _parent, bool debug = false );
  ~VirtualDrone( );
  /// \brief static Drone counter to give every drone an unique ID as long as
  /// the same factory is used
  static int droneID;
  /// \brief SDF string containing all information about the drone for gazebo
  std::string SdfString;
  /// \brief X coordinate in the simulation
  const float x;
  /// \brief Y coordinate in the simulation
  const float y;
  /// \brief Z coordinate in the simulation
  const float z;
  /// \brief Pointer to the World the drone will be placed in
  const physics::WorldPtr parent;
  /// \brief Debugging mode on or off
  const bool debug;
 };

 class RouterDrone : public VirtualDrone
 {
 public:
  /**
   * @brief Construct a new Router Drone its creates the SDF string needed for
   * injecting in the world. Important parts are: The coordinates, DroneID,
   * NodeID, RouterComponent, DroneEngine and shape.
   *
   * @param _x X coordinate in the world
   * @param _y Y coordinate in the world
   * @param _z Z coordinate in the world
   * @param _parent The world to place the drone in
   * @param debug debugging on or off
   */
  RouterDrone( const float _x, const float _y, const float _z,
               physics::WorldPtr _parent, bool debug = false );
  ~RouterDrone( );
 };

 class GatewayDrone : public VirtualDrone
 {
 public:
  /**
   * @brief Construct a new Router Drone its creates the SDF string needed for
   * injecting in the world. Important parts are: The coordinates, DroneID,
   * NodeID, GatewayComponent, DroneEngine and shape.
   *
   * @param _x X coordinate in the world
   * @param _y Y coordinate in the world
   * @param _z Z coordinate in the world
   * @param _parent The world to place the drone in
   * @param debug debugging on or off
   */
  GatewayDrone( const float _x, const float _y, const float _z,
                physics::WorldPtr _parent, bool debug = false );
  ~GatewayDrone( );
 };
}  // namespace DroneSimulation
}  // namespace gazebo
#endif  // DRONE