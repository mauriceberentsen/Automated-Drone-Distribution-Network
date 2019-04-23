/**
 * @file Drone.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for all drone classes
 * @version 1.0
 * @date 2019-04-18
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef DRONE
#define DRONE

#include <sstream>
#include <iostream>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
namespace DroneSimulation
{
 class Drone
 {
 protected:
  /**
   * @brief Construct an abstract Drone

    * @param _x X coordinate in the world
    * @param _y Y coordinate in the world
    * @param _z Z coordinate in the world
    * @param _parent The world to place the drone in
    */
  explicit Drone( const float _x, const float _y, const float _z,
                  physics::WorldPtr _parent );
  ~Drone( );
  static int droneID;
  std::string SdfString;
  const float x;
  const float y;
  const float z;
  const physics::WorldPtr parent;
 };

 class RouterDrone : public Drone
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
   */
  RouterDrone( const float _x, const float _y, const float _z,
               physics::WorldPtr _parent );
  ~RouterDrone( );
 };

 class GatewayDrone : public Drone
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
   */
  GatewayDrone( const float _x, const float _y, const float _z,
                physics::WorldPtr _parent );
  ~GatewayDrone( );
 };
}  // namespace DroneSimulation
}  // namespace gazebo
#endif  // DRONE