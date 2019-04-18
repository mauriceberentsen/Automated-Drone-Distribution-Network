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
  static int droneID;
  explicit Drone( const float _x, const float _y, const float _z,
                  physics::WorldPtr _parent );
  ~Drone( );
  const float x, y, z;
  std::string SdfString;
  physics::WorldPtr parent;
 };

 class RouterDrone : public Drone
 {
 public:
  RouterDrone( const float _x, const float _y, const float _z,
               physics::WorldPtr _parent );
  ~RouterDrone( );
 };

 class GatewayDrone : public Drone
 {
 public:
  GatewayDrone( const float _x, const float _y, const float _z,
                physics::WorldPtr _parent );
  ~GatewayDrone( );
 };
}  // namespace DroneSimulation
}  // namespace gazebo