#ifndef VIRTUALARDUINO
#define VIRTUALARDUINO

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
namespace Arduino
{
 class VirtualArduino : public ModelPlugin
 {
  virtual void setup( ) = 0;
  virtual void loop( ) = 0;
 };

}  // namespace Arduino
}  // namespace gazebo
#endif  // VIRTUALARDUINO
