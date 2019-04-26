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

}  // namespace Arduino
}  // namespace gazebo
#endif  // VIRTUALARDUINO
