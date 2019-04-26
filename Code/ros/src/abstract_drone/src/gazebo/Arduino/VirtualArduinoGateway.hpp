#ifndef VIRTUALARDUINOGATEWAY
#define VIRTUALARDUINOGATEWAY
#include "VirtualArduino.hpp"

#include "../../Communication/Meshnetwork/MeshnetworkGateway.hpp"

namespace gazebo
{
namespace Arduino
{
 class VirtualArduinoGateway : public VirtualArduino
 {
 public:
  VirtualArduinoGateway( );
  ~VirtualArduinoGateway( );
  /**
   * @brief Called when a Plugin is first created, and after the World has been
   * loaded. This function should not be blocking
   *
   * @param _parent Pointer to the Model
   * @param _sdf Pointer to the SDF element of the plugin
   */
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

 private:
  void setup( );
  void loop( );

 private:
  uint8_t nodeID;
  uint8_t droneID;
  bool debug;
  Communication::Meshnetwork::MeshnetworkGateway* meshnetworkGateway;
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
 };

}  // namespace Arduino
}  // namespace gazebo
#endif  // VIRTUALARDUINOGATEWAY
