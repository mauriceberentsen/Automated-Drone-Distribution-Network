#ifndef VIRTUALARDUINOCOMMUNICATOR
#define VIRTUALARDUINOCOMMUNICATOR
#include "VirtualArduino.hpp"

#include "../../Communication/Meshnetwork/MeshnetworkCommunicator.hpp"

namespace gazebo
{
namespace Arduino
{
 class VirtualArduinoCommunicator : public VirtualArduino
 {
 public:
  VirtualArduinoCommunicator( );
  ~VirtualArduinoCommunicator( );
  /**
   * @brief Called when a Plugin is first created, and after the World has been
   * loaded. This function should not be blocking
   *
   * @param _parent Pointer to the Model
   * @param _sdf Pointer to the SDF element of the plugin
   */
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  void setup( );
  void loop( );

 private:
  uint8_t nodeID;
  uint8_t droneID;
  bool debug;
  Communication::Meshnetwork::MeshnetworkCommunicator* meshnetworkCommunicator;
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
 };

}  // namespace Arduino

}  // namespace gazebo
#endif  // VIRTUALARDUINOCOMMUNICATOR
