#ifndef VIRTUALARDUINOCOMMUNICATOR
#define VIRTUALARDUINOCOMMUNICATOR

#include "IVirtualArduino.hpp"
#include "../../Communication/Meshnetwork/MeshnetworkCommunicator.hpp"

namespace gazebo
{
namespace Arduino
{
 class VirtualArduinoCommunicator : public IVirtualArduino
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
  /**
   * @brief Initializes the Meshnetwork Communicator
   *
   */
  void setup( );
  void loop( );

 private:
  /// \brief The nodeID for the gateway
  uint8_t nodeID;
  /// \brief The ID of the connected Drone Engine
  uint8_t droneID;
  /// \brief if Debugging should be enabled
  bool debug;
  /// \brief A pointer to the connected MeshnetworkCommunicator
  Communication::Meshnetwork::MeshnetworkCommunicator* meshnetworkCommunicator;
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
 };

}  // namespace Arduino

}  // namespace gazebo
#endif  // VIRTUALARDUINOCOMMUNICATOR
