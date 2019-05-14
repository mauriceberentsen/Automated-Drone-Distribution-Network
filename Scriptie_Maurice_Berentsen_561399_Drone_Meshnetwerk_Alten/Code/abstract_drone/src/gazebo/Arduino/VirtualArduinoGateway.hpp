/**
 * @file VirtualArduinoGateway.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for VirtualArduinoGateway
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALARDUINOGATEWAY
#define VIRTUALARDUINOGATEWAY

#include "IVirtualArduino.hpp"
#include "../../Communication/Meshnetwork/MeshnetworkGateway.hpp"
#include "../../ros/RosInternetMock.hpp"
#include "../../ros/RosDroneEngineConnector.hpp"
#include "../../Communication/RoutingTechnique/HybridLMRoutingProtocol.hpp"
#include "../../ros/WirelessSimulation/VirtualNRF24.hpp"

namespace gazebo
{
namespace ArduinoSimulation
{
 class VirtualArduinoGateway : public IVirtualArduino
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
  /**
   * @brief Initializes the Meshnetwork Gateway
   *
   */
  void setup( );
  /**
   * @brief Not implemented
   *
   */
  void loop( );

 private:
  /// \brief The nodeID for the gateway
  uint8_t nodeID;
  /// \brief The ID of the connected Drone Engine
  uint8_t droneID;
  /// \brief if Debugging should be enabled
  bool debug;
  /// \brief A pointer to the connected MeshnetworkGateway
  Communication::Meshnetwork::MeshnetworkGateway* meshnetworkGateway;
  /// \brief Pointer to the routing technique
  Communication::RoutingTechnique::HybridLMRoutingProtocol* routing;
  /// \brief Pointer to the connected drone eninge
  ros::Drone::RosDroneEngineConnector* engine;
  /// \brief Pointer to the connected NRF24
  ros::WirelessSimulation::VirtualNRF24* NRF24;
  /// \brief Pointer to the internet connection
  ros::Internet::RosInternetMock* internet;
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
 };

}  // namespace ArduinoSimulation
}  // namespace gazebo
#endif  // VIRTUALARDUINOGATEWAY
