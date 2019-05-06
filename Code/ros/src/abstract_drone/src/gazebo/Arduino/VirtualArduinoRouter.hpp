/**
 * @file VirtualArduinoRouter.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for VirtualArduinoRouter
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALARDUINOROUTER
#define VIRTUALARDUINOROUTER

#include "IVirtualArduino.hpp"
#include "../../Communication/Meshnetwork/MeshnetworkRouter.hpp"

namespace gazebo
{
namespace ArduinoSimulation
{
 class VirtualArduinoRouter : public IVirtualArduino
 {
 public:
  VirtualArduinoRouter( );
  ~VirtualArduinoRouter( );
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
  /// \brief A pointer to the connected MeshnetworkRouter
  Communication::Meshnetwork::MeshnetworkRouter* meshnetworkRouter;
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
  Communication::RoutingTechnique::ChildTableTree* CTT;
  ros::Drone::RosDroneEngineConnector* RDEC;
  ros::WirelessSimulation::VirtualNRF24* NRF24;
 };

}  // namespace ArduinoSimulation

}  // namespace gazebo
#endif  // VIRTUALARDUINOROUTER
