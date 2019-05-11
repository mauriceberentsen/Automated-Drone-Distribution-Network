/**
 * @file MeshnetworkGateway.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for the class MeshnetworkGateway
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef MESHNETWORKGATEWAY
#define MESHNETWORKGATEWAY

#include "MeshnetworkComponent.hpp"
#include "../Internet/IInternetConnection.hpp"
#include "../Internet/IGatewayCommands.hpp"

namespace Communication
{
namespace Meshnetwork
{
 class MeshnetworkGateway : public MeshnetworkComponent,
                            public Internet::IGatewayCommands
 {
 public:
  /**
   * @brief Construct a new Meshnetwork Gateway object
   *
   * @param node The ID of the Node
   * @param drone The ID of the connected Drone Engine
   * @param developermode Debuging mode enabled
   * @param IRT Pointer to the RoutingTechnique interface
   * @param IDE Pointer to the DroneEngine interface
   * @param IWC Pointer to the Communication interface
   * @param ICC Pointer to the Internet interface
   */
  MeshnetworkGateway( const uint8_t node, const uint8_t drone,
                      bool developermode,
                      RoutingTechnique::IRoutingTechnique* IRT,
                      Drone::IDroneEngine* IDE,
                      Wireless::IWirelessCommunication* IWC,
                      Internet::IInternetConnection* ICC );

  /**
   * @brief Called after Load
   * -does the following
   *    -# Set gateway parameters
   *    -# Open internet connection
   *    -# Give the node a small moment to settle
   *    -# let the routerTech start Routing
   *
   */
  void Init( );
  /**
   * @brief Stops the gateway
   *
   */
  void Stop( );

  void SendGoalRequestToDrone( const uint8_t ID, const float latitude,
                               const float longitude, const uint16_t height );

 protected:
 private:
  /**
   * @brief Inherited function for introductions. As gateway we do nothing with
   * introductions
   *
   * @param message Pointer to char array[32] holding the message of type
   * IntroductionMessage
   */
  void processIntroduction( const uint8_t* message );
  /**
   * @brief Every $CheckConnectionTime$ let the routerTech maintain routing
   *
   */
  void CheckConnection( );
  /**
   * @brief Because we can't lose connection as gateway in the simulation we do
   * nothing at this moment
   *
   */
  void lostConnection( );
  /**
   * @brief Process HeartbeatMessage's. We always send a heartbeat back
   *
   * @param message Pointer to char array[32] holding the message of type
   * HeartbeatMessage
   */
  void ProcessHeartbeat( const uint8_t* message );
  /**
   * @brief  Our gateway stay where he is. Maybe in the future we need to
   * move the gateway?
   *
   * @param message Pointer to char array[32] holding the message of type
   * NegotiationMessage
   */
  void processMovementNegotiationMessage( const uint8_t* message );

 public:
 protected:
 private:
  /// \brief Internet connection interface
  std::unique_ptr< Internet::IInternetConnection > internet;
 };
 // Register this plugin with the simulator
}  // namespace Meshnetwork
}  // namespace Communication
#endif  // MESHNETWORKGATEWAY