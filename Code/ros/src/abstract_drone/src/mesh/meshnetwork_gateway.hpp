/**
 * @file meshnetwork_gateway.hpp
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

#include "meshnetwork_component.hpp"
#include "abstract_drone/RequestGatewayDroneFlight.h"

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkGateway : public MeshnetworkComponent
 {
 public:
  /**
   * @brief Construct a new Meshnetwork Gateway object
   *
   */
  MeshnetworkGateway( );

 protected:
 private:
  /**
   * @brief Through a subscriber handle incoming gateway messages
   *
   * @param _msg RequestGatewayDroneFlight messages
   */
  void gatewayQueue(
      const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg );
  /**
   * @brief Called after Load
   * -does the following
   *    -# Set gateway parameters
   *    -# Subscribe to the gateway topic
   *    -# Give the node a small moment to settle
   *    -# let the routerTech start Routing
   *
   */
  void Init( );
  /**
   * @brief Inherited function for introductions. As gateway we do nothing with
   * introductions
   *
   * @param _msg
   */
  void processIntroduction( const abstract_drone::NRF24ConstPtr &_msg );
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
   * @param _msg NRF24 Message containing a HeartbeatMessage
   */
  void ProcessHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief  Our gateway stay where he is. Maybe in the future we need to
   * move the gateway?
   * @param _msg Message containing a NegotiationMessage
   */
  void processMovementNegotiationMessage(
      const abstract_drone::NRF24ConstPtr &_msg );

 public:
 protected:
 private:
  /// \brief Subscriber to the general gateway topic
  // ros::Subscriber gatewaySub;
 };
 // Register this plugin with the simulator
}  // namespace Meshnetwork
GZ_REGISTER_MODEL_PLUGIN( Meshnetwork::MeshnetworkGateway )
}  // namespace gazebo
#endif  // MESHNETWORKGATEWAY