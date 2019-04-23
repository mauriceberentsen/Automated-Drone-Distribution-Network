/**
 * @file meshnetwork_communicator.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for MeshnetworkCommunicator
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef MESHNETWORKCOMMUNICATOR
#define MESHNETWORKCOMMUNICATOR
#include <mutex>  // std::mutex

#include "meshnetwork_component.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkCommunicator : public MeshnetworkComponent
 {
 public:
  /**
   * @brief Construct a new Meshnetwork Communicator object
   *
   */
  MeshnetworkCommunicator( );

 protected:
 private:
  /**
   * @brief called once after Load for initialization behavior.
   * Give the system some breathing time with a short sleep.
   * Tell the routerTech to start routing.
   */
  void Init( );
  /**
   * @brief Process message with the type IntroduceMessage
   * called by case Messages::PRESENT
   * If an introducee is connected to a gateway and has less hops we request his
   * location.
   *
   * @param _msg NRF24 Message holding Message of type IntroduceMessage
   */

  void processIntroduction( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Handles heartbeat Messages
   * -possibilities:
   *    - If the sender is a gateway:
   *           - Confirm we are connected with a gateway.
   *           - Set the gateway to be the prefferedGateway.
   *           - Register how many hops we are away.
   *    - If the sender is a node:
   *           - if he doesn't know the gateway but we do:\n
   *             send a heartbeat back to inform him we know a gateway.
   *           - if sender knows gateway and we don't:\n
   *             Add the route to his gateway\n
   *             Set his gateway as prefferedGateWay and try to contact that
   *             gateway.
   *
   * @param _msg  NRF24 Message holding Message of type HeartbeatMessage.
   */
  void ProcessHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Running in a thread this node preforms the following actions.
   * -actions:
   *    -# Every $CheckConnectionTime$ seconds.
   *    -# Tell the routertech to maintain their routing.
   *    -# Check for nodes nearby.
   *    -# if connected to the gateway send a heartbeat to him.
   *    -# if we dont know also ask for his location.
   *    -# if we aren't connected to the gateway we start the lostConnection\n
   *       protocol.
   */
  void CheckConnection( );
  /**
   * @brief Send an heartbeat to the prefferedGateway.
   *
   */
  void sendHeartbeatToGateway( );
  /**
   * @brief Protocol to follow after connection loss.
   *    - Connection lost protocol:
   *         -# Start a timer and wait for 30 seconds, it might come back.
   *         -# We are really lost go into StartEmergencyProtocol.
   *
   */
  void lostConnection( );
  /**
   * @brief Emergency protocol activated after a long time of no connectivity
   * -Emergency Protocol:
   *    - Check one more time if we have possible connection, if we do.
   *    - find out if we are alone if we are then we need to move back to\n
   *      lastGoodKnownLocation.
   *    - When we are in a group start a negotiation on who should move.
   *
   */
  void StartEmergencyProtocol( );
  /**
   * @brief Start negotiation on who should move
   * The one with the highest code should move.
   * In this implementation the cost is based on the distance to the gateway.
   * Only drones that want to move to a location where no other working drone is
   * active may move.
   * This function wait fors all surrounding nodes to react with their cost.
   * After that the negotiationList is cleared.
   * After moving to the lastGoodKnownLocation the next location will be of the
   * gateway when the drone is still lost.
   *
   */
  void startMovementNegotiation( );
  /**
   * @brief Add a pair to the negotiationList while using a mutex lock to
   * prevent data corruption
   *
   * @param val the pair value you want to add
   */
  void SafeAddToNegotiationList( const std::pair< float, uint8_t > &val );
  /**
   * @brief Send the surrounding node the cost for movement
   *
   * @param cost of this node
   */
  void informOthersAboutCost( float cost );
  /**
   * @brief Procces message for movement negotiation.
   * Add the cost of other node to the list.
   *
   * @param _msg NRF24 MovementNegotiationMessage
   */
  void processMovementNegotiationMessage(
      const abstract_drone::NRF24ConstPtr &_msg );

 public:
 protected:
 private:
  /// \brief Mutex lock to prevent data corruption when adding information to
  /// the negotiationList
  std::mutex mtx;
  /// \brief Boolean for checking if the timer is running during connection loss
  bool timerStarted = false;
  /// \brief multimap sorted on cost, used to determine which node should move
  std::multimap< float, uint8_t > negotiationList;
  /// \brief Time variable to register that last moment that this node was
  /// connected to a gateway
  common::Time lastTimeOnline;
  /// \brief Allowed time that no connection is available
  const float timeUntilConnectionLost = 30.0;
 };
}  // namespace Meshnetwork
GZ_REGISTER_MODEL_PLUGIN( Meshnetwork::MeshnetworkCommunicator )

// Register this plugin with the simulator
}  // namespace gazebo
#endif  // MESHNETWORKCOMMUNICATOR