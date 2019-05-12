/**
 * @file MeshnetworkComponent.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header for the abstract class MeshnetworkComponent,
 * generalisation for communication parts in a Meshnetwork
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef MESHNETWORKCOMPONENT
#define MESHNETWORKCOMPONENT
// system
#include <map>
#include <thread>
#include <functional>
// Offered interfaces
#include "../RoutingTechnique/IRoutingEssentials.hpp"
#include "../Wireless/IMeshNetwork.hpp"
#include "../Wireless/IMeshDebugInfo.hpp"
// Required interfaces
#include "../RoutingTechnique/IRoutingTechnique.hpp"
#include "../Wireless/IWirelessCommunication.hpp"
#include "../../Drone/IDroneEngine.hpp"
// classes
#include "../Messages/Message.hpp"
namespace Communication
{
namespace Meshnetwork
{
 class MeshnetworkComponent : public RoutingTechnique::IRoutingEssentials,
                              public Wireless::IMeshNetwork,
                              public Wireless::IMeshDebugInfo
 {
 public:
  /**
   * @brief Send a heartbeat message using wireless communication
   *
   * @param other The destination node to recieve the heartbeat
   * @return true Message sent away node doesn't need to be destination
   * @return false Message sending failed
   */
  bool sendHeartbeat( uint8_t other );

  /**
   * @brief Broadcast a introduction to every node nearby
   *
   */
  void searchOtherNodesInRange( );

  /**
   * @brief Send a IntroduceMessage to a nearby node
   * Cant be used for node out of range
   *
   * @param other Node to introduce yourself to
   */
  void IntroduceNode( uint8_t other );
  /**
   * @brief Get the nodeID
   *
   * @return uint8_t this->nodeID
   */
  const uint8_t getNodeID( ) const;
  /**
   * @brief Get the ConnectedToGateway
   *
   * @return true connected
   * @return false not connected
   */
  const bool getConnectedToGateway( ) const;
  /**
   * @brief Get the Total Message Sent
   *
   * @return const uint32_t amount
   */
  const uint32_t getTotalMessageSent( ) const;
  /**
   * @brief Get the Preffered Gateway ID
   *
   * @return const uint8_t ID
   */
  const uint8_t getPrefferedGateway( ) const;
  /**
   * @brief Get theHopsFromGatewayAway
   *
   * @return const uint8_t amount of hops away
   */
  const uint8_t getHopsFromGatewayAway( ) const;
  /**
   * @brief Get the ID of LastGoodKnownLocation
   *
   * @return const uint8_t ID
   */
  const uint8_t getLastGoodKnownLocationID( ) const;
  /**
   * @brief Get the size of the RouterTechTable
   *
   * @return const uint8_t
   */
  const uint8_t getRouterTechTableSize( ) const;
  /**
   * @brief Called upon each received Message.
   *
   * @param message Pointer to char array[32] holding the message
   */
  void OnMsg( const uint8_t *message );

  /**
   * @brief Send a goal to fly towards to another drone
   *
   * @param ID Drone that needs to fly
   * @param latitude
   * @param longitude
   * @param height
   */
  void SendGoalToDrone( const uint8_t ID, const float latitude,
                        const float longitude, const uint16_t height );

  /**
   * @brief called once after Load for initialization behavior.
   *
   */
  virtual void Init( ) = 0;

 protected:
  /**
   * @brief Construct a new Meshnetwork Component object
   *
   * @param node The ID of the Node
   * @param drone The ID of the connected Drone Engine
   * @param developermode Debuging mode enabled
   * @param IRT Pointer to the RoutingTechnique interface
   * @param IDE Pointer to the DroneEngine interface
   * @param IWC Pointer to the Communication interface
   */
  MeshnetworkComponent( const uint8_t node, const uint8_t drone,
                        bool developermode,
                        RoutingTechnique::IRoutingTechnique *IRT,
                        Drone::IDroneEngine *IDE,
                        Wireless::IWirelessCommunication *IWC );

  /**
   * @brief Used internal to tell the drone to fly towards a location
   *
   * @param _msg LocationMessage with the location to fly towards
   */
  void sendGoalToEngine( const Messages::LocationMessage &_msg );

  /**
   * @brief Start up protocol to follow when the connection is lost
   *
   */
  virtual void lostConnection( ) = 0;
  /**
   * @brief Inform connected childs about a node that went missing
   *
   * @param parent The parent of the missing child
   * @param child The child that went missing
   */
  void informAboutMissingChild( uint8_t parent, uint8_t child );
  /**
   * @brief Request the location of another MeshnetworkComponent
   *
   * @param other the node you want the location from
   */
  void requestLocation( const uint8_t other );
  /**
   * @brief Send your location to another MeshnetworkComponent
   *
   * @param other receiving Component
   */
  void sendLocation( const uint8_t other );
  /**
   * @brief Uses pythagoras to calculate the distance between self and given
   * location using latitude, altitude and height
   *
   * @param LocMsg LocationMessage holding the other location
   * @return float the distance in meters
   */
  float distanceBetweenMeAndLocation( const Messages::LocationMessage &LocMsg );
  /**
   * @brief Execute tasks that ensure you connection
   *
   */
  virtual void CheckConnection( ) = 0;
  /**
   * @brief Handover the message to be sent to the antenna
   *
   * @param message Pointer to the char[32] message to be sending
   * @param to The addressee of the message
   * @return true Sending was succesfull
   * @return false Sending not succeeded
   */
  bool SendMessage( const uint8_t *message, const uint8_t to );
  /**
   * @brief Process message with the type IntroduceMessage
   * called by case Messages::PRESENT
   * override this function for correct respones upon introduction
   *
   * @param message Pointer to char array[32] holding the message of type
   * IntroduceMessage
   */
  virtual void processIntroduction( const uint8_t *message ) = 0;
  /**
   * @brief Process message with the type HeartbeatMessage
   * called by case Messages::HEARTBEAT
   * override this function for correct respones upon Heartbeats
   *
   * @param message Pointer to char array[32] holding the message of type
   * HeartbeatMessage
   */
  virtual void ProcessHeartbeat( const uint8_t *message ) = 0;

  /**
   * @brief Process message with the type requestLocation
   * called by case Messages::REQUESTLOCATION
   *
   * @param message Pointer to char array[32] holding the message with type
   * REQUESTLOCATION
   */
  void processRequestLocation( const uint8_t *message );
  /**
   * @brief Process message with the type MovementNegotiationMessage
   * called by case Messages::MOVEMENT_NEGOTIATION
   * override this function to process negotiation message
   *
   * @param message Pointer to char array[32] holding the message of type
   * MovementNegotiationMessage
   */
  virtual void processMovementNegotiationMessage( const uint8_t *message ) = 0;

 private:
  /**
   * @brief Used for forwarding messages. HeartbeatMessage gets an extra hop
   *
   * @param message Pointer to char array[32] holding the message to forward
   */
  void forwardMessage( const uint8_t *message );

  /**
   * @brief Used for directing messages to the right functions of our own.
   * Only called upon if the message is meant for us.
   *
   *@param message Pointer to char array[32] holding the message to process
   */
  void processMessage( const uint8_t *message );
  /**
   * @brief Process message with the type Location
   * called by case Messages::LOCATION
   *
   * @param message Pointer to char array[32] holding the message of type
   * LocationMessage
   */
  void processLocation( const uint8_t *message );

  /**
   * @brief Process message with the type MissingMessage
   * called by case Messages::MISSING
   * Inform the router tech that a node went missing
   * Tell others about a missing node
   *
   * @param message Pointer to char array[32] holding the message of type
   * MissingMessage
   */
  void processMissing( const uint8_t *message );
  /**
   * @brief Process message with the type GoToLocationMessage
   * called by case Messages::MOVE_TO_LOCATION
   * Sends the goal to the engine of the connected drone
   *
   * @param message Pointer to char array[32] holding the message of type
   * GoToLocationMessage
   */
  void processSendGoalToEngine( const uint8_t *message );

  /***************** Variables *************************/

 public:
  // No public variables
 protected:
  /// \brief The time to wait before rechecking the Component
  const int CheckConnectionTime = 10;
  /// \brief The ID of the current Node
  uint8_t nodeID;
  /// \brief The ID of the connected Drone
  uint16_t droneID;
  /// \brief Boolean for enabling debug publishers
  bool debug = false;
  /// \brief pointer to the used RoutingTechnique
  std::unique_ptr< RoutingTechnique::IRoutingTechnique > routerTech;
  /// \brief Connection towards the DroneEngine
  std::unique_ptr< Drone::IDroneEngine > droneEngine;
  /// \brief Wireless communication interface
  std::unique_ptr< Wireless::IWirelessCommunication > communication;

  /// \brief The ID of the prefferedGateWay
  uint8_t prefferedGateWay = 0;
  /// \brief The amount of hops this node is away from the prefferedGateWay
  uint8_t hopsFromGatewayAway = 0;
  /// \brief Boolean if this node knows the location of his prefferedGateWay
  bool knowPrefferedGatewayLocation = false;
  /// \brief boolean if this node is connected to a gateway
  bool connectedToGateway = false;
  /// \brief The amount of messages send by this node
  uint32_t totalMessageSent = 0;
  /// \brief The last known location that was known to be a good location
  Messages::LocationMessage lastGoodKnownLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0, 0, 0, 0 );
  /// \brief The location of the prefferedGateWay
  Messages::LocationMessage prefferedGateWayLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0, 0, 0, 0 );

 private:
  /// \brief thread to check the connection every CheckConnectionTime seconds
  std::thread checkConnectionThread;
 };
}  // namespace Meshnetwork
}  // namespace Communication
#endif  // MESHNETWORKCOMPONENT