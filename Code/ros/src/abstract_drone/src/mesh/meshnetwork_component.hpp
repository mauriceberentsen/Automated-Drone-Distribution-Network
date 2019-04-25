/**
 * @file meshnetwork_component.hpp
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
// libary
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// Required interfaces
#include "IWireless.hpp"
#include "IRoutingTechnique.hpp"
#include "IDroneEngine.hpp"
// implemented interfaces
#include "VirtualNRF24.hpp"
#include "ChildTableTree.hpp"
#include "rosDroneEngineConnector.hpp"
// classes
#include "message.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkComponent : public ModelPlugin
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
  uint8_t getNodeID( );
  /**
   * @brief Called upon each received Ros Message.
   *
   * @param _msg The received Ros Message. We use NRF24 Message
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
  void sendGoalToDrone( const uint8_t ID, const float latitude,
                        const float longitude, const uint16_t height );

 protected:
  /**
   * @brief Construct a new Meshnetwork Component object
   */
  MeshnetworkComponent( );
  /**
   * @brief Called when a Plugin is first created, and after the World has been
   * loaded. This function should not be blocking
   *
   * @param _parent Pointer to the Model
   * @param _sdf Pointer to the SDF element of the plugin
   */
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
  /**
   * @brief called once after Load for initialization behavior.
   *
   */
  virtual void Init( ) = 0;

  /**
   * @brief Used internal for the drone to fly towards a remebered location
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
   * @brief Handover the message to be sending with the wireless signal
   * simulator
   *
   * @param message Message to be sending
   * @return true Sending was succesfull
   * @return false Sending not succeeded
   */
  bool SendMessage( const uint8_t *message, const uint8_t to );
  /**
   * @brief Process message with the type IntroduceMessage
   * called by case Messages::PRESENT
   * override this function for correct respones upon introduction
   *
   * @param _msg NRF24 Message holding Message of type IntroduceMessage
   */
  virtual void processIntroduction( const uint8_t *message ) = 0;
  /**
   * @brief Process message with the type HeartbeatMessage
   * called by case Messages::HEARTBEAT
   * override this function for correct respones upon Heartbeats
   *
   * @param _msg  NRF24 Message holding Message of type HeartbeatMessage
   */
  virtual void ProcessHeartbeat( const uint8_t *message ) = 0;

  /**
   * @brief Process message with the type requestLocation
   * called by case Messages::REQUESTLOCATION
   *
   * @param _msg NRF24 Message holding normal Message
   */
  void processRequestLocation( const uint8_t *payload );
  /**
   * @brief Process message with the type MovementNegotiationMessage
   * called by case Messages::MOVEMENT_NEGOTIATION
   * override this function to process negotiation message
   *
   * @param _msg  NRF24 Message holding Message of type
   * MovementNegotiationMessage
   */
  virtual void processMovementNegotiationMessage( const uint8_t *message ) = 0;

 private:
  /**
   * @brief Used for forwarding messages. HeartbeatMessage gets an extra hop
   *
   * @param _msg The message to forward
   */
  void forwardMessage( const uint8_t *message );

  /**
   * @brief Used for directing messages to the right functions of our own.
   * Only called upon if the message is meant for us.
   *
   * @param _msg The message received
   */
  void processMessage( const uint8_t *message );
  /**
   * @brief Process message with the type Location
   * called by case Messages::LOCATION
   *
   * @param _msg NRF24 Message holding Location Message
   */
  void processLocation( const uint8_t *message );

  /**
   * @brief Process message with the type MissingMessage
   * called by case Messages::MISSING
   * Inform the router tech that a node went missing
   * Tell others about a missing node
   *
   * @param _msg  NRF24 Message holding Message of type MissingMessage
   */
  void processMissing( const uint8_t *message );
  /**
   * @brief Process message with the type GoToLocationMessage
   * called by case Messages::MOVE_TO_LOCATION
   * Sends the goal to the engine of the connected drone
   *
   * @param _msg  NRF24 Message holding Message of type GoToLocationMessage
   */
  void processSendGoalToEngine( const uint8_t *message );

  /***************** Variables *************************/

 public:
  // No public vars
  // protected:
  /// \brief The time to wait at initialization
  const float initTime = 0.001;
  /// \brief The time to wait before rechecking the Component
  const float CheckConnectionTime = 10.0;
  /// \brief The ID of the current Node
  uint8_t nodeID;
  /// \brief The ID of the prefferedGateWay
  uint8_t prefferedGateWay = 0;
  /// \brief The amount of hops this node is away from the prefferedGateWay
  uint8_t hopsFromGatewayAway = 0;
  /// \brief Boolean for enabling debug publishers
  bool debug = false;

  /// \brief Boolean if this node knows the location of his prefferedGateWay
  bool knowPrefferedGatewayLocation = false;
  /// \brief boolean if this node is connected to a gateway
  bool connectedToGateway = false;
  /// \brief The ID of the connected Drone
  uint16_t droneID;
  /// \brief The amount of messages send by this node
  uint32_t totalMessageSent = 0;
  /// \brief The last known location that was known to be a good location
  Messages::LocationMessage lastGoodKnownLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0, 0, 0, 0 );
  /// \brief The location of the prefferedGateWay
  Messages::LocationMessage prefferedGateWayLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0, 0, 0, 0 );
  /// \brief pointer to this model plugin
  physics::ModelPtr model;
  /// \brief pointer to the used RoutingTechnique
  std::unique_ptr< RoutingTechnique::IRoutingTechnique > routerTech;

  /// \brief Connection towards the DroneEngine
  std::unique_ptr< Drone::IDroneEngine > droneEngine;
  /// \brief Wireless communication
  std::unique_ptr< Wireless::IWirelessCommunication > communication;

 private:
  /// \brief thread to check the connection every CheckConnectionTime seconds
  std::thread checkConnectionThread;
 };
}  // namespace Meshnetwork
}  // namespace gazebo

#endif  // MESHNETWORKCOMPONENT