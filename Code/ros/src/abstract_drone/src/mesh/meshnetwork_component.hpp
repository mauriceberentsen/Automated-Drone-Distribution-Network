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
// ros
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ros/callback_queue.h"
// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// local
// ros generated messages
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/NRF24.h"
#include "abstract_drone/NodeDebugInfo.h"
// interfaces
#include "IRoutingTechnique.hpp"
// classes
#include "ChildTableTree.hpp"
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

 protected:
  /**
   * @brief Construct a new Meshnetwork Component object
   * @author M.W.J. Berentsen
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
   * @brief Send a goal to fly towards to another drone
   *
   * @param ID Drone that needs to fly
   * @param longitude
   * @param latitude
   * @param height
   */
  void sendGoalToDrone( const uint8_t ID, const float longitude,
                        const float latitude, const uint16_t height );

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
   * @param message WirelessMessage to be sending
   * @return true Sending was succesfull
   * @return false Sending not succeeded
   */
  bool sendMessage( abstract_drone::WirelessMessage &message );
  /**
   * @brief Process message with the type IntroduceMessage
   * called by case Messages::PRESENT
   * override this function for correct respones upon introduction
   *
   * @param _msg NRF24 Message holding Message of type IntroduceMessage
   */
  virtual void processIntroduction(
      const abstract_drone::NRF24ConstPtr &_msg ) = 0;
  /**
   * @brief Process message with the type HeartbeatMessage
   * called by case Messages::HEARTBEAT
   * override this function for correct respones upon Heartbeats
   *
   * @param _msg  NRF24 Message holding Message of type HeartbeatMessage
   */
  virtual void ProcessHeartbeat(
      const abstract_drone::NRF24ConstPtr &_msg ) = 0;

  /**
   * @brief Process message with the type requestLocation
   * called by case Messages::REQUESTLOCATION
   *
   * @param _msg NRF24 Message holding normal Message
   */
  void processRequestLocation( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Process message with the type MovementNegotiationMessage
   * called by case Messages::MOVEMENT_NEGOTIATION
   * override this function to process negotiation message
   *
   * @param _msg  NRF24 Message holding Message of type
   * MovementNegotiationMessage
   */
  virtual void processMovementNegotiationMessage(
      const abstract_drone::NRF24ConstPtr &_msg ) = 0;

 private:
  /**
   * @brief Called upon each received Ros Message.
   *
   * @param _msg The received Ros Message. We use NRF24 Message
   */
  void OnRosMsg( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Used for forwarding messages. HeartbeatMessage gets an extra hop
   *
   * @param _msg The message to forward
   */
  void forwardMessage( const abstract_drone::NRF24ConstPtr &_msg );

  /**
   * @brief Used for directing messages to the right functions of our own.
   * Only called upon if the message is meant for us.
   *
   * @param _msg The message received
   */
  void processMessage( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Process message with the type Location
   * called by case Messages::LOCATION
   *
   * @param _msg NRF24 Message holding Location Message
   */
  void processLocation( const abstract_drone::NRF24ConstPtr &_msg );

  /**
   * @brief Process message with the type MissingMessage
   * called by case Messages::MISSING
   * Inform the router tech that a node went missing
   * Tell others about a missing node
   *
   * @param _msg  NRF24 Message holding Message of type MissingMessage
   */
  void processMissing( const abstract_drone::NRF24ConstPtr &_msg );
  /**
   * @brief Process message with the type GoToLocationMessage
   * called by case Messages::MOVE_TO_LOCATION
   * Sends the goal to the engine of the connected drone
   *
   * @param _msg  NRF24 Message holding Message of type GoToLocationMessage
   */
  void processSendGoalToEngine( const abstract_drone::NRF24ConstPtr &_msg );

  /**
   * @brief Used to queue up and handle ros message
   *
   */
  void QueueThread( );
  /**
   * @brief ROS service function to switch power on and off
   *
   * @param request NOT USED
   * @param response NOT USED
   * @return true Service call succesfull
   * @return false Service call not succesfull
   */
  bool switchPower( std_srvs::TriggerRequest &request,
                    std_srvs::TriggerResponse &response );
  /**
   * @brief Publishes information about the Component at 1hz
   *
   */
  void publishDebugInfo( );

  /***************** Variables *************************/

 public:
  // No public vars
 protected:
  const float initTime = 0.001;
  const float CheckConnectionTime = 10.0;

  uint8_t nodeID;
  uint8_t prefferedGateWay = 0;
  uint8_t hopsFromGatewayAway = 0;

  bool on = true;
  bool knowPrefferedGatewayLocation = false;
  bool connectedToGateway = false;
  bool isGateway = false;

  uint16_t droneID;
  uint32_t totalMessageSent = 0;

  Messages::LocationMessage lastGoodKnownLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0 );
  Messages::LocationMessage prefferedGateWayLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0 );

  physics::ModelPtr model;

  std::unique_ptr< RoutingTechnique::IRoutingTechnique > routerTech;
  std::unique_ptr< ros::NodeHandle > rosNode;

  ros::CallbackQueue rosQueue;

 private:
  ros::ServiceServer switchPowerService;

  ros::ServiceClient areaScanner;
  ros::ServiceClient GPSLink;
  ros::ServiceClient publishService;

  ros::Publisher rosPub;
  ros::Publisher nodeDebugTopic;
  ros::Publisher droneEnginePublisher;

  ros::Subscriber rosSub;

  std::thread rosQueueThread;
  std::thread heartbeatThread;
  std::thread NodeInfoThread;
 };
}  // namespace Meshnetwork
}  // namespace gazebo

#endif  // MESHNETWORKCOMPONENT