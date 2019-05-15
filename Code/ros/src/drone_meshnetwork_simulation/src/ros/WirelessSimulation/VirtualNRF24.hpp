/**
 * @file VirtualNRF24.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for VirtualNRF24
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef VIRTUALNRF24
#define VIRTUALNRF24

#include <thread>
// ros
#include "ros/ros.h"
#include "ros/callback_queue.h"
// ros generated messages
#include "drone_meshnetwork_simulation/WirelessMessage.h"
#include "drone_meshnetwork_simulation/PowerSwitch.h"
#include "drone_meshnetwork_simulation/NRF24.h"
#include "drone_meshnetwork_simulation/NodeDebugInfo.h"
// offered interface
#include "../../Communication/Wireless/IWirelessCommunication.hpp"
// Required interface
#include "../../Communication/Wireless/IMeshNetwork.hpp"
#include "../../Communication/Wireless/IMeshDebugInfo.hpp"

namespace Communication
{
namespace Meshnetwork
{
 class MeshnetworkComponent;
}
}  // namespace Communication
namespace ros
{
namespace WirelessSimulation
{
 class VirtualNRF24 : public Communication::Wireless::IWirelessCommunication
 {
 public:
  /**
   * @brief Construct a new VirtualNRF24 object
   *
   */
  VirtualNRF24( );
  /**
   * @brief Destroy the Virtual NRF24 object
   *
   */
  ~VirtualNRF24( );
  /**
   * @brief Start the communication pipe of the NRF24
   *
   * @param MC Reference to the connected MeshnetworkComponent
   */
  void StartAntenna( Communication::Wireless::IMeshNetwork* MC );
  /**
   * @brief Stop the communication pipe
   *
   */
  void StopAntenna( );
  /**
   * @brief Send a payload
   *
   * @param msg uint8_t pointer to the message to be send
   * @return true Sending was succesfull
   * @return false Sending didn't succeed
   */
  bool SendMessageTo( const uint8_t* msg );
  /**
   * @brief Broadcast a message to all nodes nearby
   *
   * @param msg pointer to the message to be send
   */
  void BroadcastMessage( const uint8_t* msg );
  /**
   * @brief debuging activation
   *
   * @param debug Pointer to the debug interface of the meshnetworkComponent
   * @param on state
   */
  void DebugingMode( Communication::Wireless::IMeshDebugInfo* debug,
                     const bool on = true );
  /**
   * @brief Handles incomming ROS messages
   *
   * @param _msg NRF24 message
   */
  void OnRosMsg( const drone_meshnetwork_simulation::NRF24ConstPtr& _msg );
  /**
   * @brief get On state
   *
   * @return true on
   * @return false off
   */
  const bool On( );

  /**
   * @brief ROS service function to switch power on and off
   *
   * @param request NOT USED
   * @param response NOT USED
   * @return true Service call succesfull
   * @return false Service call not succesfull
   */
  bool switchPower(
      drone_meshnetwork_simulation::PowerSwitchRequest& request,
      drone_meshnetwork_simulation::PowerSwitchResponse& response );

 private:
  /**
   * @brief Used to queue up and handle ros message
   *
   */
  void QueueThread( );
  /**
   * @brief Publishes information about the Component at 1hz
   *
   */
  void publishDebugInfo( );

 private:
  /// \brief thread for publishing debug information at 1hz
  std::thread NodeDebugInfoThread;
  /// \brief Service to turn the communication on and off
  ros::ServiceServer switchPowerService;
  /// \brief The connected Meshnetwork Interface used for messages
  Communication::Wireless::IMeshNetwork* meshnetworkComponent;
  /// \brief Interface providing debug info about the meshnetwork
  Communication::Wireless::IMeshDebugInfo* debuginfo;
  /// \brief Pointer to the Ros Node of this class
  std::unique_ptr< ros::NodeHandle > rosNode;
  /// \brief rosQueue for handling messages
  ros::CallbackQueue rosQueue;
  /// \brief the name this antenna communicates on
  std::string Node_TopicName;
  /// \brief the name the WirelessSignalSimulator listens to
  const std::string WirelessSignalSimulatorName = "/WirelessSignalSimulator";
  /// \brief Service for requesting all nodes near
  ros::ServiceClient areaScanner;
  /// \brief Service for sending messages using the WirelessSignalSimulator
  ros::ServiceClient publishService;
  /// \brief Service for publishing information about this node to the
  /// WirelessSignalSimulator
  ros::Publisher rosPub;
  /// \brief Service for publishing information for debuging
  ros::Publisher nodeDebugTopic;
  /// \brief Subscriber to recieve NRF24 messages
  ros::Subscriber rosSub;
  /// \brief thread to keep an open line for receiving NRF24 messages
  std::thread rosQueueThread;
  /// \brief The ON/OFF state of the antenna
  bool on = true;
 };
}  // namespace WirelessSimulation
}  // namespace ros
#endif  // VIRTUALNRF24
