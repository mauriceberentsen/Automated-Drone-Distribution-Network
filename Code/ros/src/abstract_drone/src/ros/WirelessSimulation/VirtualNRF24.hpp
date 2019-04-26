#ifndef VIRTUALNRF24
#define VIRTUALNRF24

#include <thread>

#include "../../Communication/Wireless/IWirelessCommunication.hpp"
// ros
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ros/callback_queue.h"
// ros generated messages
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/NRF24.h"
#include "abstract_drone/NodeDebugInfo.h"
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
  explicit VirtualNRF24( Communication::Meshnetwork::MeshnetworkComponent& MC );
  ~VirtualNRF24( );
  void StartAntenna( );
  void StopAntenna( );
  bool SendMessageTo( const uint8_t* msg );
  void BroadcastMessage( const uint8_t* msg );
  void DebugingMode( const bool on = true );
  void OnRosMsg( const abstract_drone::NRF24ConstPtr& _msg );
  const bool On( );

 private:
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
  bool switchPower( std_srvs::TriggerRequest& request,
                    std_srvs::TriggerResponse& response );
  /**
   * @brief Publishes information about the Component at 1hz
   *
   */
  void publishDebugInfo( );

  /// \brief thread for publishing debug information at 1hz
  std::thread NodeDebugInfoThread;
  /// \brief Service to turn the communication on and off
  ros::ServiceServer switchPowerService;
  /// \brief The connected MeshnetworkComponent used for WirelessSimulator
  Communication::Meshnetwork::MeshnetworkComponent& meshnetworkComponent;
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
