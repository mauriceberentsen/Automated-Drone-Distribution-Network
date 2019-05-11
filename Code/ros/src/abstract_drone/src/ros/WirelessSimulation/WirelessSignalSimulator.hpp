/**
 * @file WirelessSignalSimulator.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the the WirelessSignalSimulator
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef WIRELESSSIGNALSIMULATOR
#define WIRELESSSIGNALSIMULATOR

#include <map>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "abstract_drone/DroneInfo.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/AreaScan.h"

#include "Node.hpp"

namespace ros
{
namespace WirelessSimulation
{
 class WirelessSignalSimulator
 {
 public:
  /**
   * @brief Construct a new Wireless Signal Simulator object
   *         calls start on load
   *
   *@param comDistance maximum communication distance
   */
  explicit WirelessSignalSimulator( const float comDistance );
  /**
   * @brief Destroy the Wireless Signal Simulator object
   *
   */
  ~WirelessSignalSimulator( );
  /**
   * @brief A servicecall used for sending messages from one node to another
   * conform basic rules of wireless communication
   *
   * @param req holds the NRF24 message that the sender wants to send
   * @param res for the succes boolean describing the succes of sending the
   * message
   * @return true servicecall succesfull
   * @return false servicecall failed
   */
  bool send_message( abstract_drone::WirelessMessage::Request &req,
                     abstract_drone::WirelessMessage::Response &res );
  /**
   * @brief Since we can't just broadcast a signal towards all directions this
   * servicecall gives all nodes in range of the requested NodeID
   *
   * @param req Holds the id The ID of the requesting node
   * @param res returns the near std::Vector holding all nodes near
   * @return true servicecall succesfull
   * @return false servicecall failed
   */
  bool getNodesInRange( abstract_drone::AreaScan::Request &req,
                        abstract_drone::AreaScan::Response &res );

  /**
   * @brief This function is called at loading the plugin.
   *        - Start ros if not already running
   *        - Setup ROS Subscribers and Services
   *        - Spin up thread for ros handling
   *
   *@param comDistance maximum communication distance
   */
  void Start( );
  /**
   * @brief Stops the WirelessSignalSimulator
   *
   */
  void Stop( );

 protected:
 private:
  /**
   * @brief Handles Ros messages recieved of the type DroneInfo
   *        - Used for administration of the following
   *            -# The whereabouts of the nodes
   *            -# The topic they are subscribed to for sending messages
   *            -# The On/Off state of the node.
   *
   * @param _msg ROS message holding Node information
   */
  void OnRosMsg( const abstract_drone::DroneInfoConstPtr &_msg );

  /**
   * @brief  ROS helper function that processes messages
   *
   */
  void QueueThread( );
  /**
   * @brief Add a node to the network using a pair so the drone can easily be
   * found
   *
   * @param id The ID of the node.
   * @param pos The Position of the node.
   * @param pubtopicname The name of the topic the node is subscribed to.
   */
  void addNodeToNetwork( uint8_t id, Vector3< float > &pos,
                         std::string pubtopicname );

 public:
 protected:
 private:
  /// \brief default CommunicationDistance
  float maxComDistance = 30.0;  // meters
  /// \brief Service to find all nodes near
  ros::ServiceServer service;
  /// \brief Service used sending messages
  ros::ServiceServer messageservice;
  /// \brief List of all known nodes
  std::map< uint8_t, Node * > Network;
  /// \brief A node use for ROS transport
  std::shared_ptr< ros::NodeHandle > rosNode;
  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;
 };
}  // namespace WirelessSimulation

}  // namespace ros
#endif  // WIRELESSSIGNALSIMULATOR