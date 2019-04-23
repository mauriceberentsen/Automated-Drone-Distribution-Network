/**
 * @file wireless_signal_simulator.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef WIRELESSSIGNALSIMULATOR
#define WIRELESSSIGNALSIMULATOR

#include <map>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "abstract_drone/nodeInfo.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/AreaScan.h"

#include "node.hpp"

namespace gazebo
{
namespace Wireless
{
 class WirelessSignalSimulator : public WorldPlugin
 {
 public:
  /**
   * @brief A servicecall used for sending messages from one node to another
   * conform basic rules of wireless communication
   *
   * @param req.message NRF24 message that the sender wants to send
   * @param res.succes boolean describing the succes of sending the message
   * @return true servicecall succesfull
   * @return false servicecall failed
   */
  bool send_message( abstract_drone::WirelessMessage::Request &req,
                     abstract_drone::WirelessMessage::Response &res );
  /**
   * @brief Since we can't just broadcast a signal towards all directions this
   * servicecall gives all nodes in range of the requested NodeID
   *
   * @param req.id The ID of the requesting node
   * @param res.near std::Vector holding a nodes near
   * @return true servicecall succesfull
   * @return false servicecall failed
   */
  bool getNodesInRange( abstract_drone::AreaScan::Request &req,
                        abstract_drone::AreaScan::Response &res );

 protected:
 private:
  /**
   * @brief This function is called at loading the plugin.
   *        - Start ros if not already running
   *        - Read SDF for the CommunicationDistance tag
   *        - Setup ROS Subscribers and Services
   *        - Spin up thread for ros handling
   *
   * @param _parent Pointer to this plugin
   * @param _sdf pointer to this plugin SDF
   */
  void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf );
  /**
   * @brief Handles Ros messages recieved of the type nodeInfo
   *        - Used for administration of the following
   *            -# The whereabouts of the nodes
   *            -# The topic they are subscribed to for sending messages
   *            -# The On/Off state of the node.
   *
   * @param _msg ROS message holding Node information
   */
  void OnRosMsg( const abstract_drone::nodeInfoConstPtr &_msg );

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
  float maxComDistance = 30.0;  // meters

  ros::ServiceServer service;
  ros::ServiceServer messageservice;

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
}  // namespace Wireless
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( Wireless::WirelessSignalSimulator )
}  // namespace gazebo
#endif  // WIRELESSSIGNALSIMULATOR