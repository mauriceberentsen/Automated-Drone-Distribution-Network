/**
 * @file Node.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for Node class
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef NODEHPP
#define NODEHPP

#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "abstract_drone/NRF24.h"

using namespace ignition::math;
namespace ros
{
namespace WirelessSimulation
{
 class Node
 {
 public:
  /**
   * @brief Construct a new Node used for administration of the the
   * WirelessSignalSimulator. Advertises the topic the node is listing to
   *
   * @param _position The current position of the Node
   * @param _nodeHandle The Nodehandler of the WirelessSignalSimulator
   * @param _subtopicname The name the of the topic
   */
  Node( const Vector3< float >& _position,
        const std::shared_ptr< ros::NodeHandle >& _nodeHandle,
        const std::string& _subtopicname );
  ~Node( );
  /**
   * @brief Used for sending NRF24 messages to this node
   *
   * @param msg THe NRF24 message
   */
  void recieveMessage( const abstract_drone::NRF24& msg ) const;

  const Vector3< float >& getPosition( ) const;
  void setPosition( const Vector3< float >& position );
  const bool getOn( ) const;
  void setOn( const bool on );

 private:
  /// \brief ON/OFF state for the node. If off its not allowed to recieve
  /// any message
  bool on;
  /// \brief ROS Publisher to the topic of this node
  ros::Publisher connectedNRF;
  /// \brief Pointer towards the nodehandler to be ale to publish
  std::shared_ptr< ros::NodeHandle > nodeHandle;
  /// \brief the current position of this drone
  Vector3< float > position;
  /// \brief the name of the topic this node is publing towards.
  const std::string subtopicname;
 };
}  // namespace WirelessSimulation
}  // namespace ros
#endif  // NODEHPP