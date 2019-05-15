/**
 * @file Node.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for Node
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "Node.hpp"
namespace ros
{
namespace WirelessSimulation
{
 Node::Node( const Vector3< float >& _position,
             const std::shared_ptr< ros::NodeHandle >& _nodeHandle,
             const std::string& _subtopicname )
     : position( _position )
     , nodeHandle( _nodeHandle )
     , subtopicname( _subtopicname )
     , on( true )
 {
  connectedNRF = nodeHandle->advertise< drone_meshnetwork_simulation::NRF24 >(
      subtopicname, 100, true );
 }

 Node::~Node( )
 {
 }

 void Node::recieveMessage(
     const drone_meshnetwork_simulation::NRF24& msg ) const
 {
  connectedNRF.publish( msg );
 }

 const Vector3< float >& Node::getPosition( ) const
 {
  return this->position;
 }

 void Node::setPosition( const Vector3< float >& position )
 {
  this->position = position;
 }

 const bool Node::getOn( ) const
 {
  return this->on;
 }
 void Node::setOn( const bool on )
 {
  this->on = on;
 }

}  // namespace WirelessSimulation
}  // namespace ros