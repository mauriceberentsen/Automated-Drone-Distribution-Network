/**
 * @file WirelessSignalSimulator.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for the WirelessSignalSimulator
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ros/subscribe_options.h"

#include "WirelessSignalSimulator.hpp"

namespace ros
{
namespace WirelessSimulation
{
 WirelessSignalSimulator::WirelessSignalSimulator( const float comDistance )
     : maxComDistance( comDistance )
 {
  std::string Node_TopicName = "/WirelessSignalSimulator";
  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "WirelessSignalSimulator",
              ros::init_options::NoSigintHandler );
  }

  // CreateROS node.
  this->rosNode.reset( new ros::NodeHandle( "SignalSimulator" ) );

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create< abstract_drone::DroneInfo >(
          Node_TopicName, 1000,
          boost::bind( &WirelessSignalSimulator::OnRosMsg, this, _1 ),
          ros::VoidPtr( ), &this->rosQueue );
  this->rosSub = this->rosNode->subscribe( so );
  this->messageservice = this->rosNode->advertiseService(
      "message", &WirelessSignalSimulator::send_message, this );
  this->service = this->rosNode->advertiseService(
      "othersInRange", &WirelessSignalSimulator::getNodesInRange, this );

  this->rosQueueThread =
      std::thread( std::bind( &WirelessSignalSimulator::QueueThread, this ) );
 }

 bool WirelessSignalSimulator::send_message(
     abstract_drone::WirelessMessage::Request &req,
     abstract_drone::WirelessMessage::Response &res )
 {
  auto to = Network.find( req.to );

  auto from = Network.find( req.from );

  if ( from != Network.end( ) &&
       to != Network.end( ) )  // Already exists in the Network UpdateLocation
  {
   if ( !from->second->getOn( ) || !to->second->getOn( ) ) {
    res.succes = false;

   } else {
    float distance =
        from->second->getPosition( ).Distance( to->second->getPosition( ) );
    // using pythgoras in the function Vector3 to get the distance between to
    // nodes
    if ( distance < maxComDistance ) {
     to->second->recieveMessage( req.message );
     res.succes = true;
    } else {
     res.succes = false;
    }
   }
  } else {
   if ( from == Network.end( ) ) {
    ROS_ERROR( "Sender %d doesnt exist in Network", ( int )req.from );
    res.succes = false;
   }
   if ( to == Network.end( ) ) {
    ROS_ERROR( "reciever %d doesnt exist in Network", ( int )req.to );
    res.succes = false;
   }
   res.succes = false;
  }
  return true;
 }

 bool WirelessSignalSimulator::getNodesInRange(
     abstract_drone::AreaScan::Request &req,
     abstract_drone::AreaScan::Response &res )
 {
  auto from = Network.find( req.id );
  res.near.clear( );
  if ( from != Network.end( ) ) {
   for ( std::pair< uint8_t, Node * > other : Network ) {
    if ( other.first == from->first ) continue;  // not interrested in ourself
    if ( !other.second->getOn( ) ) continue;
    float distance =
        from->second->getPosition( ).Distance( other.second->getPosition( ) );
    // using pythgoras in the function Vector3 to get the distance between to
    // nodes
    if ( distance < maxComDistance ) {
     res.near.push_back( ( int )other.first );
    }
   }
   return true;
  } else {
   ROS_ERROR( "requester %d doesnt exist in Network", ( int )req.id );
   return false;
  }
 }

 void WirelessSignalSimulator::OnRosMsg(
     const abstract_drone::DroneInfoConstPtr &_msg )
 {
  auto it = Network.find( _msg->nodeID );
  if ( it !=
       Network.end( ) )  // Already exists in the Network so update location
  {
   if ( it->second->getOn( ) == _msg->on ) {
    float x = _msg->position[0];
    float y = _msg->position[1];
    float z = _msg->position[2];
    Vector3< float > vec( x, y, z );
    it->second->setPosition( vec );
   } else {
    it->second->setOn( _msg->on );
   }
  } else  // new node addNodeToNetwork
  {
   if ( _msg->sub == "" ) {
    ROS_ERROR( "NO TOPIC NAME" );
    return;
   }  // We dont want to add a node with no topic name;
   float x = _msg->position[0];
   float y = _msg->position[1];
   float z = _msg->position[2];
   Vector3< float > vec( x, y, z );
   addNodeToNetwork( _msg->nodeID, vec, _msg->sub );
  }
 }

 void WirelessSignalSimulator::QueueThread( )
 {
  static const double timeout = 0.01;
  while ( this->rosNode->ok( ) ) {
   this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
  }
 }

 void WirelessSignalSimulator::addNodeToNetwork( uint8_t id,
                                                 Vector3< float > &pos,
                                                 std::string pubtopicname )
 {
  this->Network.insert(
      std::make_pair( id, new Node( pos, rosNode, pubtopicname ) ) );
  ROS_INFO( "added node [%d] on position [%f] [%f] [%f] with name %s ", id,
            pos.X( ), pos.Y( ), pos.Z( ), pubtopicname.c_str( ) );
 }

}  // namespace WirelessSimulation
// Register this plugin with the simulator
}  // namespace ros