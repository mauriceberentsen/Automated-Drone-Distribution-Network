#include <map>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "abstract_drone/nodeInfo.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/AreaScan.h"

#include "node.hpp"
#include <iostream>

namespace gazebo
{
class WirelessSignalSimulator : public WorldPlugin
{
private:
 std::map< uint8_t, wireless::Node * > Network;

private:
 bool send_message( abstract_drone::WirelessMessage::Request &req,
                    abstract_drone::WirelessMessage::Response &res )
 {
  auto from = Network.find( req.from );

  auto to = Network.find( req.to );
  if ( from != Network.end( ) &&
       to != Network.end( ) )  // Already exists in the Network UpdateLocation
  {
   ROS_INFO( "compare nodes %d ==> %d", req.from, req.to );
   float distance =
       from->second->getPosition( ).Distance( to->second->getPosition( ) );
   // using pythgoras in the function Vector3 to get the distance between to
   // nodes
   ROS_INFO( "distance = %f", distance );
   if ( distance < maxComDistance ) {
    to->second->recieveMessage( req.message );
    res.succes = true;
   } else {
    res.succes = false;
   }
  } else {
   if ( from == Network.end( ) ) {
    ROS_ERROR( "Sender %d doesnt exist in Network", ( int )req.from );
   }
   if ( to == Network.end( ) ) {
    ROS_ERROR( "reciever %d doesnt exist in Network", ( int )req.to );
   }
   res.succes = false;
  }
 }

private:
 bool getNodesInRange( abstract_drone::AreaScan::Request &req,
                       abstract_drone::AreaScan::Response &res )
 {
  auto from = Network.find( req.id );
  res.near.clear( );
  if ( from != Network.end( ) ) {
   for ( std::pair< uint8_t, wireless::Node * > other : Network ) {
    if ( other.first == from->first ) continue;
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

public:
 void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf )
 {
  std::string Node_TopicName = "/postMaster";
  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "WirelessSignalSimulator",
              ros::init_options::NoSigintHandler );
  }

  if ( _sdf->HasElement( "CommunicationDistance" ) ) {
   this->maxComDistance = _sdf->Get< float >( "CommunicationDistance" );
  }

  // CreateROS node.
  this->rosNode.reset( new ros::NodeHandle( "SignalSimulator" ) );

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create< abstract_drone::nodeInfo >(
          Node_TopicName, 1,
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

public:
 void OnRosMsg( const abstract_drone::nodeInfoConstPtr &_msg )
 {
  auto it = Network.find( _msg->nodeID );
  if ( it != Network.end( ) )  // Already exists in the Network UpdateLocation
  {
   float x = _msg->position[0];
   float y = _msg->position[1];
   float z = _msg->position[2];
   Vector3< float > vec( x, y, z );
   it->second->setPosition( vec );
  }

  else  // new node addNodeToNetwork
  {
   if ( _msg->sub == "" ) {
    ROS_WARN( "NO TOPIC NAME" );
    return;
   }  // We dont want to add a node with no topic name;
   float x = _msg->position[0];
   float y = _msg->position[1];
   float z = _msg->position[2];
   Vector3< float > vec( x, y, z );
   addNodeToNetwork( _msg->nodeID, vec, _msg->sub );
  }
 }

 /// \brief ROS helper function that processes messages
private:
 void QueueThread( )
 {
  static const double timeout = 0.01;
  while ( this->rosNode->ok( ) ) {
   this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
  }
 }

private:
 void addNodeToNetwork( uint8_t id, Vector3< float > &pos,
                        std::string pubtopicname )
 {
  this->Network.insert(
      std::make_pair( id, new wireless::Node( pos, rosNode, pubtopicname ) ) );
  ROS_INFO( "added node [%d] on position [%f] [%f] [%f] with name %s ", id,
            pos.X( ), pos.Y( ), pos.Z( ), pubtopicname.c_str( ) );
 }

private:
 float maxComDistance = 30.0;  // meters

private:
 ros::ServiceServer service;
 ros::ServiceServer messageservice;

 /// \brief A node use for ROS transport
private:
 std::shared_ptr< ros::NodeHandle > rosNode;
 /// \brief A ROS subscriber
private:
 ros::Subscriber rosSub;
 /// \brief A ROS callbackqueue that helps process messages
private:
 ros::CallbackQueue rosQueue;
 /// \brief A thread the keeps running the rosQueue
private:
 std::thread rosQueueThread;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( WirelessSignalSimulator )
}  // namespace gazebo