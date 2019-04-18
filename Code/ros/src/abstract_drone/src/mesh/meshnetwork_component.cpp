#include "meshnetwork_component.hpp"
#include "abstract_drone/RequestGPS.h"
#include <math.h>

namespace gazebo
{
void MeshnetworkComponent::Load( physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf )
{
 // Store the pointer to the model
 this->model = _parent;
 this->nodeID = 255;   // unkown nodes are number 255
 this->droneID = 666;  // Drones need unique id's to connect the motor

 if ( _sdf->HasElement( "nodeID" ) ) {
  this->nodeID = _sdf->Get< int >( "nodeID" );
 }
 if ( _sdf->HasElement( "DroneID" ) ) {
  this->droneID = _sdf->Get< int >( "DroneID" );
 } else {
  ROS_ERROR(
      "Drone com being used without a droneid, set up using the tag \
    <DroneID> unique <DroneID> \n  Restart ussing droneID's else drone \
    movement will be messy" );
 }
 this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     std::bind( &MeshnetworkComponent::OnUpdate, this ) );
 std::string Node_TopicName;
 std::string WirelessSignalSimulatorName;
 // Check that the velocity element exists, then read the value
 Node_TopicName = "/Node/" + std::to_string( nodeID );
 WirelessSignalSimulatorName = "/postMaster";
 std::string connectedEngine =
     "/Drones/" + std::to_string( this->droneID ) + "/goal";
 std::string DebugInfoName =
     "/Nodes/" + std::to_string( this->droneID ) + "/debug";
 std::string powerSwitch =
     "/Nodes/" + std::to_string( this->droneID ) + "/powerSwitch";
 std::string gps_ServiceName =
     "/Drones/" + std::to_string( this->droneID ) + "/gps";
 // Initialize ros, if it has not already been initialized.
 if ( !ros::isInitialized( ) ) {
  int argc = 0;
  char **argv = NULL;
  ros::init( argc, argv, "node", ros::init_options::NoSigintHandler );
 }
 ROS_INFO( "modelName =%s", Node_TopicName.c_str( ) );
 // Create our ROS node. This acts in a similar manner to
 // the Gazebo node
 this->rosNode.reset( new ros::NodeHandle( "node" ) );

 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::NRF24 >(
         Node_TopicName, 1000,
         boost::bind( &MeshnetworkComponent::OnRosMsg, this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );
 this->rosSub = this->rosNode->subscribe( so );
 this->rosPub = this->rosNode->advertise< abstract_drone::nodeInfo >(
     WirelessSignalSimulatorName, 100 );
 this->nodeDebugTopic =
     this->rosNode->advertise< abstract_drone::NodeDebugInfo >( DebugInfoName,
                                                                100 );
 this->droneEnginePublisher =
     this->rosNode->advertise< abstract_drone::Location >( connectedEngine,
                                                           100 );
 // Setup  area scanner
 this->areaScanner = this->rosNode->serviceClient< abstract_drone::AreaScan >(
     "/SignalSimulator/othersInRange" );
 // Connect to the GPS service
 this->GPSLink = this->rosNode->serviceClient< abstract_drone::RequestGPS >(
     gps_ServiceName );
 // Setup communication
 this->publishService =
     this->rosNode->serviceClient< abstract_drone::WirelessMessage >(
         "/SignalSimulator/message" );
 // Give the node an ON/OFF button
 this->switchPowerService = this->rosNode->advertiseService(
     powerSwitch, &MeshnetworkComponent::switchPower, this );
 // Spin up the queue helper thread.
 this->rosQueueThread =
     std::thread( std::bind( &MeshnetworkComponent::QueueThread, this ) );
 this->heartbeatThread =
     std::thread( std::bind( &MeshnetworkComponent::CheckConnection, this ) );
 this->NodeInfoThread =
     std::thread( std::bind( &MeshnetworkComponent::publishDebugInfo, this ) );
 ROS_WARN( "Loaded MeshnetworkComponent Plugin with parent...%s",
           this->model->GetName( ).c_str( ) );

 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = nodeID;
 nodeinf.sub = Node_TopicName;
 nodeinf.on = true;
 rosPub.publish( nodeinf );
}

bool MeshnetworkComponent::switchPower( std_srvs::TriggerRequest &request,
                                        std_srvs::TriggerResponse &response )
{
 this->on = !this->on;
 if ( !this->on ) lostConnection( );
 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = this->nodeID;
 nodeinf.on = this->on;
 rosPub.publish( nodeinf );
 return true;
}

void MeshnetworkComponent::publishDebugInfo( )
{
 abstract_drone::NodeDebugInfo msg;
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 1 );  // 1hz refresh is enough
  msg.nodeID = this->nodeID;
  msg.ConnectedWithGateway = this->connectedToGateway;
  msg.familySize = this->nodeTable.getTableSize( );
  msg.totalMessages = this->totalMessageSent;
  msg.connectedNodes = this->nodeTable.getAmountOfChildren( );
  msg.prefferedGateWay = this->prefferedGateWay;
  msg.on = this->on;
  msg.hops = this->hopsFromGatewayAway;
  msg.prefLoc = this->lastGoodKnownLocation.getID( );
  nodeDebugTopic.publish( msg );
 }
}

void MeshnetworkComponent::OnRosMsg( const abstract_drone::NRF24ConstPtr &_msg )
{
 ( _msg->forward == this->nodeID ) ? processMessage( _msg )
                                   : forwardMessage( _msg );
 nodeTable.OtherCanCommunicateWithNode( _msg->from, _msg->payload[0] );
 // TODO send ack message back
}

void MeshnetworkComponent::forwardMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 abstract_drone::WirelessMessage WM;
 WM.request.message = *( _msg );
 if ( _msg->payload[1] == Messages::HEARTBEAT ) {
  Messages::HeartbeatMessage msg( _msg->payload.data( ) );
  msg.makeHop( );
  if ( msg.getHops( ) > 255 ) return;

  msg.toPayload( WM.request.message.payload.data( ) );
 }

 uint8_t other = nodeTable.getDirectionToNode( _msg->forward );
 if ( other == 255 ) return;
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 sendMessage( WM );
}

bool MeshnetworkComponent::sendHeartbeat( uint8_t other )
{
 uint8_t towards = nodeTable.getDirectionToNode( other );
 if ( towards == 255 ) return false;
 // create a Message to introduce yourself to others
 Messages::HeartbeatMessage heartbeat( this->nodeID, connectedToGateway,
                                       prefferedGateWay );
 abstract_drone::WirelessMessage WM;

 WM.request.message.from = this->nodeID;
 WM.request.message.to = towards;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 heartbeat.toPayload( WM.request.message.payload.data( ) );
 return sendMessage( WM );
}

void MeshnetworkComponent::sendGoalToDrone( const uint8_t ID,
                                            const float longitude,
                                            const float latitude,
                                            const uint16_t height )
{
 Messages::GoToLocationMessage GTLmsg( this->nodeID, longitude, latitude,
                                       height );

 abstract_drone::WirelessMessage WM;

 uint8_t other = nodeTable.getDirectionToNode( ID );
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = ID;
 WM.request.message.ack = 0;
 GTLmsg.toPayload( WM.request.message.payload.data( ) );
 sendMessage( WM );
}

void MeshnetworkComponent::processMissing(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Messages::MissingMessage msg( _msg->payload.data( ) );
 if ( nodeTable.OtherCantCommunicateWithNode( msg.getID( ),
                                              msg.getDeceased( ) ) > 0 )
  informAboutMissingChild( msg.getID( ), msg.getDeceased( ) );
}

void MeshnetworkComponent::informAboutMissingChild( uint8_t parent,
                                                    uint8_t child )
{
 Messages::MissingMessage deceased( this->nodeID, child );
 abstract_drone::WirelessMessage WM;

 for ( auto &chi1d : nodeTable.getSetOfChildren( ) ) {
  if ( parent == chi1d ) continue;
  uint8_t other = nodeTable.getDirectionToNode( chi1d );
  if ( other == 255 ) continue;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = other;
  WM.request.message.forward = chi1d;
  WM.request.message.ack = 0;
  deceased.toPayload( WM.request.message.payload.data( ) );

  sendMessage( WM );
 }
}

void MeshnetworkComponent::searchOtherNodesInRange( )
{
 abstract_drone::AreaScan scanMsg;
 scanMsg.request.id = this->nodeID;
 if ( areaScanner.call( scanMsg ) ) {
  if ( scanMsg.response.near.empty( ) ) {
   ROS_WARN( "Node %d found none", this->nodeID );
   return;
  }

  for ( const uint8_t &n : scanMsg.response.near ) {
   IntroduceNode( n );
  }
 } else {
  ROS_ERROR( "Failed to call service othersInRange" );
 }
}

void MeshnetworkComponent::requestLocation( const uint8_t other )
{
 Messages::Message msg( this->nodeID, Messages::REQUESTLOCATION );
 abstract_drone::WirelessMessage WM;
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 msg.toPayload( WM.request.message.payload.data( ) );

 sendMessage( WM );
}

void MeshnetworkComponent::processRequestLocation(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Messages::Message msg( _msg->payload.data( ) );
 sendLocation( msg.getID( ) );
}
void MeshnetworkComponent::sendLocation( const uint8_t other )
{
 abstract_drone::RequestGPS GPS;
 GPS.request.ID = this->nodeID;
 if ( GPSLink.call( GPS ) ) {
  Messages::LocationMessage msg( this->nodeID, GPS.response.latitude,
                                 GPS.response.longitude, GPS.response.height,
                                 0 );
  abstract_drone::WirelessMessage WM;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = other;
  WM.request.message.forward = other;
  WM.request.message.ack = 0;
  msg.toPayload( WM.request.message.payload.data( ) );
  sendMessage( WM );
 }
}
void MeshnetworkComponent::processLocation(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Messages::LocationMessage msg( _msg->payload.data( ) );
 if ( msg.getID( ) == prefferedGateWay ) {
  prefferedGateWayLocation = msg;
  knowPrefferedGatewayLocation = true;
 } else {
  lastGoodKnownLocation = msg;
 }
}

float MeshnetworkComponent::distanceBetweenMeAndLocation(
    const Messages::LocationMessage &A )
{
 abstract_drone::RequestGPS GPS;

 GPS.request.ID = this->nodeID;
 if ( GPSLink.call( GPS ) ) {
  float a = A.longitude - GPS.response.longitude;
  float b = A.latitude - GPS.response.latitude;
  float c = A.height - GPS.response.height;
  return std::sqrt( std::pow( a, 2 ) + std::pow( b, 2 ) + std::pow( c, 2 ) );
 } else {
  return 0;
 }
}

void MeshnetworkComponent::IntroduceNode( const uint8_t other )
{
 // create a Message to introduce yourself to others
 Messages::IntroduceMessage introduce( this->nodeID, this->hopsFromGatewayAway,
                                       this->connectedToGateway );
 abstract_drone::WirelessMessage WM;

 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 introduce.toPayload( WM.request.message.payload.data( ) );

 sendMessage( WM );
}

bool MeshnetworkComponent::sendMessage(
    abstract_drone::WirelessMessage &message )
{
 if ( publishService.call( message ) ) {
  if ( !message.response.succes ) {
   if ( nodeTable.cantCommunicateWithNode( message.request.message.to ) > 0 )
    informAboutMissingChild( this->nodeID, message.request.message.to );
   return false;
  } else {
   nodeTable.canCommunicateWithNode( message.request.message.to );
   ++totalMessageSent;
   return true;
  }
 } else {
  ROS_ERROR( "SIGNAL SIMULATOR NOT AVAILABLE" );
  return false;
 }
}

void MeshnetworkComponent::reassignID( uint8_t ID )
{
 std::string Node_TopicName = "/Node/" + std::to_string( ID );
 ROS_WARN( "Reassign to ID %d", ID );
 this->rosNode.reset( new ros::NodeHandle( "node" ) );
 this->nodeID = ID;
 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::NRF24 >(
         Node_TopicName, 1000,
         boost::bind( &MeshnetworkComponent::OnRosMsg, this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );
 this->rosSub = this->rosNode->subscribe( so );
}

void MeshnetworkComponent::sendGoalToEngine(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Messages::LocationMessage locmsg( _msg->payload.data( ) );
 abstract_drone::Location msg;
 msg.latitude = locmsg.latitude;
 msg.longitude = locmsg.longitude;
 msg.height = locmsg.height;

 droneEnginePublisher.publish( msg );
}

void MeshnetworkComponent::sendGoalToEngine(
    const Messages::LocationMessage &_msg )
{
 abstract_drone::Location msg;
 msg.latitude = _msg.latitude;
 msg.longitude = _msg.longitude;
 msg.height = _msg.height;

 droneEnginePublisher.publish( msg );
}

/// \brief ROS helper function that processes messages
void MeshnetworkComponent::QueueThread( )
{
 static const double timeout = 0.01;
 while ( this->rosNode->ok( ) ) {
  this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
 }
}
}  // namespace gazebo
