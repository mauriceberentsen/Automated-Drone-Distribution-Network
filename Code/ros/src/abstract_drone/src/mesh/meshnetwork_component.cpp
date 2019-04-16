#include "meshnetwork_component.hpp"
#include "abstract_drone/RequestGPS.h"
#include <math.h>

namespace gazebo
{
void MeshnetworkCommponent::Load( physics::ModelPtr _parent,
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
     std::bind( &MeshnetworkCommponent::OnUpdate, this ) );
 std::string Node_TopicName;
 std::string WirelessSignalSimulatorName;
 std::string ModelName = this->model->GetName( );
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
         boost::bind( &MeshnetworkCommponent::OnRosMsg, this, _1 ),
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
     powerSwitch, &MeshnetworkCommponent::switchPower, this );
 // Spin up the queue helper thread.
 this->rosQueueThread =
     std::thread( std::bind( &MeshnetworkCommponent::QueueThread, this ) );
 this->heartbeatThread =
     std::thread( std::bind( &MeshnetworkCommponent::CheckConnection, this ) );
 this->NodeInfoThread =
     std::thread( std::bind( &MeshnetworkCommponent::publishDebugInfo, this ) );
 ROS_WARN( "Loaded MeshnetworkCommponent Plugin with parent...%s",
           this->model->GetName( ).c_str( ) );

 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = nodeID;
 nodeinf.sub = Node_TopicName;
 nodeinf.on = true;
 rosPub.publish( nodeinf );
}

bool MeshnetworkCommponent::switchPower( std_srvs::TriggerRequest &request,
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

void MeshnetworkCommponent::publishDebugInfo( )
{
 abstract_drone::NodeDebugInfo msg;
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 1 );  // 1hz refresh is enough
  msg.nodeID = this->nodeID;
  msg.ConnectedWithGateway = this->connectedToGateway;
  msg.familySize = this->nodeTable.familysize( );
  msg.totalMessages = this->totalMessageSent;
  msg.connectedNodes = this->nodeTable.getFamily( ).size( );
  msg.prefferedGateWay = this->prefferedGateWay;
  msg.on = this->on;
  msg.hops = this->hopsFromGatewayAway;
  msg.prefLoc = this->lastGoodKnownLocation.getID( );
  nodeDebugTopic.publish( msg );
 }
}

void MeshnetworkCommponent::OnRosMsg(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 ( _msg->forward == this->nodeID ) ? processMessage( _msg )
                                   : forwardMessage( _msg );
 nodeTable.proofOfAvailability( _msg->from, _msg->payload[0] );
 // TODO send ack message back
}

void MeshnetworkCommponent::forwardMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 abstract_drone::WirelessMessage WM;
 WM.request.message = *( _msg );
 if ( _msg->payload[1] == HEARTBEAT ) {
  HeartbeatMessage msg( _msg->payload.data( ) );
  msg.makeHop( );
  msg.toPayload( WM.request.message.payload.data( ) );
 }

 uint8_t other = nodeTable.getDirectionToNode( _msg->forward );
 if ( other == 255 ) return;
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 sendMessage( WM );
}

bool MeshnetworkCommponent::sendHeartbeat( uint8_t other )
{
 uint8_t towards = nodeTable.getDirectionToNode( other );
 if ( towards == 255 ) return false;
 // create a Message to introduce yourself to others
 HeartbeatMessage heartbeat( this->nodeID, connectedToGateway,
                             prefferedGateWay );
 abstract_drone::WirelessMessage WM;

 WM.request.message.from = this->nodeID;
 WM.request.message.to = towards;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 heartbeat.toPayload( WM.request.message.payload.data( ) );
 return sendMessage( WM );
}

void MeshnetworkCommponent::sendGoalToDrone( const uint8_t ID,
                                             const float longitude,
                                             const float latitude,
                                             const uint16_t height )
{
 GoToLocationMessage GTLmsg( this->nodeID, longitude, latitude, height );

 abstract_drone::WirelessMessage WM;

 uint8_t other = nodeTable.getDirectionToNode( ID );
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = ID;
 WM.request.message.ack = 0;
 GTLmsg.toPayload( WM.request.message.payload.data( ) );
 sendMessage( WM );
}

void MeshnetworkCommponent::processDeceased(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 DeceasedMessage msg( _msg->payload.data( ) );
 if ( nodeTable.proofOfMissing( msg.getID( ), msg.getDeceased( ) ) > 0 )
  informAboutMissingChild( msg.getID( ), msg.getDeceased( ) );
}

void MeshnetworkCommponent::informAboutMissingChild( uint8_t parent,
                                                     uint8_t child )
{
 DeceasedMessage deceased( this->nodeID, child );
 abstract_drone::WirelessMessage WM;

 for ( auto &child : nodeTable.getFamily( ) ) {
  if ( parent == child.first ) continue;
  uint8_t other = nodeTable.getDirectionToNode( child.first );
  if ( other == 255 ) continue;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = other;
  WM.request.message.forward = child.first;
  WM.request.message.ack = 0;
  deceased.toPayload( WM.request.message.payload.data( ) );

  sendMessage( WM );
 }
}

void MeshnetworkCommponent::searchOtherNodesInRange( )
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

void MeshnetworkCommponent::requestLocation( const uint8_t other )
{
 Message msg( this->nodeID, REQUESTLOCATION );
 abstract_drone::WirelessMessage WM;
 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 msg.toPayload( WM.request.message.payload.data( ) );

 sendMessage( WM );
}

void MeshnetworkCommponent::processRequestLocation(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 Message msg( _msg->payload.data( ) );
 sendLocation( msg.getID( ) );
}
void MeshnetworkCommponent::sendLocation( const uint8_t other )
{
 abstract_drone::RequestGPS GPS;
 GPS.request.ID = this->nodeID;
 if ( GPSLink.call( GPS ) ) {
  LocationMessage msg( this->nodeID, GPS.response.latitude,
                       GPS.response.longitude, GPS.response.height, 0 );
  abstract_drone::WirelessMessage WM;
  WM.request.message.from = this->nodeID;
  WM.request.message.to = other;
  WM.request.message.forward = other;
  WM.request.message.ack = 0;
  msg.toPayload( WM.request.message.payload.data( ) );
  sendMessage( WM );
 }
}
void MeshnetworkCommponent::processLocation(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 LocationMessage msg( _msg->payload.data( ) );
 if ( msg.getID( ) == prefferedGateWay ) {
  prefferedGateWayLocation = msg;
  knowPrefferedGatewayLocation = true;
 } else {
  lastGoodKnownLocation = msg;
 }
}

float MeshnetworkCommponent::distanceBetweenMeAndLocation(
    const LocationMessage &A )
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

void MeshnetworkCommponent::IntroduceNode( const uint8_t other )
{
 // create a Message to introduce yourself to others
 IntroduceMessage introduce( this->nodeID, this->hopsFromGatewayAway,
                             this->connectedToGateway );
 abstract_drone::WirelessMessage WM;

 WM.request.message.from = this->nodeID;
 WM.request.message.to = other;
 WM.request.message.forward = other;
 WM.request.message.ack = 0;
 introduce.toPayload( WM.request.message.payload.data( ) );

 sendMessage( WM );
}

bool MeshnetworkCommponent::sendMessage(
    abstract_drone::WirelessMessage &message )
{
 if ( publishService.call( message ) ) {
  if ( !message.response.succes ) {
   if ( nodeTable.proofOfMissing( message.request.message.to,
                                  message.request.message.to ) > 0 )
    informAboutMissingChild( this->nodeID, message.request.message.to );
   return false;
  } else {
   nodeTable.proofOfAvailability( message.request.message.to,
                                  message.request.message.to );
   ++totalMessageSent;
   return true;
  }
 } else {
  ROS_ERROR( "SIGNAL SIMULATOR NOT AVAILABLE" );
  return false;
 }
}

void MeshnetworkCommponent::reassignID( uint8_t ID )
{
 std::string Node_TopicName = "/Node/" + std::to_string( ID );
 ROS_WARN( "Reassign to ID %d", ID );
 this->rosNode.reset( new ros::NodeHandle( "node" ) );
 this->nodeID = ID;
 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::NRF24 >(
         Node_TopicName, 1000,
         boost::bind( &MeshnetworkCommponent::OnRosMsg, this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );
 this->rosSub = this->rosNode->subscribe( so );
}

void MeshnetworkCommponent::sendGoalToEngine(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 LocationMessage locmsg( _msg->payload.data( ) );
 abstract_drone::Location msg;
 msg.latitude = locmsg.latitude;
 msg.longitude = locmsg.longitude;
 msg.height = locmsg.height;

 droneEnginePublisher.publish( msg );
}

void MeshnetworkCommponent::sendGoalToEngine( const LocationMessage &_msg )
{
 abstract_drone::Location msg;
 msg.latitude = _msg.latitude;
 msg.longitude = _msg.longitude;
 msg.height = _msg.height;

 droneEnginePublisher.publish( msg );
}

/// \brief ROS helper function that processes messages
void MeshnetworkCommponent::QueueThread( )
{
 static const double timeout = 0.01;
 while ( this->rosNode->ok( ) ) {
  this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
 }
}

}  // namespace gazebo