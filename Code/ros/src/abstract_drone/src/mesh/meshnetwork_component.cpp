#include "meshnetwork_component.hpp"

namespace gazebo
{
void MeshnetworkCommponent::Load( physics::ModelPtr _parent,
                                  sdf::ElementPtr _sdf )
{
 // Store the pointer to the model
 this->model = _parent;
 this->NodeID = 255;   // unkown nodes are number 255
 this->droneID = 666;  // Drones need unique id's to connect the motor

 if ( _sdf->HasElement( "NodeID" ) ) {
  this->NodeID = _sdf->Get< int >( "NodeID" );
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
 Node_TopicName = "/Node/" + std::to_string( NodeID );
 WirelessSignalSimulatorName = "/postMaster";
 std::string connectedEngine =
     "/Drones/" + std::to_string( this->droneID ) + "/goal";
 std::string DebugInfoName =
     "/Nodes/" + std::to_string( this->droneID ) + "/debug";
 std::string powerSwitch =
     "/Nodes/" + std::to_string( this->droneID ) + "/powerSwitch";

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

 // Plannar Pose
 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::NRF24 >(
         Node_TopicName, 1000,
         boost::bind( &MeshnetworkCommponent::OnRosMsg, this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );
 this->rosSub = this->rosNode->subscribe( so );
 this->rosPub = this->rosNode->advertise< abstract_drone::nodeInfo >(
     WirelessSignalSimulatorName, 100 );
 this->NodeDebugTopic =
     this->rosNode->advertise< abstract_drone::NodeDebugInfo >( DebugInfoName,
                                                                100 );
 this->droneEnginePublisher =
     this->rosNode->advertise< abstract_drone::Location >( connectedEngine,
                                                           100 );
 this->areaScanner = this->rosNode->serviceClient< abstract_drone::AreaScan >(
     "/SignalSimulator/othersInRange" );
 this->publishService =
     this->rosNode->serviceClient< abstract_drone::WirelessMessage >(
         "/SignalSimulator/message" );
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
 nodeinf.nodeID = NodeID;
 nodeinf.sub = Node_TopicName;
 nodeinf.on = true;
 rosPub.publish( nodeinf );
}

bool MeshnetworkCommponent::switchPower( std_srvs::TriggerRequest &request,
                                         std_srvs::TriggerResponse &response )
{
 this->on = !this->on;
 if(!this->on)lostConnection( );
 abstract_drone::nodeInfo nodeinf;
 nodeinf.nodeID = this->NodeID;
 nodeinf.on = this->on;
 rosPub.publish( nodeinf );
 ROS_INFO( "SWITCH POWER OF %s to %d", this->model->GetName( ).c_str(), this->on );
 return true;
}

void MeshnetworkCommponent::publishDebugInfo( )
{
 abstract_drone::NodeDebugInfo msg;
 while ( this->rosNode->ok( ) ) {
  common::Time::Sleep( 1 );  // 1hz refresh is enough
  msg.nodeID = this->NodeID;
  msg.ConnectedWithGateway = this->connectedToGateway;
  msg.familySize = this->NodeTable.familysize( );
  msg.totalMessages = this->totalMessageSent;
  msg.connectedNodes = this->NodeTable.getFamily( ).size( );
  msg.prefferedGateWay = this->prefferedGateWay;
  msg.on = this->on;
  NodeDebugTopic.publish( msg );
 }
}

void MeshnetworkCommponent::OnRosMsg(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 ( _msg->forward == this->NodeID ) ? processMessage( _msg )
                                   : forwardMessage( _msg );
 NodeTable.proofOfLive( _msg->from, _msg->payload[0] );
 // TODO send ack message back
}

void MeshnetworkCommponent::forwardMessage(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 ROS_WARN( "%s foward message from %u for %u type %u",
           this->model->GetName( ).c_str( ), _msg->from, _msg->forward,
           _msg->payload[1] );
 abstract_drone::WirelessMessage WM;
 WM.request.from = this->NodeID;
 uint8_t nextInLine = NodeTable.getDirectionToNode( _msg->forward );
 if ( nextInLine == 255 ) return;
 WM.request.to = nextInLine;
 WM.request.message = *( _msg );
 WM.request.message.from = this->NodeID;
 WM.request.message.to = nextInLine;
 if ( publishService.call( WM ) ) ++totalMessageSent;
}

bool MeshnetworkCommponent::sendHeartbeat( uint8_t other )
{
 // create a Message to introduce yourself to others
 uint8_t payload[28];
 const uint8_t constID = this->NodeID;
 HeartbeatMessage heartbeat( constID, connectedToGateway, prefferedGateWay );
 abstract_drone::WirelessMessage WM;
 abstract_drone::NRF24 nrfmsg;

 uint8_t towards = NodeTable.getDirectionToNode( other );
 if ( towards == 255 ) return false;
 WM.request.from = this->NodeID;
 WM.request.to = towards;
 nrfmsg.from = this->NodeID;
 nrfmsg.to = towards;
 nrfmsg.forward = other;
 nrfmsg.ack = 0;
 heartbeat.toPayload( nrfmsg.payload.data( ) );
 WM.request.message = nrfmsg;
 ++totalMessageSent;
 if ( publishService.call( WM ) ) {
  if ( !WM.response.succes ) {
   if ( NodeTable.proofOfDeceased( towards, towards ) > 0 )
    informAboutDeceasedChild( this->NodeID, towards );
  } else {
   NodeTable.proofOfLive( towards, towards );
   return true;
  }
 } else {
  if ( NodeTable.proofOfDeceased( towards, towards ) > 0 )
   informAboutDeceasedChild( this->NodeID, towards );
  return false;
 }
}

void MeshnetworkCommponent::sendGoalToDrone( const uint8_t ID,
                                             const float longitude,
                                             const float latitude,
                                             const uint16_t height )
{
 GoToLocationMessage GTLmsg( this->NodeID, longitude, latitude, height );

 abstract_drone::WirelessMessage WM;
 abstract_drone::NRF24 nrfmsg;

 uint8_t towards = NodeTable.getDirectionToNode( ID );
 WM.request.from = this->NodeID;
 WM.request.to = towards;
 nrfmsg.from = this->NodeID;
 nrfmsg.to = towards;
 nrfmsg.forward = ID;
 nrfmsg.ack = 0;
 GTLmsg.toPayload( nrfmsg.payload.data( ) );
 WM.request.message = nrfmsg;
 ++totalMessageSent;
 if ( publishService.call( WM ) ) {
  if ( !WM.response.succes ) {
   NodeTable.proofOfDeceased( towards, towards );
  } else {
   NodeTable.proofOfLive( towards, towards );
  }
 } else {
  NodeTable.proofOfDeceased( towards, towards );
 }
}

void MeshnetworkCommponent::processDeceased(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 DeceasedMessage msg( _msg->payload.data( ) );
 if ( NodeTable.proofOfDeceased( msg.getID( ), msg.getDeceased( ) ) > 0 )
  informAboutDeceasedChild( msg.getID( ), msg.getDeceased( ) );
}

void MeshnetworkCommponent::informAboutDeceasedChild( uint8_t parent,
                                                      uint8_t child )
{
 DeceasedMessage deceased( this->NodeID, child );
 abstract_drone::WirelessMessage WM;
 abstract_drone::NRF24 nrfmsg;
 for ( auto &other : NodeTable.getFamily( ) ) {
  if ( parent == other.first ) continue;
  uint8_t towards = NodeTable.getDirectionToNode( other.first );
  if ( towards == 255 ) continue;
  WM.request.from = this->NodeID;
  WM.request.to = towards;
  nrfmsg.from = this->NodeID;
  nrfmsg.to = towards;
  nrfmsg.forward = other.first;
  nrfmsg.ack = 0;
  deceased.toPayload( nrfmsg.payload.data( ) );
  WM.request.message = nrfmsg;
  ++totalMessageSent;
  if ( publishService.call( WM ) ) {
   if ( !WM.response.succes ) {
    NodeTable.proofOfDeceased( towards, towards );
   } else {
    NodeTable.proofOfLive( towards, towards );
   }
  } else {
   NodeTable.proofOfDeceased( towards, towards );
  }
 }
}

void MeshnetworkCommponent::searchOtherNodesInRange( )
{
 abstract_drone::AreaScan scanMsg;
 scanMsg.request.id = this->NodeID;
 ROS_WARN( "Node %d  started to look for others", this->NodeID );
 if ( areaScanner.call( scanMsg ) ) {
  if ( scanMsg.response.near.empty( ) ) {
   ROS_WARN( "Node %d found none", this->NodeID );
   return;
  }

  ROS_WARN( "%s  found  %d others", this->model->GetName( ).c_str( ),
            scanMsg.response.near.size( ) );
  for ( const uint8_t &n : scanMsg.response.near ) {
   IntroduceNode( n );
  }
 } else {
  ROS_ERROR( "Failed to call service othersInRange" );
 }
}

void MeshnetworkCommponent::processIntroduction(
    const abstract_drone::NRF24ConstPtr &_msg )
{
 //  IntroduceMessage introduce( _msg->payload.data( ) );
 //  ROS_INFO( "%s recieved IntroduceMessage %s", this->model->GetName( ).c_str(
 //  ),
 //            introduce.toString( ).c_str( ) );
 //  auto from = connectedNodes.find( introduce.getID( ) );
 //  if ( introduce.getHopsUntilsGateway( ) <
 //       HopsUntilGateway )  // minus one since we are only interested in
 //       shorter
 //                           // paths not alternatives
 //  {
 //   shortestPathToGatewayID = introduce.getID( );
 //   HopsUntilGateway = introduce.getHopsUntilsGateway( ) + 1;
 //  }
 //  if ( from != connectedNodes.end( ) )  // A known node update the hops
 //  {
 //   connectedNodes.insert( std::pair< uint8_t, uint8_t >(
 //       introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
 //  } else  // A new node lets add him and send back a response
 //  {
 //   connectedNodes.insert( std::pair< uint8_t, uint8_t >(
 //       introduce.getID( ), introduce.getHopsUntilsGateway( ) ) );
 //   IntroduceNode( introduce.getID( ) );
 //  }
}
void MeshnetworkCommponent::IntroduceNode( uint8_t other )
{
 ROS_INFO( "%s presents himself to %d", this->model->GetName( ).c_str( ) ,other );
 // create a Message to introduce yourself to others
 uint8_t payload[28];

 IntroduceMessage introduce( this->NodeID, this->HopsUntilGateway );
 abstract_drone::WirelessMessage WM;
 abstract_drone::NRF24 nrfmsg;
 WM.request.from = this->NodeID;
 WM.request.to = other;
 nrfmsg.from = this->NodeID;
 nrfmsg.to = other;
 nrfmsg.forward = other;
 nrfmsg.ack = 0;
 introduce.toPayload( nrfmsg.payload.data( ) );
 WM.request.message = nrfmsg;
 ++totalMessageSent;
 if ( publishService.call( WM ) ) {
  if ( !WM.response.succes ) {
   if ( NodeTable.proofOfDeceased( other, other ) > 0 )
    informAboutDeceasedChild( this->NodeID, other );
  } else {
   NodeTable.proofOfLive( other, other );
   return;
  }
 } else {
  if ( NodeTable.proofOfDeceased( other, other ) > 0 )
   informAboutDeceasedChild( this->NodeID, other );
  return;
 }
}

void MeshnetworkCommponent::reassignID( uint8_t ID )
{
 std::string Node_TopicName = "/Node/" + std::to_string( ID );
 ROS_WARN( "Reassign to ID %d", ID );
 this->rosNode.reset( new ros::NodeHandle( "node" ) );
 this->NodeID = ID;
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
 GoToLocationMessage locmsg( _msg->payload.data( ) );
 abstract_drone::Location msg;
 msg.latitude = locmsg.latitude;
 msg.longitude = locmsg.longitude;
 msg.height = locmsg.height;

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