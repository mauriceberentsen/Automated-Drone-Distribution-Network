#ifndef MESHNETWORKCOMPONENT
#define MESHNETWORKCOMPONENT

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <map>
#include "message.hpp"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_srvs/Trigger.h"
#include "abstract_drone/Location.h"
#include "abstract_drone/NRF24.h"
#include "abstract_drone/nodeInfo.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/AreaScan.h"
#include "abstract_drone/NodeDebugInfo.h"
#include "ChildTableTree.hpp"

#include <random>
using namespace Messages;
namespace gazebo
{
class MeshnetworkComponent : public ModelPlugin
{
public:
 void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
 void forwardMessage( const abstract_drone::NRF24ConstPtr &_msg );
 void OnRosMsg( const abstract_drone::NRF24ConstPtr &_msg );

 virtual void OnUpdate( ) = 0;
 virtual void processMessage( const abstract_drone::NRF24ConstPtr &_msg ) = 0;
 virtual void processIntroduction(
     const abstract_drone::NRF24ConstPtr &_msg ) = 0;

 void publishDebugInfo( );
 bool sendHeartbeat( uint8_t other );
 void searchOtherNodesInRange( );
 void IntroduceNode( uint8_t other );
 void reassignID( uint8_t ID );
 void informAboutMissingChild( uint8_t parent, uint8_t child );
 void processMissing( const abstract_drone::NRF24ConstPtr &_msg );
 void sendGoalToDrone( const uint8_t ID, const float longitude,
                       const float latitude, const uint16_t height );
 void sendGoalToEngine( const abstract_drone::NRF24ConstPtr &_msg );
 void sendGoalToEngine( const LocationMessage &_msg );
 bool switchPower( std_srvs::TriggerRequest &request,
                   std_srvs::TriggerResponse &response );
 virtual void lostConnection( ) = 0;
 void requestLocation( const uint8_t other );
 void processRequestLocation( const abstract_drone::NRF24ConstPtr &_msg );
 void sendLocation( const uint8_t other );
 void processLocation( const abstract_drone::NRF24ConstPtr &_msg );
 float distanceBetweenMeAndLocation( const LocationMessage &A );
 // Called by the world update start event
 /// \brief ROS helper function that processes messages
protected:
 void QueueThread( );
 virtual void CheckConnection( ) = 0;
 /// \brief A ROS subscriber
 bool sendMessage( abstract_drone::WirelessMessage &message );

 uint8_t nodeID;
 bool init = false;
 uint16_t droneID;

 std::map< uint8_t, uint8_t > connectedNodes;  // ID and hop route;

 uint32_t totalMessageSent = 0;
 ros::ServiceClient areaScanner;
 ros::ServiceClient GPSLink;

 ros::ServiceClient publishService;
 // Pointer to the model

 physics::ModelPtr model;
 // Pointer to the update event connection

 event::ConnectionPtr updateConnection;
 /// \brief A node use for ROS transport

 std::unique_ptr< ros::NodeHandle > rosNode;
 /// \brief A ROS publisher

 ros::Publisher rosPub;
 ros::Publisher nodeDebugTopic;
 ros::ServiceServer switchPowerService;

 ros::Publisher droneEnginePublisher;
 ros::Subscriber rosSub;
 /// \brief A ROS callbackqueue that helps process messages

 ros::CallbackQueue rosQueue;
 /// \brief A thread the keeps running the rosQueue

 std::thread rosQueueThread;
 std::thread heartbeatThread;
 std::thread NodeInfoThread;
 LocationMessage lastGoodKnownLocation = LocationMessage( 0, 0, 0, 0, 0 );
 bool knowPrefferedGatewayLocation = false;
 LocationMessage prefferedGateWayLocation = LocationMessage( 0, 0, 0, 0, 0 );
 RoutingTechnique::ChildTableTree nodeTable;
 bool on = true;
 bool connectedToGateway = false;
 bool isGateway = false;
 uint8_t prefferedGateWay = 0;
 uint8_t hopsFromGatewayAway = 0;

};  // namespace gazebo
}  // namespace gazebo

#endif  // MESHNETWORKCOMPONENT