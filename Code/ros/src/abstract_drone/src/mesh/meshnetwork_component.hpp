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
#include "IRoutingTechnique.hpp"
#include <random>

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkComponent : public ModelPlugin
 {
 public:
  MeshnetworkComponent( );
  virtual void Init( ) = 0;
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  void forwardMessage( const abstract_drone::NRF24ConstPtr &_msg );
  void OnRosMsg( const abstract_drone::NRF24ConstPtr &_msg );

  void processMessage( const abstract_drone::NRF24ConstPtr &_msg );
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
  void sendGoalToEngine( const Messages::LocationMessage &_msg );
  bool switchPower( std_srvs::TriggerRequest &request,
                    std_srvs::TriggerResponse &response );
  virtual void lostConnection( ) = 0;
  void requestLocation( const uint8_t other );
  void processRequestLocation( const abstract_drone::NRF24ConstPtr &_msg );
  void sendLocation( const uint8_t other );
  void processLocation( const abstract_drone::NRF24ConstPtr &_msg );
  float distanceBetweenMeAndLocation( const Messages::LocationMessage &A );
  virtual void processMovementNegotiationMessage(
      const abstract_drone::NRF24ConstPtr &_msg ) = 0;
  virtual void ProcessHeartbeat(
      const abstract_drone::NRF24ConstPtr &_msg ) = 0;

 protected:
  void QueueThread( );
  virtual void CheckConnection( ) = 0;
  bool sendMessage( abstract_drone::WirelessMessage &message );

  uint8_t nodeID;
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
  Messages::LocationMessage lastGoodKnownLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0 );
  bool knowPrefferedGatewayLocation = false;
  Messages::LocationMessage prefferedGateWayLocation =
      Messages::LocationMessage( 0, 0, 0, 0, 0 );
  std::unique_ptr< RoutingTechnique::IRoutingTechnique > routerTech;
  bool on = true;
  bool connectedToGateway = false;
  bool isGateway = false;
  uint8_t prefferedGateWay = 0;
  uint8_t hopsFromGatewayAway = 0;
  const float initTime = 0.001;
  const float CheckConnectionTime = 10.0;
 };
}  // namespace Meshnetwork
}  // namespace gazebo

#endif  // MESHNETWORKCOMPONENT