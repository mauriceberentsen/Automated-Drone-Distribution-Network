#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "abstract_drone/Location.h"
#include "abstract_drone/NRF24.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/nodeInfo.h"
#include "abstract_drone/AreaScan.h"
#include "message.hpp"
#include <set>

namespace gazebo
{
class MeshnetworkGateway : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();
  void forwardMessage(const abstract_drone::NRF24ConstPtr &_msg);
  void OnRosMsg(const abstract_drone::NRF24ConstPtr &_msg);
  void processMessage(const abstract_drone::NRF24ConstPtr &_msg);
  void processIntroduction(const abstract_drone::NRF24ConstPtr &_msg);

  void processHeartbeat(const abstract_drone::NRF24ConstPtr &_msg);
  void registerNode(const abstract_drone::NRF24ConstPtr &_msg);
  void handOutNewID(const abstract_drone::NRF24ConstPtr &_msg);
  void IntroduceNode(uint8_t ID);







  /// \brief ROS helper function that processes messages
private:
  void QueueThread();

  std::map<uint8_t, uint8_t> connectedNodes; //ID and hop route towards gateway
  abstract_drone::AreaScan scanMsg;
  uint8_t shortestPathToGatewayID = 0;
  uint8_t HopsUntilGateway = 0;
  std::set<uint8_t> nodeList;
  uint16_t droneID;
  uint8_t NodeID = 0;
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Publisher rosPub;
  ros::Publisher droneEnginePublisher;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  ros::ServiceClient areaScanner;
  ros::ServiceClient publishService;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MeshnetworkGateway)
} // namespace gazebo