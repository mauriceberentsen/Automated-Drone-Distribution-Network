#include "meshnetwork_component.hpp"

namespace gazebo
{
class MeshnetworkGateway : public MeshnetworkCommponent
{
public:
  MeshnetworkGateway();

  void OnUpdate();
  void processMessage(const abstract_drone::NRF24ConstPtr &_msg);
  void processIntroduction(const abstract_drone::NRF24ConstPtr &_msg);

  void processHeartbeat(const abstract_drone::NRF24ConstPtr &_msg);

  void registerNode(const abstract_drone::NRF24ConstPtr &_msg);
  void handOutNewID(const abstract_drone::NRF24ConstPtr &_msg);






  void CheckConnection(){};

  /// \brief ROS helper function that processes messages

  //std::map<uint8_t, uint8_t> connectedNodes; //ID and hop route towards gateway
  //abstract_drone::AreaScan scanMsg;
  //uint8_t shortestPathToGatewayID = 0;
  //uint8_t HopsUntilGateway = 0;
  // uint16_t droneID;
  // uint8_t NodeID = 0;
  // physics::ModelPtr model;
  // event::ConnectionPtr updateConnection;
  // std::unique_ptr<ros::NodeHandle> rosNode;
  // ros::Publisher rosPub;
  //ros::Publisher droneEnginePublisher;
  // ros::Subscriber rosSub;
  // ros::CallbackQueue rosQueue;
  // std::thread rosQueueThread;
  // ros::ServiceClient areaScanner;
  // ros::ServiceClient publishService;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MeshnetworkGateway)
} // namespace gazebo