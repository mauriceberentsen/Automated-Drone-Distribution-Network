#include "meshnetwork_gateway.hpp"

namespace gazebo
{

void MeshnetworkGateway::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->NodeID = 255;  //unkown nodes are number 255
  this->droneID = 666; //Drones need unique id's in the simulation to connect each motor


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MeshnetworkGateway::OnUpdate, this));

  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("NodeID"))
  {
    this->NodeID = _sdf->Get<int>("NodeID");
  }
  if (_sdf->HasElement("DroneID"))
  {
    this->droneID = _sdf->Get<int>("DroneID");
  }
  else
  {
    ROS_ERROR("Drone com being used without a droneid, set up using the tag <DroneID> unique <DroneID> \n  Restart ussing droneID's else drone movement will be messy");
  }
  std::string Node_TopicName;
  Node_TopicName = "/Node/" + std::to_string(NodeID);
  std::string ModelName = this->model->GetName();
  std::string WirelessSignalSimulatorName = "/postMaster";
  std::string connectedEngine = "/Drones/" + std::to_string(this->droneID) + "/goal";

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "node",
              ros::init_options::NoSigintHandler);
  }
  ROS_INFO("modelName =%s", Node_TopicName.c_str());
  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("node"));

  // Plannar Pose
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<abstract_drone::NRF24>(
          Node_TopicName,
          1,
          boost::bind(&MeshnetworkGateway::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);
  this->rosPub = this->rosNode->advertise<abstract_drone::nodeInfo>(WirelessSignalSimulatorName, 100);
  this->droneEnginePublisher = this->rosNode->advertise<abstract_drone::Location>(connectedEngine, 100);
  this->areaScanner = this->rosNode->serviceClient<abstract_drone::AreaScan>("/SignalSimulator/othersInRange");
  this->publishService = this->rosNode->serviceClient<abstract_drone::WirelessMessage>("/SignalSimulator/message");
  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread(std::bind(&MeshnetworkGateway::QueueThread, this));

  ROS_WARN("Loaded MeshnetworkGateway Plugin with parent...%s", this->model->GetName().c_str());
  scanMsg.request.id = this->NodeID;

  abstract_drone::nodeInfo nodeinf;
  nodeinf.nodeID = NodeID;
  nodeinf.sub = Node_TopicName;
  rosPub.publish(nodeinf);
}

void MeshnetworkGateway::OnUpdate()
{
  //Will probally stay empty
}

void MeshnetworkGateway::IntroduceNode(uint8_t ID)
{
  //create a Message to introduce yourself to others
  uint8_t payload[28];
  ROS_WARN("%s ==> ID = [%d] HOPS = [%d]", this->model->GetName().c_str(), this->NodeID, this->HopsUntilGateway);
  IntroduceMessage introduce(this->NodeID, this->HopsUntilGateway);
  ROS_INFO("%s created IntroduceMessage %s", this->model->GetName().c_str(), introduce.toString().c_str());
  abstract_drone::WirelessMessage WM;
  abstract_drone::NRF24 msg;
  WM.request.from = this->NodeID;
  WM.request.to = ID;
  msg.from = this->NodeID;
  msg.to = ID;
  msg.forward = ID;
  msg.ack = 0;
  introduce.toPayload(msg.payload.data());
  WM.request.message = msg;
  publishService.call(WM);
}

void MeshnetworkGateway::processIntroduction(const abstract_drone::NRF24ConstPtr &_msg)
{
  IntroduceMessage introduce(_msg->payload.data());
  ROS_INFO("%s recieved IntroduceMessage %s", this->model->GetName().c_str(), introduce.toString().c_str());
  auto from = connectedNodes.find(introduce.getID());
  if (from != connectedNodes.end())
  { // A known node update the hops
    ROS_INFO("NODE %d UPDATE", introduce.getID());
    connectedNodes.insert(std::pair<uint8_t, uint8_t>(introduce.getID(), introduce.getHopsUntilsGateway()));
  }
  else //A new node lets add him and send back a response
  {
    ROS_INFO("NEW NODE %d ADDED", introduce.getID());
    connectedNodes.insert(std::pair<
                          uint8_t, uint8_t>(introduce.getID(), introduce.getHopsUntilsGateway()));
    IntroduceNode(introduce.getID());
  }
}

void MeshnetworkGateway::processMessage(const abstract_drone::NRF24ConstPtr &_msg)
{
  ROS_WARN("ProcessMessage with type %d", _msg->payload[1]);
  const uint8_t msgType = _msg->payload[1];
  switch (msgType)
  {
  case NOTDEFINED:
    ROS_WARN("Not defined message recieved");
    ROS_WARN("Payload %s", _msg->payload);
    break;
  case LOCATION:
    ROS_WARN("Location message recieved");
    break;
  case PRESENT:
    ROS_INFO("%s Recieved PRESENT", this->model->GetName().c_str());
    processIntroduction(_msg);
    break;
  case SIGNON:
    ROS_WARN("%s SIGNON message recieved", this->model->GetName().c_str());
    _msg->from == 255 ? handOutNewID(_msg) : registerNode(_msg);
    break;
  case HEARTBEAT:
    ROS_WARN("HEARTBEAT message recieved");
    processHeartbeat(_msg);
    break;
  case GIVEID:
    ROS_WARN("GIVEID message recieved");
    break;
  default:
    ROS_WARN("UNKNOWN message recieved");
    break;
  }
}
void MeshnetworkGateway::processHeartbeat(const abstract_drone::NRF24ConstPtr &_msg)
{
  Message msg(_msg->payload.data());
  ROS_WARN("%s heartbeat message recieved: %s", msg.toString().c_str(), this->model->GetName().c_str());
}

void MeshnetworkGateway::registerNode(const abstract_drone::NRF24ConstPtr &_msg)
{
  ROS_WARN("NODE REGISTERED");
}

void MeshnetworkGateway::handOutNewID(const abstract_drone::NRF24ConstPtr &_msg)
{
  ROS_WARN("ASSIGNING NEW ID: not working");
  // abstract_drone::NRF24 outgoing_message;
  // outgoing_message.from = this->NodeID;
  // outgoing_message.to = _msg->from;
  // outgoing_message.forward = _msg->from;
  // Message* msgptr;
  // uint8_t payload[28];

  // msgptr = new GiveIDMessage(NodeID, OtherNodes);
  // msgptr->toPayload(payload);
  // outgoing_message.payload.at(0) = payload[0];
  // outgoing_message.payload.at(1) = payload[1];
  // outgoing_message.payload.at(2) = payload[2];
  // outgoing_message.payload.at(3) = payload[3];
  // rosPub.publish(outgoing_message);
}

void MeshnetworkGateway::forwardMessage(const abstract_drone::NRF24ConstPtr &_msg)
{
  ROS_WARN("%s foward message", this->model->GetName().c_str());
}

void MeshnetworkGateway::OnRosMsg(const abstract_drone::NRF24ConstPtr &_msg)
{
  (_msg->forward == this->NodeID || _msg->forward == 0) ? processMessage(_msg) : forwardMessage(_msg);
  //TODO send ack message back
}

/// \brief ROS helper function that processes messages

void MeshnetworkGateway::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

} // namespace gazebo