#include "meshnetwork_communicator.hpp"

namespace gazebo
{

void MeshnetworkCommunicator::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->NodeID = 255;  //unkown nodes are number 255
  this->droneID = 666; //Drones need unique id's in the simulation to connect each motor

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
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MeshnetworkCommunicator::OnUpdate, this));
  std::string Node_TopicName;
  std::string WirelessSignalSimulatorName;
  std::string ModelName = this->model->GetName();
  // Check that the velocity element exists, then read the value
  Node_TopicName = "/Node/" + std::to_string(NodeID);
  WirelessSignalSimulatorName = "/postMaster";
  std::string connectedEngine = "/Drones/" + std::to_string(this->droneID) + "/goal";
  std::string DebugInfoName = "/Nodes/" + std::to_string(this->droneID) + "/debug";

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
          boost::bind(&MeshnetworkCommunicator::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);
  this->rosPub = this->rosNode->advertise<abstract_drone::nodeInfo>(WirelessSignalSimulatorName, 100);
  this->NodeDebugTopic = this->rosNode->advertise<abstract_drone::NodeDebugInfo>(DebugInfoName, 100);
  this->droneEnginePublisher = this->rosNode->advertise<abstract_drone::Location>(connectedEngine, 100);
  this->areaScanner = this->rosNode->serviceClient<abstract_drone::AreaScan>("/SignalSimulator/othersInRange");
  this->publishService = this->rosNode->serviceClient<abstract_drone::WirelessMessage>("/SignalSimulator/message");
  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread(std::bind(&MeshnetworkCommunicator::QueueThread, this));
  this->heartbeatThread =
      std::thread(std::bind(&MeshnetworkCommunicator::CheckConnection, this));
  this->NodeInfoThread =
      std::thread(std::bind(&MeshnetworkCommunicator::publishDebugInfo, this));
  ROS_WARN("Loaded MeshnetworkCommunicator Plugin with parent...%s", this->model->GetName().c_str());

  abstract_drone::nodeInfo nodeinf;
  nodeinf.nodeID = NodeID;
  nodeinf.sub = Node_TopicName;
  rosPub.publish(nodeinf);
}
void MeshnetworkCommunicator::publishDebugInfo()
{
  abstract_drone::NodeDebugInfo msg;
  while (this->rosNode->ok())
  {
    common::Time::Sleep(1); //1hz refresh is enough
    msg.nodeID = this->NodeID;
    msg.HopsUntilGateway = this->HopsUntilGateway;
    msg.PreferredNode = this->shortestPathToGatewayID;
    msg.totalMessages = this->totalMessageSund;
    NodeDebugTopic.publish(msg);
  }
}

// Called by the world update start event
void MeshnetworkCommunicator::OnUpdate()
{
  if (!this->init) // we want this to happen once after all nodes are loaded
  {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(10.0, 20.0);

    common::Time::Sleep(dist(mt));
    searchOtherNodesInRange();
    //simulate startup time of a nrf node
    //TODO research how long a nrf takes to start;
    //search a connection route;
    //scan for nodes Nodes near
    init = true;
  }
}
bool MeshnetworkCommunicator::sendHeartbeat(uint8_t other, bool gateway)
{
  //ROS_INFO("Send a heartbeat to %d", other);
  //create a Message to introduce yourself to others
  uint8_t payload[28];
  const uint8_t constID = this->NodeID;
  Message heartbeat(constID, HEARTBEAT);
  //ROS_INFO("%s created heartbeat %s", this->model->GetName().c_str(), heartbeat.toString().c_str());
  abstract_drone::WirelessMessage WM;
  abstract_drone::NRF24 nrfmsg;
  uint8_t toward = gateway ? other : shortestPathToGatewayID;
  WM.request.from = this->NodeID;
  WM.request.to = toward;
  nrfmsg.from = this->NodeID;
  nrfmsg.to = other;
  nrfmsg.forward = toward;
  nrfmsg.ack = 0;
  heartbeat.toPayload(nrfmsg.payload.data());
  WM.request.message = nrfmsg;
  ++totalMessageSund;
  if (publishService.call(WM))
  {
    //ROS_INFO("%s We have [%s] heartbeat to [%d]",this->model->GetName().c_str() , std::to_string(WM.response.succes).c_str(), other);
    return WM.response.succes;
  }
  else
  {
    //("%s, We have no repsonse to heartbeat service", this->model->GetName().c_str());
    return false;
  }
}

void MeshnetworkCommunicator::searchOtherNodesInRange()
{
  abstract_drone::AreaScan scanMsg;
  scanMsg.request.id = this->NodeID;
  ROS_WARN("Node %d  started to look for others", this->NodeID);
  if (areaScanner.call(scanMsg))
  {
    if (scanMsg.response.near.empty())
    {
      ROS_WARN("Node %d found none", this->NodeID);
      return;
    }

    ROS_WARN("%s  found  %d others", this->model->GetName().c_str(), scanMsg.response.near.size());
    for (const uint8_t &n : scanMsg.response.near)
    {
      IntroduceNode(n);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service othersInRange");
  }
}

void MeshnetworkCommunicator::IntroduceNode(uint8_t other)
{
  ROS_INFO("PRESENT ourself to %d", other);
  //create a Message to introduce yourself to others
  uint8_t payload[28];
  ROS_WARN("%s ==> ID = [%d] HOPS = [%d]", this->model->GetName().c_str(), this->NodeID, this->HopsUntilGateway);
  IntroduceMessage introduce(this->NodeID, this->HopsUntilGateway);
  ROS_INFO("%s created IntroduceMessage %s", this->model->GetName().c_str(), introduce.toString().c_str());
  abstract_drone::WirelessMessage WM;
  abstract_drone::NRF24 nrfmsg;
  WM.request.from = this->NodeID;
  WM.request.to = other;
  nrfmsg.from = this->NodeID;
  nrfmsg.to = other;
  nrfmsg.forward = other;
  nrfmsg.ack = 0;
  introduce.toPayload(nrfmsg.payload.data());
  WM.request.message = nrfmsg;
  if (publishService.call(WM))
    ++totalMessageSund;
}

void MeshnetworkCommunicator::processIntroduction(const abstract_drone::NRF24ConstPtr &_msg)
{
  IntroduceMessage introduce(_msg->payload.data());
  ROS_INFO("%s recieved IntroduceMessage %s", this->model->GetName().c_str(), introduce.toString().c_str());
  auto from = connectedNodes.find(introduce.getID());
  if (introduce.getHopsUntilsGateway() < HopsUntilGateway) //minus one since we are only interested in shorter paths not alternatives
  {
    shortestPathToGatewayID = introduce.getID();
    HopsUntilGateway = introduce.getHopsUntilsGateway() + 1;
  }
  if (from != connectedNodes.end()) // A known node update the hops
  {
    connectedNodes.insert(std::pair<uint8_t, uint8_t>(introduce.getID(), introduce.getHopsUntilsGateway()));
  }
  else //A new node lets add him and send back a response
  {
    connectedNodes.insert(std::pair<uint8_t, uint8_t>(introduce.getID(), introduce.getHopsUntilsGateway()));
    IntroduceNode(introduce.getID());
  }
}

void MeshnetworkCommunicator::reassignID(uint8_t ID)
{
  std::string Node_TopicName = "/Node/" + std::to_string(ID);
  ROS_WARN("Reassign to ID %d", ID);
  this->rosNode.reset(new ros::NodeHandle("node"));
  this->NodeID = ID;
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<abstract_drone::NRF24>(
          Node_TopicName,
          1,
          boost::bind(&MeshnetworkCommunicator::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);
}

void MeshnetworkCommunicator::processMessage(const abstract_drone::NRF24ConstPtr &_msg)
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
  case SIGNON:
    ROS_WARN("%s SIGNON message recieved", this->model->GetName().c_str());
    break;
  case GIVEID:
    ROS_WARN("GIVEID message recieved");
    reassignID(_msg->payload[2]);
    break;
  case PRESENT:
    ROS_INFO("%s Recieved PRESENT", this->model->GetName().c_str());
    processIntroduction(_msg);
    break;
  case MOVE_TO_LOCATION:
    ROS_WARN("MOVE_TO_LOCATION message recieved");
    sendGoalToEngine(_msg);
    break;
  default:
    ROS_WARN("UNKNOWN message recieved");
    break;
  }
}

void MeshnetworkCommunicator::forwardMessage(const abstract_drone::NRF24ConstPtr &_msg)
{
  ROS_WARN("%s foward message", this->model->GetName().c_str());
  abstract_drone::WirelessMessage WM;
  WM.request.from = this->NodeID;
  uint8_t nextInLine = (_msg->forward == 0) ? this->shortestPathToGatewayID : getNodePath(_msg->forward);
  WM.request.to = nextInLine;
  WM.request.message = *(_msg);
  WM.request.message.from = this->NodeID;
  WM.request.message.to = nextInLine;
  if (publishService.call(WM))
    ++totalMessageSund;
}

uint8_t MeshnetworkCommunicator::getNodePath(uint8_t other)
{
  return 2;
}

void MeshnetworkCommunicator::sendGoalToEngine(const abstract_drone::NRF24ConstPtr &_msg)
{
  GoToLocationMessage locmsg(_msg->payload.data());
  abstract_drone::Location msg;
  msg.latitude = locmsg.latitude;
  msg.longitude = locmsg.longitude;
  msg.height = locmsg.height;

  droneEnginePublisher.publish(msg);
}

void MeshnetworkCommunicator::OnRosMsg(const abstract_drone::NRF24ConstPtr &_msg)
{
  (_msg->forward == this->NodeID || _msg->forward == 255) ? processMessage(_msg) : forwardMessage(_msg);
  //TODO send ack message back
}

/// \brief ROS helper function that processes messages
void MeshnetworkCommunicator::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void MeshnetworkCommunicator::CheckConnection()
{
  while (this->rosNode->ok())
  {
    common::Time::Sleep(30); //check every 30 seconds
    if (!sendHeartbeat(shortestPathToGatewayID))
    {
      lostConnection();
    }
    else
    {
      searchOtherNodesInRange(); //maybe there is someone closer
      sendHeartbeatToGateway();
    }
  }
}
void MeshnetworkCommunicator::sendHeartbeatToGateway()
{
}
void MeshnetworkCommunicator::lostConnection()
{
  //ROS_WARN("%s lost conncection", this->model->GetName().c_str());
  shortestPathToGatewayID = 255;
  HopsUntilGateway = 255;
}

} // namespace gazebo