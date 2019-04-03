#ifndef MESHNETWORKCOMMUNICATOR
#define MESHNETWORKCOMMUNICATOR


#include "meshnetwork_component.hpp"

namespace gazebo
{
class MeshnetworkCommunicator : public MeshnetworkCommponent
{
public:
  void OnUpdate();
  void processMessage(const abstract_drone::NRF24ConstPtr &_msg);
  void processIntroduction(const abstract_drone::NRF24ConstPtr &_msg);

  uint8_t getNodePath(uint8_t other);


  // Called by the world update start event
  /// \brief ROS helper function that processes messages
protected:
  void CheckConnection();
  void sendHeartbeatToGateway();
  void lostConnection();

}; // namespace gazebo

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MeshnetworkCommunicator)
} // namespace gazebo
#endif //MESHNETWORKCOMMUNICATOR