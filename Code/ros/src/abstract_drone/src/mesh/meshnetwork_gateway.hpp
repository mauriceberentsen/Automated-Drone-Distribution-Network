#ifndef MESHNETWORKGATEWAY
#define MESHNETWORKGATEWAY

#include "meshnetwork_component.hpp"
#include "abstract_drone/SwitchBool.h"
#include "abstract_drone/RequestGatewayDroneFlight.h"

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkGateway : public MeshnetworkComponent
 {
 private:
  ros::Subscriber gatewaySub;

  /**INTERFACE FUNCTIONS**/
  void gatewayQueue(
      const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg );
  void Init( );
  void processIntroduction( const abstract_drone::NRF24ConstPtr &_msg );
  void CheckConnection( );
  void lostConnection( ){};
  /**GATEWAY FUNCTIONS**/
  void ProcessHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
  void processMovementNegotiationMessage(
      const abstract_drone::NRF24ConstPtr &_msg );
 };
 // Register this plugin with the simulator
}  // namespace Meshnetwork
GZ_REGISTER_MODEL_PLUGIN( Meshnetwork::MeshnetworkGateway )
}  // namespace gazebo
#endif  // MESHNETWORKGATEWAY