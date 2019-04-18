#ifndef MESHNETWORKGATEWAY
#define MESHNETWORKGATEWAY

#include "meshnetwork_component.hpp"
#include "abstract_drone/SwitchBool.h"
#include "abstract_drone/RequestGatewayDroneFlight.h"

namespace gazebo
{
class MeshnetworkGateway : public MeshnetworkComponent
{
public:
 MeshnetworkGateway( );

private:
 ros::Subscriber gatewaySub;

 /**INTERFACE FUNCTIONS**/
 void gatewayQueue(const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg);
 void OnUpdate( );
 void processMessage( const abstract_drone::NRF24ConstPtr &_msg );
 void processIntroduction( const abstract_drone::NRF24ConstPtr &_msg );
 void CheckConnection( );
 void lostConnection( ){};
 /**GATEWAY FUNCTIONS**/
 void ProcessHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( MeshnetworkGateway )
}  // namespace gazebo
#endif  // MESHNETWORKGATEWAY