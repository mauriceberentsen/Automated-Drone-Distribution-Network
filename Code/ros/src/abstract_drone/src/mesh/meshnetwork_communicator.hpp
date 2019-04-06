#ifndef MESHNETWORKCOMMUNICATOR
#define MESHNETWORKCOMMUNICATOR

#include "meshnetwork_component.hpp"

namespace gazebo
{
class MeshnetworkCommunicator : public MeshnetworkCommponent
{
private:
 /**INTERFACE FUNCTIONS**/
 void OnUpdate( );
 void processMessage( const abstract_drone::NRF24ConstPtr &_msg );
 void processIntroduction( const abstract_drone::NRF24ConstPtr &_msg );
 void CheckConnection( );
 /**COMMUNICATOR FUNCTIONS**/
 uint8_t getNodePath( uint8_t other );
 void sendHeartbeatToGateway( );
 void lostConnection( );
 void processHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
 

};  // namespace gazebo
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( MeshnetworkCommunicator )
}  // namespace gazebo
#endif  // MESHNETWORKCOMMUNICATOR