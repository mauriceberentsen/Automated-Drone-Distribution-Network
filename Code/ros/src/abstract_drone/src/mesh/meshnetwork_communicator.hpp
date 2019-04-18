#ifndef MESHNETWORKCOMMUNICATOR
#define MESHNETWORKCOMMUNICATOR

#include "meshnetwork_component.hpp"

namespace gazebo
{
class MeshnetworkCommunicator : public MeshnetworkComponent
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
 void ProcessHeartbeat( const abstract_drone::NRF24ConstPtr &_msg );
 void StartEmergencyProtocol( );
 void startMovementNegotiation( );
 void informOthersAboutDistance( float distance );
 void processMovementNegotiationMessage(
     const abstract_drone::NRF24ConstPtr &_msg );
 bool timerStarted = false;
 std::multimap< float, uint8_t > negotiationList;
 common::Time lastTimeOnline;
};  // namespace gazebo
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( MeshnetworkCommunicator )
}  // namespace gazebo
#endif  // MESHNETWORKCOMMUNICATOR