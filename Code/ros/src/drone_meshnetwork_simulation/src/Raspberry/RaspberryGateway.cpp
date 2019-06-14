#include <iostream>
#include "../Communication/Meshnetwork/MeshnetworkRouter.hpp"
#include "../Communication/RoutingTechnique/HybridLMRoutingProtocol.hpp"
#include "../Drone/MockDroneEngine.hpp"
#include "../NRF24/NRF24HighLevelInterface.hpp"
#include "../Pistache/InternetGateway/InternetGateway.hpp"

uint8_t nodeID = 0;
uint8_t droneID = 0;
bool debug = false;

/// \brief A pointer to the connected MeshnetworkRouter
// Communication::Meshnetwork::MeshnetworkRouter meshnetworkRouter;
/// \brief Pointer to the routing technique
Communication::RoutingTechnique::HybridLMRoutingProtocol routing;
MockDroneEngine engine;
NRF24HighLevelInterface nRF24;
InternetGateway gateway;

int main( int argc, char const *argv[] )
{
 std::cout << "START" << std::endl;
 Communication::Meshnetwork::MeshnetworkGateway meshnetworkGateway(
     nodeID, droneID, debug, &routing, &engine, &nRF24, &gateway );
 std::cout << "INITIALIZING" << std::endl;
 for ( int i = 0; i < 10000000; i++ ) { /**wait**/
 }
 meshnetworkRouter.Init( );
 std::cout << "IDLE" << std::endl;
 while ( true ) { /**run**/
 }
 return 0;
}
