#include "../Communication/Meshnetwork/MeshnetworkRouter.hpp"
//#include "../ros/RosDroneEngineConnector.hpp"
#include "../Communication/RoutingTechnique/HybridLMRoutingProtocol.hpp"
//#include "../ros/WirelessSimulation/VirtualNRF24.hpp"
#include "../Drone/MockDroneEngine.hpp"
#include "../NRF24/NRF24HighLevelInterface.hpp"

uint8_t nodeID = 1;
uint8_t droneID = 1;
bool debug = false;

  /// \brief A pointer to the connected MeshnetworkRouter
  Communication::Meshnetwork::MeshnetworkRouter meshnetworkRouter;
  /// \brief Pointer to the routing technique
  Communication::RoutingTechnique::HybridLMRoutingProtocol routing;
  MockDroneEngine engine;  
  NRF24HighLevelInterface NRF24;

int main(int argc, char const *argv[])
{
     meshnetworkRouter = new Communication::Meshnetwork::MeshnetworkRouter(
      nodeID, droneID, debug, &routing, &engine, &NRF24 );
  meshnetworkRouter->Init( );
    return 0;
}
