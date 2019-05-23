#include "../Communication/Meshnetwork/MeshnetworkRouter.hpp"
//#include "../ros/RosDroneEngineConnector.hpp"
#include "../Communication/RoutingTechnique/HybridLMRoutingProtocol.hpp"
//#include "../ros/WirelessSimulation/VirtualNRF24.hpp"
#include "../Drone/MockDroneEngine.hpp"
#include "../NRF24/NRF24HighLevelInterface.hpp"
#include <iostream>

uint8_t nodeID = 17;
uint8_t droneID = 17;
bool debug = false;

  /// \brief A pointer to the connected MeshnetworkRouter
  //Communication::Meshnetwork::MeshnetworkRouter meshnetworkRouter;
  /// \brief Pointer to the routing technique
  Communication::RoutingTechnique::HybridLMRoutingProtocol routing;
  MockDroneEngine engine;  
  NRF24HighLevelInterface nRF24;

int main(int argc, char const *argv[])
{
  std::cout<<"START"<<std::endl;
  Communication::Meshnetwork::MeshnetworkRouter meshnetworkRouter( nodeID, droneID, debug, &routing, &engine, &nRF24 );
  std::cout<<"INITIALIZING"<<std::endl;
  for(int i = 0; i < 10000000; i++){/**wait**/}
  meshnetworkRouter.Init( );
  std::cout<<"IDLE"<<std::endl;
  while(true){/**run**/}
    return 0;
}
