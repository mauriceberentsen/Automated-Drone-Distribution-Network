/**
 * @file MockNetworkComponent.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief
 * @version 1.0
 * @date 2019-05-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef MOCKNETWORKCOMPONENT
#define MOCKNETWORKCOMPONENT
#include "../Communication/Wireless/IMeshDebugInfo.hpp"
#include "../Communication/Wireless/IMeshNetwork.hpp"
#include "NRF24HighLevelInterface.hpp"

class MockNetworkComponent : public Communication::Wireless::IMeshNetwork,
                             public Communication::Wireless::IMeshDebugInfo
{
private:
 uint8_t NodeID;
 uint8_t other;
 NRF24HighLevelInterface highLevelInterface;

public:
 explicit MockNetworkComponent( uint8_t id, uint8_t other );
 ~MockNetworkComponent( );
 /* IMeshNetwork */
 void OnMsg( const uint8_t* message );
 const uint8_t getNodeID( ) const;
 /* IMeshDebugInfo */
 /**
  * @brief Get the ConnectedToGateway
  *
  * @return true connected
  * @return false not connected
  */
 const bool getConnectedToGateway( ) const;
 /**
  * @brief Get the Total Message Sent
  *
  * @return const uint32_t amount
  */
 const uint32_t getTotalMessageSent( ) const;
 /**
  * @brief Get the Preffered Gateway ID
  *
  * @return const uint8_t ID
  */
 const uint8_t getPrefferedGateway( ) const;
 /**
  * @brief Get theHopsFromGatewayAway
  *
  * @return const uint8_t amount of hops away
  */
 const uint8_t getHopsFromGatewayAway( ) const;
 /**
  * @brief Get the ID of LastGoodKnownLocation
  *
  * @return const uint8_t ID
  */
 const uint8_t getLastGoodKnownLocationID( ) const;
 /**
  * @brief Get the size of the RouterTechTable
  *
  * @return const uint8_t
  */
 const uint8_t getRouterTechTableSize( ) const;

 void DoStuf( );
};

#endif  // MOCKNETWORKCOMPONENT
