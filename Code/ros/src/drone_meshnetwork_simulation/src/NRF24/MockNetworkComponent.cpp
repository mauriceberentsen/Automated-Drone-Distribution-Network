/**
 * @file MockNetworkComponent.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief
 * @version 1.0
 * @date 2019-05-22
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "MockNetworkComponent.hpp"
#include <iostream>
#include "../Communication/Messages/Message.hpp"
using namespace Communication::Messages;

MockNetworkComponent::MockNetworkComponent( uint8_t id, uint8_t other )
    : NodeID( id ), other( other )
{
 highLevelInterface.StartAntenna( this );
}

MockNetworkComponent::~MockNetworkComponent( )
{
}

void MockNetworkComponent::DoStuf( )
{
 uint8_t buf[32] = {0};
 LocationMessage msg( NodeID, NodeID, other, 5, 12.00, 14.00, 69, 74185 );
 msg.toPayload( buf );
 static int D = 0;
 ++D;
 if ( D > 100000000 ) {
  std::cout << "Send" << std::endl;
  if ( highLevelInterface.SendMessageTo( buf ) ) {
   std::cout << "Success" << std::endl;
  }
  D = 0;
 }
}

void MockNetworkComponent::OnMsg( const uint8_t* message )
{
 for ( int i = 0; i < 32; i++ ) {
  std::cout << ( int )message[i];
 }
 std::cout << std::endl;
}
const uint8_t MockNetworkComponent::getNodeID( ) const
{
 return NodeID;
}
/* IMeshDebugInfo */
/**
 * @brief Get the ConnectedToGateway
 *
 * @return true connected
 * @return false not connected
 */
const bool MockNetworkComponent::getConnectedToGateway( ) const
{
 return 0;
}
/**
 * @brief Get the Total Message Sent
 *
 * @return const uint32_t amount
 */
const uint32_t MockNetworkComponent::getTotalMessageSent( ) const
{
 return 0;
}
/**
 * @brief Get the Preffered Gateway ID
 *
 * @return const uint8_t ID
 */
const uint8_t MockNetworkComponent::getPrefferedGateway( ) const
{
 return 0;
}
/**
 * @brief Get theHopsFromGatewayAway
 *
 * @return const uint8_t amount of hops away
 */
const uint8_t MockNetworkComponent::getHopsFromGatewayAway( ) const
{
 return 0;
}
/**
 * @brief Get the ID of LastGoodKnownLocation
 *
 * @return const uint8_t ID
 */
const uint8_t MockNetworkComponent::getLastGoodKnownLocationID( ) const
{
 return 0;
}
/**
 * @brief Get the size of the RouterTechTable
 *
 * @return const uint8_t
 */
const uint8_t MockNetworkComponent::getRouterTechTableSize( ) const
{
 return 0;
}
