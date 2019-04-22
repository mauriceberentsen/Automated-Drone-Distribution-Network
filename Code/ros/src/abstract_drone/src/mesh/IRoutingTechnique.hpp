/**
 * @file IRoutingTechnique.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Interface for Routing Techniques
 * @version 1.0
 * @date 2019-04-04
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IROUTINGTECHNIQUE
#define IROUTINGTECHNIQUE
#include <inttypes.h>
#include <set>

namespace RoutingTechnique
{
class IRoutingTechnique
{
public:
 virtual ~IRoutingTechnique( ){};
 /**
  * @brief Get the node you need to communicate with to let the message reach.
  * the destination
  *
  * @param node destination node.
  * @return uint8_t node to communicate with.
  * @return 255 if destination not found.
  */
 virtual uint8_t getDirectionToNode( const uint8_t node ) = 0;
 /**
  * @brief Call this method is called once to start the routing technique.
  *
  */
 virtual void startRouting( ) = 0;
 /**
  * @brief This method is called every time to maintain the routing.
  *
  */
 virtual void maintainRouting( ) = 0;
 /**
  * @brief This method is called after the object is moved.
  *
  */
 virtual void NodeMovedLocation( ) = 0;
 /**
  * @brief This method gets called when this node can't communicate with another
  * node.
  *
  * @param node other node ID
  * @return uint8_t amount of nodes removed from the table
  */
 virtual uint8_t cantCommunicateWithNode( const uint8_t node ) = 0;
 /**
  * @brief This method is called when another node informs this node that he
  * lost a connection with a node.
  *
  * @param other The one that informs you about losing a connection
  * @param node The node that he lost a connection with
  * @return uint8_t The amount of nodes that this node removed from his table
  */
 virtual uint8_t OtherCantCommunicateWithNode( const uint8_t other,
                                               const uint8_t node ) = 0;
 /**
  * @brief This method is called when another node informs this node that he
  * found a new connection with a node.
  *
  * @param other The one that informs you about a connection.
  * @param node The node that he found a new connection with.
  */
 virtual void OtherCanCommunicateWithNode( const uint8_t other,
                                           const uint8_t node ) = 0;
 /**
  * @brief This method is called if confirm that we can communicate with another
  * node
  *
  * @param node The node we have a confirmed connection with;
  */
 virtual void canCommunicateWithNode( const uint8_t node ) = 0;
 /**
  * @brief Get the amount of all known node in this network
  *
  * @return const uint16_t amount of nodes
  */
 virtual const uint16_t getTableSize( ) = 0;
 /**
  * @brief Get the Amount Of Children to this node
  *
  * @return const uint16_t  amount of children
  */
 virtual const uint16_t getAmountOfChildren( ) = 0;
 /**
  * @brief Get a set of directly connected Nodes to this node.
  *
  * @return const std::set< uint8_t > Set with all connected children in it
  */
 virtual const std::set< uint8_t > getSetOfChildren( ) = 0;
 /**
  * @brief Used to see if this node as any connection
  *
  * @return true if no connections are registered
  * @return false if any connection is registered
  */
 virtual const bool empty( ) = 0;
};
}  // namespace RoutingTechnique

#endif  // IROUTINGTECHNIQUE
