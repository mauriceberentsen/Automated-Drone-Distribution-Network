/**
 * @file HybridLMRoutingProtocol.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for the class HybridLMRoutingProtocol
 * @version 1.0
 * @date 2019-04-04
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef CHILDTABLETREE
#define CHILDTABLETREE
#include <map>
#include <set>

#include "IRoutingEssentials.hpp"
#include "IRoutingTechnique.hpp"
namespace Communication
{
namespace Meshnetwork
{
 class MeshnetworkComponent;
}
}  // namespace Communication

namespace Communication
{
namespace RoutingTechnique
{
 class HybridLMRoutingProtocol : public IRoutingTechnique
 {
 public:
  /**
   * @brief Construct a new Child Table Tree object
   *
   */
  HybridLMRoutingProtocol( );
  /**
   * @brief Destroy the Child Table Tree object
   *
   */
  ~HybridLMRoutingProtocol( );

  /**
   * @brief Get the node you need to communicate with to let the message reach.
   * the destination
   *
   * @param node destination node.
   * @return uint8_t node to communicate with.
   * @return 255 if destination not found.
   */
  uint8_t getDirectionToNode( const uint8_t node );
  /**
   * @brief Call this method is called once to start the routing technique.
   *
   */
  void startRouting( Communication::RoutingTechnique::IRoutingEssentials* IRE );
  /**
   * @brief This method is called every time to maintain the routing.
   *
   */
  void maintainRouting( );
  /**
   * @brief This method is called after the object is moved.
   *
   */
  void NodeMovedLocation( );
  /**
   * @brief This method gets called when this node can't communicate with
   * another node.
   *
   * @param node other node ID
   * @return uint8_t amount of nodes removed from the table
   */
  uint8_t cantCommunicateWithNode( const uint8_t node );
  /**
   * @brief This method is called when another node informs this node that he
   * lost a connection with a node.
   *
   * @param other The one that informs you about losing a connection
   * @param node The node that he lost a connection with
   * @return uint8_t The amount of nodes that this node removed from his table
   */
  uint8_t OtherCantCommunicateWithNode( const uint8_t other,
                                        const uint8_t node );
  /**
   * @brief This method is called when another node informs this node that he
   * found a new connection with a node.
   *
   * @param other The one that informs you about a connection.
   * @param node The node that he found a new connection with.
   */
  void OtherCanCommunicateWithNode( const uint8_t other, const uint8_t node );
  /**
   * @brief This method is called if confirm that we can communicate with
   * another node
   *
   * @param node The node we have a confirmed connection with;
   */
  void canCommunicateWithNode( const uint8_t node );
  /**
   * @brief Get the amount of all known node in this network
   *
   * @return const uint16_t amount of nodes
   */
  const uint16_t getTableSize( );
  /**
   * @brief Get the Amount Of Children to this node
   *
   * @return const uint16_t  amount of children
   */
  const uint16_t getAmountOfChildren( );
  /**
   * @brief Get a set of directly connected Nodes to this node.
   *
   * @return const std::set< uint8_t > Set with all connected children in it
   */
  const std::set< uint8_t > getSetOfChildren( );
  /**
   * @brief Used to see if this node as any connection
   *
   * @return true if no connections are registered
   * @return false if any connection is registered
   */
  const bool empty( );

 private:
  /**
   * @brief We add the child node and his connected node to our table of
   * connections If we find a child of our own we use his ID for both params
   *
   * @param teller The one that found a node
   * @param child The one that is found
   */
  void proofOfAvailability( uint8_t teller, uint8_t child );
  /**
   * @brief If we have proof that a node went missing remove him from the table.
   *        When it's a child of our own the ID goes into both params.
   *
   * @param teller The one that lost a node
   * @param child The node that is lost
   * @return uint8_t The amount that is removed from this table
   */
  uint8_t proofOfMissing( uint8_t teller, uint8_t child );
  /**
   * @brief Registers a child node to the table
   *
   * @param child that we want to add
   */
  void RegisterChild( uint8_t child );
  /**
   * @brief Register a grandchild to a child in our table
   *
   * @param child The parent of the grandchild
   * @param grandChild The grandchild that needs to be added
   */
  void RegisterGrandChildOfChild( uint8_t child, uint8_t grandChild );

 private:
  /// \brief The conneceted MeshnetworkComponent used for sending messages to
  /// other nodes.
  IRoutingEssentials* meshnetworkComponent;
  /// \brief The family with connected childeren and grandchildren.
  std::map< uint8_t, std::set< uint8_t > > family;
 };
}  // namespace RoutingTechnique
}  // namespace Communication
#endif  // CHILDTABLETREE
