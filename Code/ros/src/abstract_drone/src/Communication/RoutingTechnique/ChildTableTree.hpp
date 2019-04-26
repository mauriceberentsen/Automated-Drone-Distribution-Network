/**
 * @file ChildTableTree.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for the class ChildTableTree
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
 class ChildTableTree : public IRoutingTechnique
 {
 public:
  /**
   * @brief Construct a new Child Table Tree object
   *
   * @param MC reference to a MeshnetworkComponent because we want to send
   * messages for maintainance
   */
  explicit ChildTableTree( Meshnetwork::MeshnetworkComponent& MC );
  ~ChildTableTree( );

  /*virtual */ uint8_t getDirectionToNode( const uint8_t node );
  /**
   * @brief We search for others in range
   *
   */
  /*virtual */ void startRouting( );
  /**
   * @brief We search for others in range and send them a heartbeat
   *
   */
  /*virtual */ void maintainRouting( );
  /*virtual */ void canCommunicateWithNode( const uint8_t node );
  /*virtual */ uint8_t cantCommunicateWithNode( const uint8_t node );
  /*virtual */ uint8_t OtherCantCommunicateWithNode( const uint8_t other,
                                                     const uint8_t node );
  /*virtual */ void OtherCanCommunicateWithNode( const uint8_t other,
                                                 const uint8_t node );
  /*virtual */ const uint16_t getAmountOfChildren( );
  /*virtual */ const uint16_t getTableSize( );
  /*virtual */ const bool empty( );
  /*virtual */ const std::set< uint8_t > getSetOfChildren( );
  /*virtual */ void NodeMovedLocation( );

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
  Meshnetwork::MeshnetworkComponent& meshnetworkComponent;
  /// \brief The family with connected childeren and grandchildren.
  std::map< uint8_t, std::set< uint8_t > > family;
 };
}  // namespace RoutingTechnique
}  // namespace Communication
#endif  // CHILDTABLETREE
