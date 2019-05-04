/**
 * @file ChildTableTree.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief source file for the class ChildTableTree
 * @version 1.0
 * @date 2019-04-04
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <iostream>

#include "ChildTableTree.hpp"
#include "../Meshnetwork/MeshnetworkComponent.hpp"

namespace Communication
{
namespace RoutingTechnique
{
 ChildTableTree::ChildTableTree( IRoutingEssentials& MC )
     : meshnetworkComponent( MC )
 {
 }

 ChildTableTree::~ChildTableTree( )
 {
 }

 void ChildTableTree::startRouting( )
 {
  meshnetworkComponent.searchOtherNodesInRange( );
 }
 void ChildTableTree::maintainRouting( )
 {
  meshnetworkComponent.searchOtherNodesInRange( );
  for ( auto& node : getSetOfChildren( ) )
   meshnetworkComponent.sendHeartbeat( node );
 }
 void ChildTableTree::canCommunicateWithNode( const uint8_t node )
 {
  proofOfAvailability( node, node );
 }
 uint8_t ChildTableTree::cantCommunicateWithNode( const uint8_t node )
 {
  return proofOfMissing( node, node );
 }
 uint8_t ChildTableTree::OtherCantCommunicateWithNode( const uint8_t other,
                                                       const uint8_t node )
 {
  return proofOfMissing( other, node );
 }
 void ChildTableTree::OtherCanCommunicateWithNode( const uint8_t other,
                                                   const uint8_t node )
 {
  proofOfAvailability( other, node );
 }

 uint8_t ChildTableTree::getDirectionToNode( const uint8_t node )
 {
  // is it one of our own childeren?
  auto it = family.find( node );
  if ( it != family.end( ) ) {
   return it->first;
  } else  // is it one of the grandchilderen? Direct them that way
  {
   for ( auto& child : family ) {
    if ( child.second.find( node ) != child.second.end( ) ) return child.first;
   }
  }
  return UINT8_MAX;
 }

 const uint16_t ChildTableTree::getAmountOfChildren( )
 {
  return family.size( );
 }

 const uint16_t ChildTableTree::getTableSize( )
 {
  uint16_t size = 0;
  for ( auto& child : family ) {
   ++size;
   size += child.second.size( );
  }
  return size;
 }

 const bool ChildTableTree::empty( )
 {
  return family.empty( );
 };

 const std::set< uint8_t > ChildTableTree::getSetOfChildren( )
 {
  std::set< uint8_t > children;
  for ( auto& child : family )
   children.insert( child.first );
  return children;
 }
 void ChildTableTree::NodeMovedLocation( )
 {
  family.clear( );
 }
 /*private*/ void ChildTableTree::RegisterGrandChildOfChild(
     uint8_t child, uint8_t grandChild )
 {
  auto it = family.find( child );
  if ( it == family.end( ) )  // unknown child requested
  {
   return;
  } else {
   it->second.insert( grandChild );
   return;  // add grandchild to set and return;
  }
 }

 /*private*/ uint8_t ChildTableTree::proofOfMissing( uint8_t teller,
                                                     uint8_t child )
 {
  // teller claims that he lost a child, check if he is talking about himself
  if ( teller == child ) {
   // delete the teller;
   return family.erase( child );

  } else {
   if ( family.find( teller ) != family.end( ) )
    return family.at( teller ).erase( child );
  }
  return 0;
 }

 /*private*/ void ChildTableTree::proofOfAvailability( uint8_t teller,
                                                       uint8_t child )
 {
  // teller claims that he knows child, check if he is talking about himself
  if ( teller == child ) {
   // talking about himself do we know him already?
   RegisterChild( teller );
   return;
  } else {
   // child not talking about himself but still check if we know him;
   RegisterChild( teller );
   // add the grand child to his family
   RegisterGrandChildOfChild( teller, child );
  }
 }

 /*private*/ void ChildTableTree::RegisterChild( uint8_t child )
 {
  auto it = family.find( child );
  if ( it == family.end( ) )  // we dont know the child add him to the family
  {
   family.insert( std::pair< uint8_t, std::set< uint8_t > >(
       child, std::set< uint8_t >( ) ) );
  }
 }
}  // namespace RoutingTechnique
}  // namespace Communication