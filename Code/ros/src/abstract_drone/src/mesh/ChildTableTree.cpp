#include "ChildTableTree.hpp"
#include <iostream>
namespace gazebo
{
ChildTableTree::ChildTableTree( )
{
}

ChildTableTree::~ChildTableTree( )
{
}

uint8_t ChildTableTree::getDirectionToNode( const uint8_t node )
{
 // is it one of our own childeren?
 auto it = family.find( node );
 if ( it != family.end( ) )  // we dont know the child add him to the family
 {
  return it->first;
 } else  // is it one of the grandchilderen? Direct them that way
 {
  for ( auto& child : family ) {
   if ( child.second.find( node ) != child.second.end( ) ) return child.first;
  }
 }
 return 255;
}

uint8_t ChildTableTree::proofOfDeceased( uint8_t teller, uint8_t child )
{
 // teller claims that he lost a child, check if he is talking about himself
 if ( teller == child ) {
  // Disable the route to this child;
  std::cout << "Killed child " << ( int )child << " got [" << family.size( )
            << "] left" << std::endl;
  return family.erase( child );

 } else {
  if ( family.find( teller ) != family.end( ) )
   return family.at( teller ).erase( child );
 }
 return 0;
}

void ChildTableTree::proofOfLive( uint8_t teller, uint8_t child )
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

void ChildTableTree::RegisterChild( uint8_t child )
{
 auto it = family.find( child );
 if ( it == family.end( ) )  // we dont know the child add him to the family
 {
  family.insert( std::pair< uint8_t, std::set< uint8_t > >(
      child, std::set< uint8_t >( ) ) );
  std::cout << "added child " << ( int )child << " got [" << family.size( )
            << "] now" << std::endl;
 }
}

void ChildTableTree::RegisterGrandChildOfChild( uint8_t child,
                                                uint8_t grandChild )
{
 auto it = family.find( child );
 if ( it == family.end( ) )  // unknown child requested
 {
  return;
 } else {
  it->second.insert( grandChild );
  std::cout << "added child[ " << ( int )child << "][" << ( int )grandChild
            << "] got [" << family.size( ) << "] now" << std::endl;
  return;  // add grandchild to set and return;
 }
}

uint16_t ChildTableTree::familysize( )
{
 uint16_t size = 0;
 for ( auto& child : family ) {
  size += child.second.size( );
 }
 return size;
}
}  // namespace gazebo