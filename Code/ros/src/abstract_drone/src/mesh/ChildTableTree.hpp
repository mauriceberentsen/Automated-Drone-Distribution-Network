#ifndef CHILDTABLETREE
#define CHILDTABLETREE
#include <map>
#include <set>
#include "IRoutingTechnique.hpp"

namespace gazebo
{
namespace Meshnetwork
{
 class MeshnetworkComponent;
}
}  // namespace gazebo
namespace RoutingTechnique
{
class ChildTableTree : public IRoutingTechnique
{
public:
 explicit ChildTableTree( gazebo::Meshnetwork::MeshnetworkComponent& MC );
 ~ChildTableTree( );

 /*virtual */ uint8_t getDirectionToNode( const uint8_t node );
 /*virtual */ void startRouting( );
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
 void proofOfAvailability( uint8_t teller, uint8_t child );
 uint8_t proofOfMissing( uint8_t teller, uint8_t child );
 void RegisterChild( uint8_t child );
 void RegisterGrandChildOfChild( uint8_t child, uint8_t grandChild );
 gazebo::Meshnetwork::MeshnetworkComponent& meshnetworkComponent;
 std::map< uint8_t, std::set< uint8_t > > family;
};
}  // namespace RoutingTechnique
#endif  // CHILDTABLETREE
