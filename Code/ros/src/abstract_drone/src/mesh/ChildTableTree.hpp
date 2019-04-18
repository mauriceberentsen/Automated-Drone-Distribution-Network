#ifndef CHILDTABLETREE
#define CHILDTABLETREE
//#include "IRoutingTechnique.hpp"
#include <map>
#include <set>

namespace RoutingTechnique
{
class ChildTableTree  //: public IRoutingTechnique
{
public:
 ChildTableTree( );
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

 std::map< uint8_t, std::set< uint8_t > > family;
};
}  // namespace RoutingTechnique
#endif  // CHILDTABLETREE
