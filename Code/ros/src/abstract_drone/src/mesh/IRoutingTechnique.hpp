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
 virtual uint8_t getDirectionToNode( const uint8_t node ) = 0;
 virtual void startRouting( ) = 0;
 virtual void maintainRouting( ) = 0;
 virtual void NodeMovedLocation( ) = 0;
 virtual void canCommunicateWithNode( const uint8_t node ) = 0;
 virtual uint8_t cantCommunicateWithNode( const uint8_t node ) = 0;
 virtual uint8_t OtherCantCommunicateWithNode( const uint8_t other,
                                               const uint8_t node ) = 0;
 virtual void OtherCanCommunicateWithNode( const uint8_t other,
                                           const uint8_t node ) = 0;
 virtual const uint16_t getTableSize( ) = 0;
 virtual const uint16_t getAmountOfChildren( ) = 0;
 const std::set< uint8_t > getSetOfChildren( );
 virtual const bool empty( ) = 0;
};
}  // namespace RoutingTechnique

#endif  // IROUTINGTECHNIQUE
