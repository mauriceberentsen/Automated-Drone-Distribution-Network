#ifndef IROUTINGTECHNIQUE
#define IROUTINGTECHNIQUE
#include <inttypes.h>

namespace gazebo
{
class IRoutingTechnique
{

public:
 virtual uint8_t getDirectionToNode( const uint8_t node ) = 0;


};
}  // namespace gazebo

#endif  // IROUTINGTECHNIQUE
