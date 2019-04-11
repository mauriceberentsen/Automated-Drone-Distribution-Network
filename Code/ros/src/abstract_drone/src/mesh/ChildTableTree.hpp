#ifndef CHILDTABLETREE
#define CHILDTABLETREE
#include "IRoutingTechnique.hpp"
#include <map>
#include <set>
namespace gazebo
{
class ChildTableTree : public IRoutingTechnique
{
public:
 ChildTableTree( );
 ~ChildTableTree( );
 uint8_t getDirectionToNode( const uint8_t node );
 void proofOfLive(uint8_t teller, uint8_t child);
 uint8_t proofOfDeceased( uint8_t teller, uint8_t child );
 std::map<uint8_t,std::set<uint8_t>> getFamily() {return family;}
 bool empty(){return family.empty();};
 uint16_t familysize();
private:
  void RegisterChild(uint8_t child);
  void RegisterGrandChildOfChild(uint8_t child, uint8_t grandChild);
  std::map<uint8_t,std::set<uint8_t>> family;
  };
}
#endif  // CHILDTABLETREE
