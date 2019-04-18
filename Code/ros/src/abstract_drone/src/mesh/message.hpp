#ifndef MESSAGEHPP
#define MESSAGEHPP

#include <inttypes.h>
#include <string>
namespace Messages
{
enum Messagetype : uint8_t {
 NOTDEFINED = 0,
 LOCATION,              // 1
 REQUESTLOCATION,       // 2
 PRESENT,               // 4
 HEARTBEAT,             // 5
 MISSING,               // 6
 MOVE_TO_LOCATION,      // 8
 MOVEMENT_NEGOTIATION,  // 9
};

class Message
{
public:
 Message( const uint8_t _ID, Messagetype _Messagetype );
 explicit Message( const uint8_t *payload );
 ~Message( );
 virtual void toPayload( uint8_t *payload );
 virtual std::string toString( );
 void CopyFromCharArray( uint8_t *value, uint16_t size, const uint8_t *arr,
                         uint16_t start );
 const uint8_t getID( );

protected:
 uint8_t ID;

 void CopyToCharArray( uint8_t *value, uint16_t size, uint8_t *arr,
                       uint16_t start );
 uint8_t type;
};

class LocationMessage : public Message
{
public:
 LocationMessage( uint8_t _ID, float latitude, float longitude, int16_t height,
                  uint32_t timeSincePosix );
 explicit LocationMessage( const uint8_t *payload );
 ~LocationMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );

 float latitude, longitude;
 int16_t height;

private:
 uint32_t timeSincePosix;
};

class IntroduceMessage : public Message
{
public:
 IntroduceMessage( const uint8_t _ID, const uint8_t _hopsUntilsGateway,
                   const bool _knowGateway );
 explicit IntroduceMessage( const uint8_t *payload );
 ~IntroduceMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 bool getKnowGateway( );
 uint8_t getHopsUntilsGateway( );

private:
 uint8_t hopsUntilsGateway;
 bool knowGateway;
};

class HeartbeatMessage : public Message
{
public:
 HeartbeatMessage( const uint8_t _ID, const bool _knowGateway,
                   const uint8_t _prefferedGateWay, uint8_t _hops = 0 );
 explicit HeartbeatMessage( const uint8_t *payload );
 ~HeartbeatMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 bool getKnowGateway( );
 uint8_t getPrefferedGateway( );
 bool getIsGateway( );
 uint8_t hops;
 uint8_t getHops( )
 {
  return hops;
 };
 void makeHop( )
 {
  ++hops;
 };

private:
 bool knowGateway;
 uint8_t prefferedGateWay;
};

class MissingMessage : public Message
{
public:
 MissingMessage( const uint8_t _ID, const uint8_t _deceased );
 explicit MissingMessage( const uint8_t *payload );
 ~MissingMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 uint8_t getDeceased( );

private:
 uint8_t deceased;
};

class GoToLocationMessage : public Message
{
public:
 GoToLocationMessage( uint8_t _ID, float latitude, float longitude,
                      int16_t height );
 explicit GoToLocationMessage( const uint8_t *payload );
 ~GoToLocationMessage( ){};
 std::string toString( );
 void toPayload( uint8_t *payload );
 float latitude, longitude;
 int16_t height;
};

class MovementNegotiationMessage : public Message
{
public:
 MovementNegotiationMessage( const uint8_t _ID, const float _distance );
 explicit MovementNegotiationMessage( const uint8_t *payload );
 ~MovementNegotiationMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 float getDistance( );

private:
 float distance;
};
}  // namespace Messages
#endif  // MESSAGEHPP