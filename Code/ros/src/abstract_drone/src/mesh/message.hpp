#ifndef MESSAGEHPP
#define MESSAGEHPP

#include <inttypes.h>
#include <string>

enum Messagetype : uint8_t {
 NOTDEFINED = 0,
 LOCATION,
 REQUESTLOCATION,
 GIVEID,
 PRESENT,
 HEARTBEAT,
 DECEASED,
 FLOOD,
 MOVE_TO_LOCATION,
 MOVEMENT_NEGOTIATION,
 SIGNON = 255
};

class Message
{
public:
 Message( const uint8_t _ID, Messagetype _Messagetype );
 Message( const uint8_t *payload );
 ~Message( );
 virtual void toPayload( uint8_t *payload );
 virtual std::string toString( );
 void CopyFromCharArray( uint8_t *value, uint16_t size, const uint8_t *arr,
                         uint16_t start );
 uint8_t getID( )
 {
  return ID;
 }

protected:
 uint8_t ID;

 void CopyToCharArray( uint8_t *value, uint16_t size, uint8_t *arr,
                       uint16_t start );
 uint8_t type;
};

class locationMessage : public Message
{
public:
 locationMessage( uint8_t _ID, float latitude, float longitude, int16_t height,
                  uint32_t timeSincePosix, bool _knowGateway );
 locationMessage( const uint8_t *payload );
 ~locationMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );

 float latitude, longitude;
 int16_t height;
 bool knowGateway;
private:
 uint32_t timeSincePosix;
};

class GiveIDMessage : public Message
{
public:
 GiveIDMessage( uint8_t _ID, uint8_t Give )
     : Message( ID, GIVEID ), GiveID( Give ){};
 ~GiveIDMessage( ){};
 std::string toString( );
 void toPayload( uint8_t *payload );
 uint8_t GiveID;
};

class IntroduceMessage : public Message
{
public:
 IntroduceMessage( const uint8_t _ID,const bool _knowGateway, const uint8_t _hopsUntilsGateway );
 IntroduceMessage( const uint8_t *payload );
 ~IntroduceMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
  bool getKnowGateway( );
 uint8_t getHopsUntilsGateway( );

private:
 bool knowGateway;
 uint8_t hopsUntilsGateway;
};

class HeartbeatMessage : public Message
{
public:
 HeartbeatMessage( const uint8_t _ID, const bool _knowGateway, const uint8_t _prefferedGateWay, uint8_t _hops = 0 );
 HeartbeatMessage( const uint8_t *payload );
 ~HeartbeatMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 bool getKnowGateway( );
 uint8_t getPrefferedGateway();
 bool getIsGateway( );
 uint8_t hops;
 uint8_t getHops(){return hops;};
 void makeHop() {++hops;};
private:
 bool knowGateway;
 uint8_t prefferedGateWay;
};

class DeceasedMessage : public Message
{ 
public:
 DeceasedMessage( const uint8_t _ID, const uint8_t _deceased );
 DeceasedMessage( const uint8_t *payload );
 ~DeceasedMessage( );
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
 GoToLocationMessage( const uint8_t *payload );
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
 MovementNegotiationMessage( const uint8_t *payload );
 ~MovementNegotiationMessage( );
 std::string toString( );
 void toPayload( uint8_t *payload );
 float getDistance( );

private:
 float distance;
};

#endif  // MESSAGEHPP