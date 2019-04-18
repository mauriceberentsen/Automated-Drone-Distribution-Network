#include "message.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>
namespace Messages
{
Message::Message( const uint8_t _ID, Messagetype _Messagetype )
    : ID( _ID ), type( _Messagetype )
{
}
Message::Message( const uint8_t *payload )
    : Message( payload[0], ( Messagetype )payload[1] )
{
}
Message::~Message( )
{
}

std::string Message::toString( )
{
 std::stringstream ss;
 ss << "ID[" << ( int )ID << "] Type[" << ( int )type << "]" << std::endl;

 return ss.str( );
}

void Message::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
}

void Message::CopyToCharArray( uint8_t *value, uint16_t size, uint8_t *arr,
                               uint16_t start )
{
 for ( uint16_t i = 0; i < size; i++ ) {
  arr[i + start] = value[i];
 }
}

void Message::CopyFromCharArray( uint8_t *value, uint16_t size,
                                 const uint8_t *arr, uint16_t start )
{
 for ( uint16_t i = 0; i < size; i++ ) {
  value[i] = arr[i + start];
 }
}

const uint8_t Message::getID( )
{
 return ID;
}

LocationMessage::LocationMessage( uint8_t _ID, float _latitude,
                                  float _longitude, int16_t _height,
                                  uint32_t _timeSincePosix )
    : Message( _ID, LOCATION )
    , latitude( _latitude )
    , longitude( _longitude )
    , height( _height )
    , timeSincePosix( _timeSincePosix )
{
}

LocationMessage::LocationMessage( const uint8_t *payload )
    : Message( payload[0], LOCATION )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&latitude, sizeof( latitude ), payload,
                    counter );
 counter += sizeof( latitude );
 CopyFromCharArray( ( uint8_t * )&longitude, sizeof( longitude ), payload,
                    counter );
 counter += sizeof( longitude );
 CopyFromCharArray( ( uint8_t * )&height, sizeof( height ), payload, counter );
 counter += sizeof( height );
 CopyFromCharArray( ( uint8_t * )&timeSincePosix, sizeof( timeSincePosix ),
                    payload, counter );
}

LocationMessage::~LocationMessage( )
{
}

void LocationMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&latitude, sizeof( latitude ), payload,
                  counter );
 counter += sizeof( latitude );
 CopyToCharArray( ( uint8_t * )&longitude, sizeof( longitude ), payload,
                  counter );
 counter += sizeof( longitude );
 CopyToCharArray( ( uint8_t * )&height, sizeof( height ), payload, counter );
 counter += sizeof( height );
 CopyToCharArray( ( uint8_t * )&timeSincePosix, sizeof( timeSincePosix ),
                  payload, counter );
}

std::string LocationMessage::toString( )
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "latitude[" << latitude << "] longitude[" << longitude << "] height["
    << height << "] timeSincePosix[" << timeSincePosix << "]" << std::endl;

 return ss.str( );
}

IntroduceMessage::IntroduceMessage( const uint8_t _ID,
                                    const uint8_t _hopsUntilsGateway,
                                    const bool _knowGateway )
    : Message( _ID, PRESENT )
    , hopsUntilsGateway( _hopsUntilsGateway )
    , knowGateway( _knowGateway )
{
}

IntroduceMessage::~IntroduceMessage( )
{
}

IntroduceMessage::IntroduceMessage( const uint8_t *payload )
    : Message( ( uint8_t )payload[0], PRESENT )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&hopsUntilsGateway,
                    sizeof( hopsUntilsGateway ), payload, counter );
 counter += sizeof( hopsUntilsGateway );
 CopyFromCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                    counter );
}

void IntroduceMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&hopsUntilsGateway, sizeof( hopsUntilsGateway ),
                  payload, counter );
 counter += sizeof( hopsUntilsGateway );
 CopyToCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                  counter );
}

std::string IntroduceMessage::toString( )
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( hopsUntilsGateway ) << "]"
    << std::endl;

 return ss.str( );
}

uint8_t IntroduceMessage::getHopsUntilsGateway( )
{
 return hopsUntilsGateway;
}

bool IntroduceMessage::getKnowGateway( )
{
 return knowGateway;
}

MissingMessage::MissingMessage( const uint8_t _ID, const uint8_t _deceased )
    : Message( _ID, MISSING ), deceased( _deceased )
{
}

MissingMessage::~MissingMessage( )
{
}

MissingMessage::MissingMessage( const uint8_t *payload )
    : Message( ( uint8_t )payload[0], MISSING )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&deceased, sizeof( deceased ), payload,
                    counter );
}

void MissingMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&deceased, sizeof( deceased ), payload,
                  counter );
}

std::string MissingMessage::toString( )
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( deceased ) << "]" << std::endl;

 return ss.str( );
}

uint8_t MissingMessage::getDeceased( )
{
 return deceased;
}

HeartbeatMessage::HeartbeatMessage( const uint8_t _ID, const bool _knowGateway,
                                    const uint8_t _prefferedGateWay,
                                    uint8_t _hops )
    : Message( _ID, HEARTBEAT )
    , knowGateway( _knowGateway )
    , prefferedGateWay( _prefferedGateWay )
    , hops( _hops )
{
}

HeartbeatMessage::~HeartbeatMessage( )
{
}

HeartbeatMessage::HeartbeatMessage( const uint8_t *payload )
    : Message( ( uint8_t )payload[0], HEARTBEAT )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                    counter );
 counter += sizeof( knowGateway );
 CopyFromCharArray( ( uint8_t * )&prefferedGateWay, sizeof( prefferedGateWay ),
                    payload, counter );
 counter += sizeof( prefferedGateWay );
 CopyFromCharArray( ( uint8_t * )&hops, sizeof( hops ), payload, counter );
}

void HeartbeatMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                  counter );
 counter += sizeof( knowGateway );
 CopyToCharArray( ( uint8_t * )&prefferedGateWay, sizeof( prefferedGateWay ),
                  payload, counter );
 counter += sizeof( prefferedGateWay );
 CopyToCharArray( ( uint8_t * )&hops, sizeof( hops ), payload, counter );
}

std::string HeartbeatMessage::toString( )
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( knowGateway ) << "]"
    << "HOPS[" << ( int )hops << "]";

 return ss.str( );
}

uint8_t HeartbeatMessage::getPrefferedGateway( )
{
 return prefferedGateWay;
}

bool HeartbeatMessage::getIsGateway( )
{
 return ID == prefferedGateWay;
}
bool HeartbeatMessage::getKnowGateway( )
{
 return knowGateway;
}

GoToLocationMessage::GoToLocationMessage( uint8_t _ID, float _latitude,
                                          float _longitude, int16_t _height )
    : Message( _ID, MOVE_TO_LOCATION )
    , latitude( _latitude )
    , longitude( _longitude )
    , height( _height )
{
}

GoToLocationMessage::GoToLocationMessage( const uint8_t *payload )
    : Message( payload[0], MOVE_TO_LOCATION )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&latitude, sizeof( latitude ), payload,
                    counter );
 counter += sizeof( latitude );
 CopyFromCharArray( ( uint8_t * )&longitude, sizeof( longitude ), payload,
                    counter );
 counter += sizeof( longitude );
 CopyFromCharArray( ( uint8_t * )&height, sizeof( height ), payload, counter );
}

void GoToLocationMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&latitude, sizeof( latitude ), payload,
                  counter );
 counter += sizeof( latitude );
 CopyToCharArray( ( uint8_t * )&longitude, sizeof( longitude ), payload,
                  counter );
 counter += sizeof( longitude );
 CopyToCharArray( ( uint8_t * )&height, sizeof( height ), payload, counter );
}

std::string GoToLocationMessage::toString( )
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "latitude[" << latitude << "] longitude[" << longitude << "] height["
    << height << "]" << std::endl;

 return ss.str( );
}

MovementNegotiationMessage::MovementNegotiationMessage( const uint8_t _ID,
                                                        const float _distance )
    : Message( _ID, MOVEMENT_NEGOTIATION ), distance( _distance )
{
}

MovementNegotiationMessage::~MovementNegotiationMessage( )
{
}

MovementNegotiationMessage::MovementNegotiationMessage( const uint8_t *payload )
    : Message( ( uint8_t )payload[0], MOVEMENT_NEGOTIATION )
{
 int counter = 0;
 counter += sizeof( ID );
 counter += sizeof( type );
 CopyFromCharArray( ( uint8_t * )&distance, sizeof( distance ), payload,
                    counter );
}

void MovementNegotiationMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&distance, sizeof( distance ), payload,
                  counter );
}

std::string MovementNegotiationMessage::toString( )
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( distance ) << "]" << std::endl;

 return ss.str( );
}

float MovementNegotiationMessage::getDistance( )
{
 return distance;
}
}