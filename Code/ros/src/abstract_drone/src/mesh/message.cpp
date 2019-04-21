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

const std::string Message::toString( ) const
{
 std::stringstream ss;
 ss << "ID[" << ( int )ID << "] Type[" << ( int )type << "]" << std::endl;

 return ss.str( );
}

void Message::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
}

void Message::CopyToCharArray( uint8_t *value, uint16_t size, uint8_t *arr,
                               uint16_t start ) const
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

const uint8_t Message::getID( ) const
{
 return this->ID;
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

void LocationMessage::toPayload( uint8_t *payload ) const
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

const std::string LocationMessage::toString( ) const
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "latitude[" << latitude << "] longitude[" << longitude << "] height["
    << height << "] timeSincePosix[" << timeSincePosix << "]" << std::endl;

 return ss.str( );
}

const float LocationMessage::getLatitude( ) const
{
 return this->latitude;
}
const float LocationMessage::getLongitude( ) const
{
 return this->longitude;
}
const int16_t LocationMessage::getHeight( ) const
{
 return this->height;
}
const int16_t LocationMessage::gettimeSincePosix( ) const
{
 return this->timeSincePosix;
}

IntroduceMessage::IntroduceMessage( const uint8_t _ID,
                                    const uint8_t _hopsUntilGateway,
                                    const bool _knowGateway )
    : Message( _ID, PRESENT )
    , hopsUntilGateway( _hopsUntilGateway )
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
 CopyFromCharArray( ( uint8_t * )&hopsUntilGateway, sizeof( hopsUntilGateway ),
                    payload, counter );
 counter += sizeof( hopsUntilGateway );
 CopyFromCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                    counter );
}

void IntroduceMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&hopsUntilGateway, sizeof( hopsUntilGateway ),
                  payload, counter );
 counter += sizeof( hopsUntilGateway );
 CopyToCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                  counter );
}

const std::string IntroduceMessage::toString( ) const
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( hopsUntilGateway ) << "]" << std::endl;

 return ss.str( );
}

const uint8_t IntroduceMessage::getHopsUntilGateway( ) const
{
 return this->hopsUntilGateway;
}

const bool IntroduceMessage::getKnowGateway( ) const
{
 return this->knowGateway;
}

MissingMessage::MissingMessage( const uint8_t _ID, const uint8_t _missing )
    : Message( _ID, MISSING ), missing( _missing )
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
 CopyFromCharArray( ( uint8_t * )&missing, sizeof( missing ), payload,
                    counter );
}

void MissingMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&missing, sizeof( missing ), payload, counter );
}

const std::string MissingMessage::toString( ) const
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( missing ) << "]" << std::endl;

 return ss.str( );
}

const uint8_t MissingMessage::getMissing( ) const
{
 return this->missing;
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

void HeartbeatMessage::toPayload( uint8_t *payload ) const
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

const std::string HeartbeatMessage::toString( ) const
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( knowGateway ) << "]"
    << "HOPS[" << ( int )hops << "]";

 return ss.str( );
}

const uint8_t HeartbeatMessage::getPrefferedGateway( ) const
{
 return prefferedGateWay;
}

void HeartbeatMessage::makeHop( )
{
 ++hops;
}

const uint8_t HeartbeatMessage::getHops( ) const
{
 return this->hops;
}

const bool HeartbeatMessage::getIsGateway( ) const
{
 return ID == prefferedGateWay;
}
const bool HeartbeatMessage::getKnowGateway( ) const
{
 return this->knowGateway;
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

void GoToLocationMessage::toPayload( uint8_t *payload ) const
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

const std::string GoToLocationMessage::toString( ) const
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "latitude[" << latitude << "] longitude[" << longitude << "] height["
    << height << "]" << std::endl;

 return ss.str( );
}

const float GoToLocationMessage::getLatitude( ) const
{
 return this->latitude;
}

const float GoToLocationMessage::getLongitude( ) const
{
 return this->longitude;
}

const int16_t GoToLocationMessage::getHeight( ) const
{
 return this->height;
}

MovementNegotiationMessage::MovementNegotiationMessage( const uint8_t _ID,
                                                        const float _distance )
    : Message( _ID, MOVEMENT_NEGOTIATION ), cost( _distance )
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
 CopyFromCharArray( ( uint8_t * )&cost, sizeof( cost ), payload, counter );
}

void MovementNegotiationMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&cost, sizeof( cost ), payload, counter );
}

const std::string MovementNegotiationMessage::toString( ) const
{
 std::stringstream ss;
 ss << "ID[" << std::to_string( ID ) << "] Type[" << std::to_string( type )
    << "]"
    << "knowGateway[" << std::to_string( cost ) << "]" << std::endl;

 return ss.str( );
}

const float MovementNegotiationMessage::getCost( ) const
{
 return this->cost;
}
}  // namespace Messages