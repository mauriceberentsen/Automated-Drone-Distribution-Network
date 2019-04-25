/**
 * @file message.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for all messages
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <sstream>
#include <iomanip>
#include <iostream>

#include "message.hpp"

namespace Messages
{
Message::Message( const uint8_t _creator, const uint8_t _from,
                  Messagetype _Messagetype, const uint8_t _to,
                  const uint8_t _forward )
    : creator( _creator )
    , from( _from )
    , type( _Messagetype )
    , to( _to )
    , forward( _forward )
{
}
Message::Message( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], ( Messagetype )payload[TYPE],
               payload[TO], payload[FORWARD] )
{
}
Message::~Message( )
{
}

const std::string Message::toString( ) const
{
 std::stringstream ss;
 ss << "creator[" << ( int )creator << "] Type[" << ( int )type << "]"
    << std::endl;

 return ss.str( );
}

void Message::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
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
 return this->creator;
}

LocationMessage::LocationMessage( const uint8_t _creator, const uint8_t _from,
                                  const uint8_t _to, const uint8_t _forward,
                                  float _latitude, float _longitude,
                                  int16_t _height, uint32_t _timeSincePosix )
    : Message( _creator, _from, LOCATION, _to, _forward )
    , latitude( _latitude )
    , longitude( _longitude )
    , height( _height )
    , timeSincePosix( _timeSincePosix )
{
}

LocationMessage::LocationMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], LOCATION, payload[TO],
               payload[FORWARD] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
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
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );
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
 ss << std::setprecision( 8 ) << std::dec << "creator[" << ( int )creator
    << "] Type[" << ( int )type << "]" << std::endl
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

IntroduceMessage::IntroduceMessage( const uint8_t _creator, const uint8_t _from,
                                    const uint8_t _to, const uint8_t _forward,
                                    const uint8_t _hopsUntilGateway,
                                    const bool _knowGateway )
    : Message( _creator, _from, PRESENT, _to, _forward )
    , hopsUntilGateway( _hopsUntilGateway )
    , knowGateway( _knowGateway )
{
}

IntroduceMessage::~IntroduceMessage( )
{
}

IntroduceMessage::IntroduceMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], PRESENT, payload[2],
               payload[3] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
 CopyFromCharArray( ( uint8_t * )&hopsUntilGateway, sizeof( hopsUntilGateway ),
                    payload, counter );
 counter += sizeof( hopsUntilGateway );
 CopyFromCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                    counter );
}

void IntroduceMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );
 CopyToCharArray( ( uint8_t * )&hopsUntilGateway, sizeof( hopsUntilGateway ),
                  payload, counter );
 counter += sizeof( hopsUntilGateway );
 CopyToCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                  counter );
}

const std::string IntroduceMessage::toString( ) const
{
 std::stringstream ss;
 ss << "creator[" << std::to_string( creator ) << "] Ty["
    << std::to_string( type ) << "]To[" << std::to_string( to ) << "]for["
    << std::to_string( forward ) << "]hopsUG["
    << std::to_string( hopsUntilGateway ) << "]KG["
    << std::to_string( knowGateway ) << std::endl;

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

MissingMessage::MissingMessage( const uint8_t _creator, const uint8_t _from,
                                const uint8_t _to, const uint8_t _forward,
                                const uint8_t _missing )
    : Message( _creator, _from, MISSING, _to, _forward ), missing( _missing )
{
}

MissingMessage::~MissingMessage( )
{
}

MissingMessage::MissingMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], MISSING, payload[TO],
               payload[FORWARD] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
 CopyFromCharArray( ( uint8_t * )&missing, sizeof( missing ), payload,
                    counter );
}

void MissingMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );
 CopyToCharArray( ( uint8_t * )&missing, sizeof( missing ), payload, counter );
}

const std::string MissingMessage::toString( ) const
{
 std::stringstream ss;
 ss << "creator[" << std::to_string( creator ) << "] Type["
    << std::to_string( type ) << "]"
    << "knowGateway[" << std::to_string( missing ) << "]" << std::endl;

 return ss.str( );
}

const uint8_t MissingMessage::getMissing( ) const
{
 return this->missing;
}

HeartbeatMessage::HeartbeatMessage( const uint8_t _creator, const uint8_t _from,
                                    const uint8_t _to, const uint8_t _forward,
                                    const bool _knowGateway,
                                    const uint8_t _prefferedGateWay,
                                    uint8_t _hops )
    : Message( _creator, _from, HEARTBEAT, _to, _forward )
    , knowGateway( _knowGateway )
    , prefferedGateWay( _prefferedGateWay )
    , hops( _hops )
{
}

HeartbeatMessage::~HeartbeatMessage( )
{
}

HeartbeatMessage::HeartbeatMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], HEARTBEAT, payload[TO],
               payload[FORWARD] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
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
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );
 CopyToCharArray( ( uint8_t * )&knowGateway, sizeof( knowGateway ), payload,
                  counter );
 counter += sizeof( uint8_t );
 CopyToCharArray( ( uint8_t * )&prefferedGateWay, sizeof( prefferedGateWay ),
                  payload, counter );
 counter += sizeof( prefferedGateWay );
 CopyToCharArray( ( uint8_t * )&hops, sizeof( hops ), payload, counter );
}

const std::string HeartbeatMessage::toString( ) const
{
 std::stringstream ss;
 ss << "creator[" << std::to_string( creator ) << "] Ty["
    << std::to_string( type ) << "]To[" << std::to_string( to ) << "]for["
    << std::to_string( forward ) << "]KG[" << std::to_string( knowGateway )
    << "]PG[" << std::to_string( prefferedGateWay ) << "]hops["
    << std::to_string( hops ) << "]" << std::endl;

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
 return creator == prefferedGateWay;
}
const bool HeartbeatMessage::getKnowGateway( ) const
{
 return this->knowGateway;
}

GoToLocationMessage::GoToLocationMessage(
    const uint8_t _creator, const uint8_t _from, const uint8_t _to,
    const uint8_t _forward, float _latitude, float _longitude, int16_t _height )
    : Message( _creator, _from, MOVE_TO_LOCATION, _to, _forward )
    , latitude( _latitude )
    , longitude( _longitude )
    , height( _height )
{
}

GoToLocationMessage::GoToLocationMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], MOVE_TO_LOCATION, payload[TO],
               payload[FORWARD] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
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
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );

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
 ss << std::setprecision( 8 ) << std::dec << "creator[" << ( int )creator
    << "] Type[" << ( int )type << "]" << std::endl
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

MovementNegotiationMessage::MovementNegotiationMessage( const uint8_t _creator,
                                                        const uint8_t _from,
                                                        const uint8_t _to,
                                                        const uint8_t _forward,
                                                        const float _cost )
    : Message( _creator, _from, MOVEMENT_NEGOTIATION, _to, _forward )
    , cost( _cost )
{
}

MovementNegotiationMessage::~MovementNegotiationMessage( )
{
}

MovementNegotiationMessage::MovementNegotiationMessage( const uint8_t *payload )
    : Message( payload[CREATOR], payload[FROM], MOVEMENT_NEGOTIATION,
               payload[TO], payload[FORWARD] )
{
 int counter = 0;
 counter += sizeof( creator );
 counter += sizeof( from );
 counter += sizeof( type );
 counter += sizeof( to );
 counter += sizeof( forward );
 CopyFromCharArray( ( uint8_t * )&cost, sizeof( cost ), payload, counter );
}

void MovementNegotiationMessage::toPayload( uint8_t *payload ) const
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&creator, sizeof( creator ), payload, counter );
 counter += sizeof( creator );
 CopyToCharArray( ( uint8_t * )&from, sizeof( from ), payload, counter );
 counter += sizeof( from );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&to, sizeof( to ), payload, counter );
 counter += sizeof( to );
 CopyToCharArray( ( uint8_t * )&forward, sizeof( forward ), payload, counter );
 counter += sizeof( forward );
 CopyToCharArray( ( uint8_t * )&cost, sizeof( cost ), payload, counter );
}

const std::string MovementNegotiationMessage::toString( ) const
{
 std::stringstream ss;
 ss << "creator[" << std::to_string( creator ) << "] Type["
    << std::to_string( type ) << "]"
    << "knowGateway[" << std::to_string( cost ) << "]" << std::endl;

 return ss.str( );
}

const float MovementNegotiationMessage::getCost( ) const
{
 return this->cost;
}
}  // namespace Messages