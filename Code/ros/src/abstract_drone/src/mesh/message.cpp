#include "message.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>

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
 counter += sizeof( type );
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

locationMessage::locationMessage( uint8_t _ID, float _latitude,
                                  float _longitude, int16_t _height,
                                  uint32_t _timeSincePosix )
    : Message( _ID, LOCATION )
    , latitude( _latitude )
    , longitude( _longitude )
    , height( _height )
    , timeSincePosix( _timeSincePosix )
{
}

locationMessage::locationMessage( const uint8_t *payload )
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

locationMessage::~locationMessage( )
{
}

void locationMessage::toPayload( uint8_t *payload )
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

std::string locationMessage::toString( )
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "latitude[" << latitude << "] longitude[" << longitude << "] height["
    << height << "] timeSincePosix[" << timeSincePosix << "]" << std::endl;

 return ss.str( );
}

void GiveIDMessage::toPayload( uint8_t *payload )
{
 int counter = 0;
 CopyToCharArray( ( uint8_t * )&ID, sizeof( ID ), payload, counter );
 counter += sizeof( ID );
 CopyToCharArray( ( uint8_t * )&type, sizeof( type ), payload, counter );
 counter += sizeof( type );
 CopyToCharArray( ( uint8_t * )&GiveID, sizeof( GiveID ), payload, counter );
}

std::string GiveIDMessage::toString( )
{
 std::stringstream ss;
 ss << std::setprecision( 8 ) << std::dec << "ID[" << ( int )ID << "] Type["
    << ( int )type << "]" << std::endl
    << "Give[" << GiveID << "]" << std::endl;
 return ss.str( );
}

IntroduceMessage::IntroduceMessage( const uint8_t _ID,
                                    const uint8_t _hopsUntilsGateway )
    : Message( _ID, PRESENT ), hopsUntilsGateway( _hopsUntilsGateway )
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