// Bring in my package's API, which is what I'm testing
#include "../Message.hpp"
// Bring in gtest
#include <gtest/gtest.h>

using namespace Communication::Messages;

TEST( TestMessages, normalMessage )
{
 Message msg( 66, 55, NOTDEFINED, 44, 33 );
 EXPECT_EQ( 66, msg.getCreator( ) );
 EXPECT_EQ( 55, msg.getFrom( ) );
 EXPECT_EQ( NOTDEFINED, msg.getMessageType( ) );
 EXPECT_EQ( 44, msg.getTo( ) );
 EXPECT_EQ( 33, msg.getForward( ) );
}

TEST( TestMessages, normalMessageToPayload )
{
 Message amsg( 66, 55, NOTDEFINED, 44, 33 );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 Message msg( buffer );

 EXPECT_EQ( 66, msg.getCreator( ) );
 EXPECT_EQ( 55, msg.getFrom( ) );
 EXPECT_EQ( NOTDEFINED, msg.getMessageType( ) );
 EXPECT_EQ( 44, msg.getTo( ) );
 EXPECT_EQ( 33, msg.getForward( ) );
}

TEST( TestMessages, locationMessage )
{
 float lat = 2.045475;
 float lon = 4.045407;
 int16_t hei = 100;
 uint32_t time = 23422567;

 LocationMessage msg( 66, 55, 44, 33, lat, lon, hei, time );

 EXPECT_EQ( LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
 EXPECT_EQ( time, msg.gettimeSincePosix( ) );
}

TEST( TestMessages, locationMessageToPayload )
{
 float lat = 2.04573144;
 float lon = 4.04278417;
 int16_t hei = 100;
 uint32_t time = 23422567;

 LocationMessage amsg( 66, 55, 44, 33, lat, lon, hei, time );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 LocationMessage msg( buffer );

 EXPECT_EQ( LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
 EXPECT_EQ( time, msg.gettimeSincePosix( ) );
}

TEST( TestMessages, locationHonoluluMessageToPayload )
{
 float lat = 21.507462346;
 float lon = -158.08319242;
 int16_t hei = 674;
 uint32_t time = 1557433731;

 LocationMessage amsg( 66, 55, 44, 33, lat, lon, hei, time );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 LocationMessage msg( buffer );

 EXPECT_EQ( LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
 EXPECT_EQ( time, msg.gettimeSincePosix( ) );
}

TEST( TestMessages, locationFijiMessageToPayload )
{
 float lat = -17.828230;
 float lon = 178.092686;
 int16_t hei = 674;
 uint32_t time = 1557433543;

 LocationMessage amsg( 66, 55, 44, 33, lat, lon, hei, time );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 LocationMessage msg( buffer );

 EXPECT_EQ( LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
 EXPECT_EQ( time, msg.gettimeSincePosix( ) );
}

TEST( TestMessages, locationLelystadAirportMessageToPayload )
{
 float lat = 52.455702;
 float lon = 5.519210;
 int16_t hei = -5;
 uint32_t time = 1557435698;

 LocationMessage amsg( 66, 55, 44, 33, lat, lon, hei, time );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 LocationMessage msg( buffer );

 EXPECT_EQ( LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
 EXPECT_EQ( time, msg.gettimeSincePosix( ) );
}

TEST( TestMessages, IntroduceMessage )
{
 uint8_t hopsUntilGateway = 100;
 bool knowGateway = true;

 IntroduceMessage msg( 66, 55, 44, 33, hopsUntilGateway, knowGateway );

 EXPECT_EQ( PRESENT, msg.getMessageType( ) );
 EXPECT_EQ( hopsUntilGateway, msg.getHopsUntilGateway( ) );
 EXPECT_EQ( knowGateway, msg.getKnowGateway( ) );
}

TEST( TestMessages, IntroduceMessageToPayload )
{
 uint8_t hopsUntilGateway = 100;
 bool knowGateway = true;

 IntroduceMessage amsg( 66, 55, 44, 33, hopsUntilGateway, knowGateway );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 IntroduceMessage msg( buffer );

 EXPECT_EQ( PRESENT, msg.getMessageType( ) );
 EXPECT_EQ( hopsUntilGateway, msg.getHopsUntilGateway( ) );
 EXPECT_EQ( knowGateway, msg.getKnowGateway( ) );
}

TEST( TestMessages, HeartbeatMessage )
{
 uint8_t hops = 100;
 bool knowGateway = true;
 uint8_t prefferedGateWay = 42;

 HeartbeatMessage msg( 66, 55, 44, 33, knowGateway, prefferedGateWay, hops );

 EXPECT_EQ( HEARTBEAT, msg.getMessageType( ) );
 EXPECT_EQ( knowGateway, msg.getKnowGateway( ) );
 EXPECT_EQ( hops, msg.getHops( ) );
}

TEST( TestMessages, HeartbeatMessageToPayload )
{
 uint8_t hops = 100;
 bool knowGateway = true;
 uint8_t prefferedGateWay = 42;

 HeartbeatMessage amsg( 66, 55, 44, 33, knowGateway, prefferedGateWay, hops );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 HeartbeatMessage msg( buffer );

 EXPECT_EQ( HEARTBEAT, msg.getMessageType( ) );
 EXPECT_EQ( knowGateway, msg.getKnowGateway( ) );
 EXPECT_EQ( hops, msg.getHops( ) );
}

TEST( TestMessages, isGateway )
{
 uint8_t hops = 100;
 bool knowGateway = true;
 uint8_t prefferedGateWay = 42;

 HeartbeatMessage msg( 42, 55, 44, 33, knowGateway, prefferedGateWay, hops );

 EXPECT_TRUE( msg.getIsGateway( ) );
}

TEST( TestMessages, isNoGateway )
{
 uint8_t hops = 100;
 bool knowGateway = true;
 uint8_t prefferedGateWay = 66;

 HeartbeatMessage msg( 42, 55, 44, 33, knowGateway, prefferedGateWay, hops );

 EXPECT_FALSE( msg.getIsGateway( ) );
}

TEST( TestMessages, makeHop )
{
 uint8_t hops = 100;
 bool knowGateway = true;
 uint8_t prefferedGateWay = 66;

 HeartbeatMessage msg( 42, 55, 44, 33, knowGateway, prefferedGateWay, hops );
 msg.makeHop( );

 EXPECT_EQ( 101, msg.getHops( ) );
}

TEST( TestMessages, MissingMessage )
{
 uint8_t missing = 100;

 MissingMessage msg( 66, 55, 44, 33, missing );

 EXPECT_EQ( MISSING, msg.getMessageType( ) );
 EXPECT_EQ( missing, msg.getMissing( ) );
}

TEST( TestMessages, MissingMessageToPayload )
{
 uint8_t missing = 100;

 MissingMessage amsg( 66, 55, 44, 33, missing );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 MissingMessage msg( buffer );

 EXPECT_EQ( MISSING, msg.getMessageType( ) );
 EXPECT_EQ( missing, msg.getMissing( ) );
}

TEST( TestMessages, GoToLocationMessage )
{
 float lat = 2.045475;
 float lon = 4.045407;
 int16_t hei = 100;
 uint32_t time = 23422567;

 GoToLocationMessage msg( 66, 55, 44, 33, lat, lon, hei );

 EXPECT_EQ( MOVE_TO_LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
}

TEST( TestMessages, GoToLocationMessageToPayload )
{
 float lat = 2.04573144;
 float lon = 4.04278417;
 int16_t hei = 100;
 uint32_t time = 23422567;

 GoToLocationMessage amsg( 66, 55, 44, 33, lat, lon, hei );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 GoToLocationMessage msg( buffer );

 EXPECT_EQ( MOVE_TO_LOCATION, msg.getMessageType( ) );
 EXPECT_EQ( lat, msg.getLatitude( ) );
 EXPECT_EQ( lon, msg.getLongitude( ) );
 EXPECT_EQ( hei, msg.getHeight( ) );
}

TEST( TestMessages, MovementNegotiationMessage )
{
 float cost = 100;

 MovementNegotiationMessage msg( 66, 55, 44, 33, cost );

 EXPECT_EQ( MOVEMENT_NEGOTIATION, msg.getMessageType( ) );
 EXPECT_EQ( cost, msg.getCost( ) );
}

TEST( TestMessages, MovementNegotiationMessageToPayload )
{
 float cost = 100;

 MovementNegotiationMessage amsg( 66, 55, 44, 33, cost );
 uint8_t buffer[MAX_PAYLOAD];
 amsg.toPayload( buffer );

 MovementNegotiationMessage msg( buffer );

 EXPECT_EQ( MOVEMENT_NEGOTIATION, msg.getMessageType( ) );
 EXPECT_EQ( cost, msg.getCost( ) );
}

// Run all the tests that were declared with TEST()
int main( int argc, char** argv )
{
 testing::InitGoogleTest( &argc, argv );
 return RUN_ALL_TESTS( );
}