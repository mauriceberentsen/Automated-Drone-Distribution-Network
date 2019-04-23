/**
 * @file message.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for all Messages
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef MESSAGEHPP
#define MESSAGEHPP

#include <inttypes.h>
#include <string>
namespace Messages
{
/**
 * @brief Used for reconizing message types when parsed into a 32 byte array
 *
 */
enum Messagetype : uint8_t {
 NOTDEFINED = 0,        // 0
 LOCATION,              // 1
 REQUESTLOCATION,       // 2
 PRESENT,               // 3
 HEARTBEAT,             // 4
 MISSING,               // 5
 MOVE_TO_LOCATION,      // 6
 MOVEMENT_NEGOTIATION,  // 7
};

class Message
{
public:
 Message( const uint8_t _ID, Messagetype _Messagetype );
 explicit Message( const uint8_t *payload );
 ~Message( );
 /**
  * @brief Copies the the content of a message to the given payload
  *
  * @param payload array that will be filled with the message
  */
 virtual void toPayload( uint8_t *payload ) const;
 /**
  * @brief contruct a descriptive string of a message
  *
  * @return std::string string of the message object
  */
 virtual const std::string toString( ) const;

 const uint8_t getID( ) const;

protected:
 /**
  * @brief Copies the content of an array back to a value
  *
  * @param value to copy
  * @param size of the value
  * @param arr array to copy from
  * @param start place in the array to start
  */
 void CopyFromCharArray( uint8_t *value, uint16_t size, const uint8_t *arr,
                         uint16_t start );
 /**
  * @brief Copies the value to a char array
  *
  * @param value to copy to a char array
  * @param size of the value
  * @param arr array to copy to
  * @param start place in the array to begin copying
  */
 void CopyToCharArray( uint8_t *value, uint16_t size, uint8_t *arr,
                       uint16_t start ) const;
 /// \brief The ID of the node
 uint8_t ID;
 /// \brief The type of the message
 uint8_t type;
};

class LocationMessage : public Message
{
public:
 /**
  * @brief Construct a new Location Message object
  *        Location messages are used to inform others about your location
  * @param _ID The ID  of this node
  * @param latitude The latitude of this node
  * @param longitude The longitude of this node
  * @param height The height of this node
  * @param timeSincePosix The moment in time this was measured
  */
 LocationMessage( uint8_t _ID, float latitude, float longitude, int16_t height,
                  uint32_t timeSincePosix );
 /**
  * @brief Construct a new Location Message object from a NRF24 payload
  *
  * @param payload An Array that holds all variables of this message
  */
 explicit LocationMessage( const uint8_t *payload );
 ~LocationMessage( );
 const std::string toString( ) const;
 void toPayload( uint8_t *payload ) const;
 const float getLatitude( ) const;
 const float getLongitude( ) const;
 const int16_t getHeight( ) const;
 const int16_t gettimeSincePosix( ) const;

private:
 /// \brief The latitude of a location
 float latitude;
 /// \brief The longitude of location
 float longitude;
 /// \brief The height if a location
 int16_t height;
 /// \brief The time this location was measured
 uint32_t timeSincePosix;
};

class IntroduceMessage : public Message
{
public:
 /**
  * @brief Construct a new Introduce Message object
  *        This message is used to introduce yourself to others.
  *        It tells others who you are, if you are connected to the gateway.
  *        And how many hops it take you to talk with the gateway.
  *
  * @param _ID of this Node
  * @param _hopsUntilsGateway How many hops it takes you to get to the gateway
  * @param _knowGateway If you are connected to a gateway
  */
 IntroduceMessage( const uint8_t _ID, const uint8_t _hopsUntilsGateway,
                   const bool _knowGateway );
 /**
  * @brief Construct a new Introduce Message objectfrom a NRF24 payload
  *
  * @param payload An Array that holds all variables of this message
  */
 explicit IntroduceMessage( const uint8_t *payload );
 ~IntroduceMessage( );
 const std::string toString( ) const;
 void toPayload( uint8_t *payload ) const;

 const bool getKnowGateway( ) const;
 const uint8_t getHopsUntilGateway( ) const;

private:
 /// \brief The amount of hops a node is away from a gateway
 uint8_t hopsUntilGateway;
 /// \brief Boolean true if the node is connected to a gateway
 bool knowGateway;
};

class HeartbeatMessage : public Message
{
public:
 /**
  * @brief Construct a new Heartbeat Message object
  *        This message is used to enssure connectivity.
  *        It tells others who you are, if you are connected to the gateway.
  *        Who your preffere.dGateWay is and how many hops it take you to talk
  *         with the gateway. If this meesage is forwared it makes a hop.
  *
  * @param _ID of this Node.
  * @param _knowGateway If you are connected to a gateway.
  * @param _prefferedGateWay The gateway this node communicates with.
  * @param _hops How many hops it takes you to get to the gateway.
  */
 HeartbeatMessage( const uint8_t _ID, const bool _knowGateway,
                   const uint8_t _prefferedGateWay, uint8_t _hops = 0 );
 /**
  * @brief Construct a new Heartbeat Message objectfrom a NRF24 payload.
  *
  * @param payload An Array that holds all variables of this message.
  */
 explicit HeartbeatMessage( const uint8_t *payload );
 ~HeartbeatMessage( );
 const std::string toString( ) const;
 void toPayload( uint8_t *payload ) const;
 /**
  * @brief Adds a hop to the message.
  *
  */
 void makeHop( );
 /**
  * @brief Compare the ID with the prefferedGateWay. If it's equal the node is a
  * gateway
  *
  * @return true Is a Gateway
  * @return false Is a Node
  */
 const bool getIsGateway( ) const;

 const bool getKnowGateway( ) const;
 const uint8_t getPrefferedGateway( ) const;
 const uint8_t getHops( ) const;

private:
 /// \brief The amount of hops this heartbeat has taken
 uint8_t hops;
 /// \brief Boolean true if the node is connected to a gateway
 bool knowGateway;
 /// \brief The ID of prefferedGateWay
 uint8_t prefferedGateWay;
};

class MissingMessage : public Message
{
public:
 /**
  * @brief Construct a new Missing Message object
  *         Used to inform others about a missing Node in the network
  *
  * @param _ID The ID of the current Node.
  * @param _missing The ID of the Node that went missing.
  */
 MissingMessage( const uint8_t _ID, const uint8_t _missing );
 /**
  * @brief Construct a new Missing Message objectfrom a NRF24 payload
  *
  * @param payload An Array that holds all variables of this message
  */
 explicit MissingMessage( const uint8_t *payload );

 ~MissingMessage( );
 void toPayload( uint8_t *payload ) const;
 const std::string toString( ) const;
 const uint8_t getMissing( ) const;

private:
 /// \brief The ID of the Node that went missing
 uint8_t missing;
};

class GoToLocationMessage : public Message
{
public:
 /**
  * @brief Construct a new Go To Location Message object
  *        Used to tell other to go to a specific location
  *
  * @param _ID The ID of the current Node
  * @param latitude The target latitude to go to
  * @param longitude The target longitude to go to
  * @param height The target height to go to
  */
 GoToLocationMessage( uint8_t _ID, float latitude, float longitude,
                      int16_t height );
 /**
  * @brief Construct a new Go To Location Message objectfrom a NRF24 payload
  *
  * @param payload An Array that holds all variables of this message
  */
 explicit GoToLocationMessage( const uint8_t *payload );
 ~GoToLocationMessage( ){};
 const std::string toString( ) const;
 void toPayload( uint8_t *payload ) const;
 const float getLatitude( ) const;
 const float getLongitude( ) const;
 const int16_t getHeight( ) const;

private:
 /// \brief The latitude of a location
 float latitude;
 /// \brief The longitude of location
 float longitude;
 /// \brief The height if a location
 int16_t height;
};

class MovementNegotiationMessage : public Message
{
public:
 /**
  * @brief Construct a new Movement Negotiation Message object
  *        Used to negotiate with others about who should move.
  *
  * @param _ID the ID of the current Node
  * @param _cost the calculated cost.
  */
 MovementNegotiationMessage( const uint8_t _ID, const float _cost );
 /**
  * @brief Construct a new Movement Negotiation Message objectfrom a NRF24
  * payload
  *
  * @param payload An Array that holds all variables of this message
  */
 explicit MovementNegotiationMessage( const uint8_t *payload );
 ~MovementNegotiationMessage( );
 const std::string toString( ) const;
 void toPayload( uint8_t *payload ) const;
 const float getCost( ) const;

private:
 /// \brief The cost for this node for the movement
 float cost;
};
}  // namespace Messages
#endif  // MESSAGEHPP