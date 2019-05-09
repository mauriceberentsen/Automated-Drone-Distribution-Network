/**
 * @file Message.hpp
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
namespace Communication
{
namespace Messages
{
 /**
  * @brief Used for recognizing message types when parsed into a 32 byte array
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
 /**
  * @brief Used for helping with the place in a 32 byte array
  *
  */
 enum MessageHelper : uint8_t {
  CREATOR = 0,      // 0
  FROM = 1,         // 1
  TYPE = 2,         // 2
  TO = 3,           // 3
  FORWARD = 4,      // 4
  MAX_PAYLOAD = 32  // 32
 };

 class Message
 {
 public:
  /**
   * @brief Construct a new Message object
   *
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param messagetype The type of message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   */
  Message( const uint8_t creator, const uint8_t from, Messagetype messagetype,
           const uint8_t to, const uint8_t forward );
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

  const uint8_t getCreator( ) const;
  const uint8_t getFrom( ) const;
  const Messagetype getMessageType( ) const;
  const uint8_t getTo( ) const;
  const uint8_t getForward( ) const;

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
  /// \brief The creator of the message
  uint8_t creator;
  /// \brief The sender of the message
  uint8_t from;
  /// \brief The type of the message
  Messagetype type;
  /// \brief send this message to
  uint8_t to;
  /// \brief forward this message to
  uint8_t forward;
 };

 class LocationMessage : public Message
 {
 public:
  /**
   * @brief Construct a new Location Message object
   *        Location messages are used to inform others about your location
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param latitude The latitude of this node
   * @param longitude The longitude of this node
   * @param height The height of this node
   * @param timeSincePosix The moment in time this was measured
   */
  LocationMessage( const uint8_t creator, const uint8_t from, const uint8_t to,
                   const uint8_t forward, float latitude, float longitude,
                   int16_t height, uint32_t timeSincePosix );
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
  const int32_t gettimeSincePosix( ) const;

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
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param hopsUntilsGateway How many hops it takes you to get to the gateway
   * @param knowGateway If you are connected to a gateway
   */
  IntroduceMessage( const uint8_t creator, const uint8_t from, const uint8_t to,
                    const uint8_t forward, const uint8_t hopsUntilsGateway,
                    const bool knowGateway );
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
   *        Who your prefferedGateWay is and how many hops it take you to talk
   *         with the gateway. If this message is forwared it makes a hop.
   *
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param knowGateway If you are connected to a gateway.
   * @param prefferedGateWay The gateway this node communicates with.
   * @param hops How many hops it takes you to get to the gateway.
   */
  HeartbeatMessage( const uint8_t creator, const uint8_t from, const uint8_t to,
                    const uint8_t forward, const bool knowGateway,
                    const uint8_t prefferedGateWay, uint8_t hops = 0 );
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
   * @brief Compare the ID with the prefferedGateWay. If it's equal the node is
   * a gateway
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
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param missing The ID of the Node that went missing.
   */
  MissingMessage( const uint8_t creator, const uint8_t from, const uint8_t to,
                  const uint8_t forward, const uint8_t missing );
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
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param latitude The target latitude to go to
   * @param longitude The target longitude to go to
   * @param height The target height to go to
   */
  GoToLocationMessage( const uint8_t creator, const uint8_t from,
                       const uint8_t to, const uint8_t forward, float latitude,
                       float longitude, int16_t height );
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
   * @param creator The ID  of the creator node
   * @param from Who is sending this message
   * @param to To who this message will be send
   * @param forward To who this messages is destined
   * @param cost the calculated cost.
   */
  MovementNegotiationMessage( const uint8_t creator, const uint8_t from,
                              const uint8_t to, const uint8_t forward,
                              const float cost );
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
}  // namespace Communication
#endif  // MESSAGEHPP