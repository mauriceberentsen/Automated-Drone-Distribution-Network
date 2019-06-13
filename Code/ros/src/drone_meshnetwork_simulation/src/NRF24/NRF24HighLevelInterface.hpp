/**
 * @file HighLevelInterface.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief NRF24 HighLevelInterface.hpp
 * @version 1.0
 * @date 2019-05-21
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef NRF24HIGHLEVELINTERFACE
#define NRF24HIGHLEVELINTERFACE

#include "../Communication/Wireless/IMeshDebugInfo.hpp"
#include "../Communication/Wireless/IMeshNetwork.hpp"
#include "../Communication/Wireless/IWirelessCommunication.hpp"

class NRF24LowLevelInterface;

class NRF24HighLevelInterface
    : public Communication::Wireless::IWirelessCommunication
{
public:
 NRF24HighLevelInterface( /* args */ );
 ~NRF24HighLevelInterface( );
 /**
  * @brief Start the communication pipe of the NRF24
  *
  * @param MC Reference to the connected MeshnetworkComponent
  */
 void StartAntenna( Communication::Wireless::IMeshNetwork* IMN );
 /**
  * @brief Stop the Antenna
  *
  */
 void StopAntenna( );
 /**
  * @brief Send a message
  *
  * @param msg Pointer to the message to be sent
  * @return true Sending was succesfull
  * @return false Sending failed
  */
 bool SendMessageTo( const uint8_t* msg );
 /**
  * @brief Broadcast a message to all nodes in range
  *
  * @param msg message to be sent
  */
 void BroadcastMessage( const uint8_t* msg );
 /**
  * @brief debuging activation
  *
  * @param debug Pointer to the debug interface of the meshnetworkComponent
  * @param on state
  */
 void DebugingMode( Communication::Wireless::IMeshDebugInfo* debug,
                    const bool on = true );
 /**
  * @brief Get ON/OFF state
  *
  * @return true ON
  * @return false OFF
  */
 const bool On( );

 void OnMessage( const uint8_t* message );

private:
 const uint64_t addressRange = 0x544d526800LL;
 const uint8_t ackStart = 120;
 NRF24LowLevelInterface* lowLevelInterface;
 bool on;
 /// \brief The connected Meshnetwork Interface used for messages
 Communication::Wireless::IMeshNetwork* meshnetworkComponent;
 /// \brief Interface providing debug info about the meshnetwork
 Communication::Wireless::IMeshDebugInfo* debuginfo;
};

#endif  // NRF24HIGHLEVELINTERFACE
