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

#include "NRF24LowLevelInterface.hpp"

class NRF24HighLevelInterface
{
public:
    NRF24HighLevelInterface(/* args */);
    ~NRF24HighLevelInterface();
  void StartAntenna( uint8_t nodeID/*Communication::Wireless::IMeshNetwork* IMN*/ );
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
   * @brief Set debugging mode
   *
   * @param IMD Pointer to the debug interface of a meshcomponent
   * @param on on/off state
   */
  /**
   * @brief Get ON/OFF state
   *
   * @return true ON
   * @return false OFF
   */
  const bool On( );
private:
    const uint64_t addressRange = 0x544d526800LL;
    NRF24LowLevelInterface low;
    bool on;
};




#endif //NRF24HIGHLEVELINTERFACE
