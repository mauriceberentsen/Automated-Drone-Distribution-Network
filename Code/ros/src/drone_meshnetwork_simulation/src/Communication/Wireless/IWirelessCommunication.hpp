/**
 * @file IWirelessCommunication.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the interface IWirelessCommunication
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IWIRELESS
#define IWIRELESS
#include "../Meshnetwork/IMeshDebugInfo.hpp"
#include "IMeshNetwork.hpp"
namespace Communication
{
namespace Wireless
{
 class IWirelessCommunication
 {
 public:
  virtual ~IWirelessCommunication( ){};

  /**
   * @brief Start the Antenna
   *
   * @param IMN Pointer to the interface of a meshcomponent
   */
  virtual void StartAntenna( Communication::Wireless::IMeshNetwork* IMN ) = 0;
  /**
   * @brief Stop the Antenna
   *
   */
  virtual void StopAntenna( ) = 0;
  /**
   * @brief Send a message
   *
   * @param msg Pointer to the message to be sent
   * @return true Sending was succesfull
   * @return false Sending failed
   */
  virtual bool SendMessageTo( const uint8_t* msg ) = 0;
  /**
   * @brief Broadcast a message to all nodes in range
   *
   * @param msg message to be sent
   */
  virtual void BroadcastMessage( const uint8_t* msg ) = 0;
  /**
   * @brief Set debugging mode
   *
   * @param IMD Pointer to the debug interface of a meshcomponent
   * @param on on/off state
   */
  virtual void DebugingMode( Communication::Meshnetwork::IMeshDebugInfo* IMD,
                             const bool on ) = 0;
  /**
   * @brief Get ON/OFF state
   *
   * @return true ON
   * @return false OFF
   */
  virtual const bool On( ) = 0;
 };
}  // namespace Wireless
}  // namespace Communication
#endif  // WIRELESS
