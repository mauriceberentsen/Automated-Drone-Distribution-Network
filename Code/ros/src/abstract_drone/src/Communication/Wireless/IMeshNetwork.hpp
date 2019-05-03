/**
 * @file IMeshNetwork.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the interface IMeshNetwork
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IMESHNETWORK
#define IMESHNETWORK
#include <inttypes.h>
namespace Communication
{
namespace Wireless
{
 class IMeshNetwork
 {
 public:
  virtual ~IMeshNetwork( ){};
  /**
   * @brief Called upon each received Message.
   *
   * @param message Pointer to uint8_t array[32] holding the message
   */
  virtual void OnMsg( const uint8_t* message ) = 0;
  /**
   * @brief Get the nodeID
   *
   * @return uint8_t this->nodeID
   */
  virtual const uint8_t getNodeID( ) const = 0;
 };
}  // namespace Wireless
}  // namespace Communication
#endif  // IMESHNETWORK
