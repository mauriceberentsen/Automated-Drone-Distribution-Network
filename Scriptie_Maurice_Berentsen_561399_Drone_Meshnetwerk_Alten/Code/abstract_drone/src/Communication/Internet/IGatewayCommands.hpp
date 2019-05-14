/**
 * @file IGatewayCommands.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the interface IGatewayCommands
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IGATEWAYCOMMANDS
#define IGATEWAYCOMMANDS
#include <cstdint>

namespace Communication
{
namespace Internet
{
 class IGatewayCommands
 {
 public:
  /**
   * @brief Destroy the IGatewayCommands object
   *
   */
  virtual ~IGatewayCommands( ){};
  /**
   * @brief Sends a goal to a drone
   *
   */
  virtual void SendGoalRequestToDrone( const uint8_t ID, const float latitude,
                                       const float longitude,
                                       const uint16_t height ) = 0;
 };
}  // namespace Internet
}  // namespace Communication
#endif  // IGATEWAYCOMMANDS
