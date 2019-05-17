/**
 * @file IRoutingEssentials.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Interface for Routing Techniques
 * @version 1.0
 * @date 2019-04-04
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IROUTINGESSENTIALS
#define IROUTINGESSENTIALS
#include <cstdint>
#include <set>
namespace Communication
{
namespace RoutingTechnique
{
 class IRoutingEssentials
 {
 public:
  virtual ~IRoutingEssentials( ){};
  /**
   * @brief Broadcast a introduction to every node nearby
   *
   */
  virtual void searchOtherNodesInRange( ) = 0;
  /**
   * @brief Send a heartbeat message using wireless communication
   *
   * @param other The destination node to recieve the heartbeat
   * @return true Message sent away node doesn't need to be destination
   * @return false Message sending failed
   */
  virtual bool sendHeartbeat( uint8_t other ) = 0;
 };
}  // namespace RoutingTechnique
}  // namespace Communication
#endif  // IROUTINGESSENTIALS
