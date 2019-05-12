/**
 * @file IMeshDebugInfo.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the interface IMeshDebugInfo
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IMESHDEBUGINFO
#define IMESHDEBUGINFO
#include <cstdint>
namespace Communication
{
namespace Wireless
{
 class IMeshDebugInfo
 {
 public:
  virtual ~IMeshDebugInfo( ){};
  /**
   * @brief Get the nodeID
   *
   * @return uint8_t this->nodeID
   */
  virtual const uint8_t getNodeID( ) const = 0;
  /**
   * @brief Get the ConnectedToGateway
   *
   * @return true connected
   * @return false not connected
   */
  virtual const bool getConnectedToGateway( ) const = 0;
  /**
   * @brief Get the Total Message Sent
   *
   * @return const uint32_t amount
   */
  virtual const uint32_t getTotalMessageSent( ) const = 0;
  /**
   * @brief Get the Preffered Gateway ID
   *
   * @return const uint8_t ID
   */
  virtual const uint8_t getPrefferedGateway( ) const = 0;
  /**
   * @brief Get theHopsFromGatewayAway
   *
   * @return const uint8_t amount of hops away
   */
  virtual const uint8_t getHopsFromGatewayAway( ) const = 0;
  /**
   * @brief Get the ID of LastGoodKnownLocation
   *
   * @return const uint8_t ID
   */
  virtual const uint8_t getLastGoodKnownLocationID( ) const = 0;
  /**
   * @brief Get the size of the RouterTechTable
   *
   * @return const uint8_t
   */
  virtual const uint8_t getRouterTechTableSize( ) const = 0;
 };
}  // namespace Wireless
}  // namespace Communication
#endif  // IMESHDEBUGINFO
