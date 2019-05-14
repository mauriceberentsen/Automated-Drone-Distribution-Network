/**
 * @file IInternetConnection.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Header file for the IInternetConnection
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IINTERNETCONNECTION
#define IINTERNETCONNECTION
#include "../../Communication/Internet/IGatewayCommands.hpp"
namespace Communication
{
namespace Internet
{
 class IInternetConnection
 {
 public:
  /**
   * @brief Connect to the internet service
   *
   * @param IGC pointer to the required interface Gateway Commands
   */
  virtual void connect( Communication::Internet::IGatewayCommands *IGC ) = 0;
  /**
   * @brief  Disconnect from the internet service
   *
   */
  virtual void disconnect( ) = 0;
 };
}  // namespace Internet
}  // namespace Communication
#endif  // IINTERNETCONNECTION
