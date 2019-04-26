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
namespace Communication
{
class IInternetConnection
{
public:
 /**
  * @brief Connect to the internet service
  *
  */
 virtual void connect( ) = 0;
 /**
  * @brief  Disconnect from the internet service
  *
  */
 virtual void disconnect( ) = 0;
};
}  // namespace Communication

#endif  // IINTERNETCONNECTION
