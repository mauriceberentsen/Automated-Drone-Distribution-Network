/**
 * @file InternetGateway.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief 
 * @version 1.0
 * @date 2019-05-28
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef INTERNETGATEWAY
#define INTERNETGATEWAY
#include <thread>

#include <pistache/http.h>
#include <pistache/description.h>
#include <pistache/endpoint.h>

#include "../../Communication/Internet/IInternetConnection.hpp"
#include "../../Communication/Internet/IGatewayCommands.hpp"
using namespace Pistache;


class InternetGateway : public Communication::Internet::IInternetConnection
{
public:
  void init( Communication::Internet::IGatewayCommands *IGC, uint8_t _threads = 1 );
  /**
   * @brief Connect to the internet service
   *
   * @param IGC pointer to the required interface Gateway Commands
   */
  void connect();
  /**
   * @brief  Disconnect from the internet service
   *
   */
  void disconnect( );
    InternetGateway(/* args */);
    ~InternetGateway();

private:

  Rest::Router router;
  Rest::Description desc;
  
  void InternetService();
  void createDescription();
  
  
  void GetValueBack(const Rest::Request&, Http::ResponseWriter);
//   void handleReady(const Rest::Request&, Http::ResponseWriter response);
  
  
  /// \brief thread to keep an seperate internet connection open
  std::thread InternetConnectionThread;
  std::shared_ptr<Http::Endpoint> httpEndpoint;  
  /// \brief Reference to the gateway for incoming messages 
  Communication::Internet::IGatewayCommands *meshnetworkGateway; 

};




#endif //INTERNETGATEWAY
