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

#include <pistache/description.h>
#include <pistache/endpoint.h>
#include <pistache/http.h>

#include "../../Communication/Internet/IGatewayCommands.hpp"
#include "../../Communication/Internet/IInternetConnection.hpp"
using namespace Pistache;

class InternetGateway : public Communication::Internet::IInternetConnection
{
public:
 void init( Communication::Internet::IGatewayCommands *IGC,
            uint8_t _threads = 1 );
 /**
  * @brief Connect to the internet service
  *
  * @param IGC pointer to the required interface Gateway Commands
  */
 void connect( );
 /**
  * @brief  Disconnect from the internet service
  *
  */
 void disconnect( );
 InternetGateway( );
 ~InternetGateway( );

private:
 /// \brief Stores the route descriptions.
 Rest::Description desc;
 /// \brief Used for creating routes.
 Rest::Router router;
 /**
  * @brief Start the internet service
  *        Publish the routes
  *        Standby for shutdown
  */
 void InternetService( );
 /**
  * @brief Creates the route descriptions for this service

  */
 void CreateDescription( );
 /**
  * @brief Use for creatibng drone movement request
  *        Used by route "/DroneMovement/:id/:latitude/:longitude/:height"
  *
  */
 void RequestDroneMovement( const Rest::Request &, Http::ResponseWriter );

 /// \brief thread to keep an seperate internet connection open
 std::thread InternetConnectionThread;
 /// \brief Pointer for the http endpoint
 std::shared_ptr< Http::Endpoint > httpEndpoint;
 /// \brief Reference to the gateway for incoming messages
 Communication::Internet::IGatewayCommands *meshnetworkGateway;
};

#endif  // INTERNETGATEWAY
