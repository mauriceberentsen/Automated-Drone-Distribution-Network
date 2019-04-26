#ifndef ROSINTERNETMOCK
#define ROSINTERNETMOCK

#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"

#include "abstract_drone/RequestGatewayDroneFlight.h"

#include "../Communication/IInternetConnection.hpp"
namespace Communication
{
namespace Meshnetwork
{
 class MeshnetworkGateway;
}
}  // namespace Communication
namespace ros
{
class RosInternetMock : public Communication::IInternetConnection
{
public:
 RosInternetMock( Communication::Meshnetwork::MeshnetworkGateway &MG );
 ~RosInternetMock( );
 void connect( );
 void disconnect( );

private:
 /**
  * @brief Used to queue up and handle ros message
  *
  */
 void QueueThread( );
 /**
  * @brief Through a subscriber handle incoming gateway messages
  *
  * @param _msg RequestGatewayDroneFlight messages
  */
 void gatewayQueue(
     const abstract_drone::RequestGatewayDroneFlightConstPtr &_msg );
 ros::CallbackQueue rosQueue;
 /// \brief Subscriber to the general gateway topic
 ros::Subscriber gatewaySub;
 /// \brief Reference to the gateway for incoming messages
 Communication::Meshnetwork::MeshnetworkGateway &meshnetworkGateway;
 /// \brief Pointer to the Ros Node of this class
 std::shared_ptr< ros::NodeHandle > rosNode;
 /// \brief thread to keep an open line for receiving NRF24 messages
 std::thread rosQueueThread;
};
}  // namespace ros
#endif  // ROSINTERNETMOCK
