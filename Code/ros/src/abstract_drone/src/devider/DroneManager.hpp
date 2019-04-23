/**
 * @file DroneManager.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for DroneManager
 * @version 1.0
 * @date 2019-04-05
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef DRONEMANAGER
#define DRONEMANAGER

#include "ros/ros.h"
#include "abstract_drone/RequestDroneFlight.h"
namespace DroneManagerService
{
class DroneManager
{
public:
 /**
  * @brief Construct a new Drone Manager object
  *
  * @param _rosNode RosNode to start a service on
  * @param GatewayTopicName The name that gateways use
  */
 DroneManager( std::shared_ptr< ros::NodeHandle > &_rosNode,
               std::string &GatewayTopicName );
 ~DroneManager( );
 /**
  * @brief function to publish a request a specific drone to a given location
  *
  * @param ID The Drone that needs to move
  * @param longitude to go to
  * @param latitude to go to
  * @param height to go to
  * @return true always
  */
 bool RequestMovement( uint8_t ID, float longitude, float latitude,
                       uint16_t height = 0 );
 /**
  * @brief servicecall to publish a request a specific drone to a given location
  *
  * @param ID The Drone that needs to move
  * @param longitude to go to
  * @param latitude to go to
  * @param height to go to
  * @return true Service call succesfull
  * @return false Could not perform service
  */
 bool RequestMovement( abstract_drone::RequestDroneFlight::Request &req,
                       abstract_drone::RequestDroneFlight::Response &res );
 /**
  * @brief Set the Drones to a given casus
  *
  * @param req.id The drone that needs to move
  *        - 0 = All drone move
  *        - 1-17 drone id that needs to move
  *        - 100 set all drone to a grid of 10 x 10
  * @param res UNUSED PARAM
  * @return true servicecall succesfull
  * @return false servicecall had an error
  */
 bool setDronesToCasus( abstract_drone::RequestDroneFlight::Request &req,
                        abstract_drone::RequestDroneFlight::Response &res );

private:
 /// \brief Pointer to RosNode used for communication transport
 std::shared_ptr< ros::NodeHandle > nodeHandle;
 /// \brief Name of topic that Gateways are subscribed to
 std::string GatewayTopic = "/gateway";
 /// \brief ros publisher used to send messages to gateways
 ros::Publisher rosPub;
};
}  // namespace DroneManagerService
#endif  // DRONEMANAGER