/**
 * @file rosDroneEngineConnector.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for rosDroneEngineConnector
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef ROSDRONEENGINECONNECTOR
#define ROSDRONEENGINECONNECTOR
// ros libary
#include "ros/ros.h"
#include "abstract_drone/Location.h"
// local header
#include "../Drone/IDroneEngine.hpp"

namespace ros
{
class rosDroneEngineConnector : public Drone::IDroneEngine
{
public:
 rosDroneEngineConnector( uint16_t droneID );
 ~rosDroneEngineConnector( );
 /**
  * @brief Uses Ros transport to send a goal to the virtual engine
  *
  * @param latitude
  * @param longitude
  * @param height
  */
 void setGoal( const float latitude, const float longitude,
               const float height );
 /**
  * @brief Get the Location of the drone
  *
  * @return const ignition::math::Vector3< float > Current location of the
  * drone
  */
 const ignition::math::Vector3< float > getLocation( );

private:
 /// \brief the ID of the connected engine
 uint16_t ID;
 /// \brief Publisher towards the DroneEngine
 ros::Publisher droneEnginePublisher;
 /// \brief Service for requesting the current location
 ros::ServiceClient GPSLink;
 /// \brief Pointer to the Ros Node of this class
 std::shared_ptr< ros::NodeHandle > rosNode;
};
}  // namespace ros
#endif  // ROSDRONEENGINECONNECTOR