#ifndef ROSDRONEENGINECONNECTOR
#define ROSDRONEENGINECONNECTOR
#include "IDroneEngine.hpp"
#include "ros/ros.h"
#include "abstract_drone/Location.h"
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