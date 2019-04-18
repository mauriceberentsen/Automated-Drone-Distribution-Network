#ifndef DRONEMANAGER
#define DRONEMANAGER

#include "ros/ros.h"
#include "abstract_drone/RequestDroneFlight.h"
namespace DroneManagerService
{
class DroneManager
{
public:
 DroneManager( std::shared_ptr< ros::NodeHandle > &_rosNode,
               std::string &GatewayTopicName );
 ~DroneManager( );
 bool RequestMovement( uint8_t ID, float longitude, float latitude,
                       uint16_t height = 0 );
 bool RequestMovement( abstract_drone::RequestDroneFlight::Request &req,
                       abstract_drone::RequestDroneFlight::Response &res );
 bool setDronesToCasus( abstract_drone::RequestDroneFlight::Request &req,
                        abstract_drone::RequestDroneFlight::Response &res );

private:
 /// \brief ROS helper function that processes messages

 std::shared_ptr< ros::NodeHandle > nodeHandle;
 std::string GatewayTopic = "/gateway";

 /// \brief A ROS publisher
 ros::Publisher rosPub;
};
}  // namespace DroneManagerService
#endif  // DRONEMANAGER