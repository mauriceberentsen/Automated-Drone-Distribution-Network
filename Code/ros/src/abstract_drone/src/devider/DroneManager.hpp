#include <map>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "abstract_drone/nodeInfo.h"
#include "abstract_drone/WirelessMessage.h"
#include "abstract_drone/AreaScan.h"


class DroneManager
{
public:
 DroneManager( );
 ~DroneManager(){};
 void OnRosMsg( const abstract_drone::nodeInfoConstPtr &_msg );

private:
 bool sendDroneToLocation( abstract_drone::AreaScan::Request &req,
                           abstract_drone::AreaScan::Response &res );

 /// \brief ROS helper function that processes messages

 void QueueThread( );

std::string Node_TopicName;
 ros::ServiceServer service;
 ros::ServiceServer sendToLocationService;
 /// \brief A node use for ROS transport
 std::shared_ptr< ros::NodeHandle > rosNode;
 /// \brief A ROS subscriber
 ros::Subscriber rosSub;
 /// \brief A ROS callbackqueue that helps process messages
 ros::CallbackQueue rosQueue;
 /// \brief A thread the keeps running the rosQueue
 std::thread rosQueueThread;
};
