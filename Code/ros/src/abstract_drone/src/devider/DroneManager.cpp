#include "DroneManager.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv,  "DroneManager", ros::init_options::NoSigintHandler );
  ros::NodeHandle n;

  return 0;
}


DroneManager::DroneManager( ) : Node_TopicName( "DroneManagerService" )
{

 // CreateROS node.
 this->rosNode.reset( new ros::NodeHandle( "SignalSimulator" ) );

 ros::SubscribeOptions so =
     ros::SubscribeOptions::create< abstract_drone::nodeInfo >(
         Node_TopicName, 1,
         boost::bind( &DroneManager

                      ::OnRosMsg,
                      this, _1 ),
         ros::VoidPtr( ), &this->rosQueue );
 this->rosSub = this->rosNode->subscribe( so );
 this->sendToLocationService =
     this->rosNode->advertiseService( "message",
                                      &DroneManager::sendDroneToLocation,
                                      this );

 this->rosQueueThread = std::thread( std::bind( &DroneManager

                                                ::QueueThread,
                                                this ) );
}

void DroneManager::OnRosMsg( const abstract_drone::nodeInfoConstPtr &_msg )
{
}

/// \brief ROS helper function that processes messages
void DroneManager::QueueThread( )
{
 static const double timeout = 0.01;
 while ( this->rosNode->ok( ) ) {
  this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
 }
}

