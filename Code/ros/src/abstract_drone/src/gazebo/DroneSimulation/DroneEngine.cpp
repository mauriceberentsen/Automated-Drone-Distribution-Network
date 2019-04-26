/**
 * @file droneEngine.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for the drone engine
 * @version 1.0
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ros/subscribe_options.h"
#include "abstract_drone/nodeInfo.h"

#include "DroneEngine.hpp"

namespace gazebo
{
namespace DroneSimulation
{
 DroneEngine::DroneEngine( ) : drone_id( UINT8_MAX ){};

 void DroneEngine::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
 {
  if ( _sdf->HasElement( "DroneID" ) ) {
   this->drone_id = _sdf->Get< int >( "DroneID" );
  } else {
   ROS_ERROR(
       "No DroneID is given, we need this to connect to a drone. Component "
       "will not be loaded" );
   return;
  }

  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind( &DroneEngine::OnUpdate, this ) );

  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "drone", ros::init_options::NoSigintHandler );
  }

  std::string move_to_TopicName;
  std::string gps_TopicName;
  move_to_TopicName = "/Drones/" + std::to_string( this->drone_id ) + "/goal";
  gps_TopicName = "/Drones/" + std::to_string( this->drone_id ) + "/gps";
  this->rosNode.reset( new ros::NodeHandle( "drone" ) );

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create< abstract_drone::Location >(
          move_to_TopicName, 100,
          boost::bind( &DroneEngine::OnRosMsg_Pos, this, _1 ), ros::VoidPtr( ),
          &this->rosQueue );

  this->rosSub = this->rosNode->subscribe( so );

  this->wirelessSimulatorPub =
      this->rosNode->advertise< abstract_drone::nodeInfo >(
          "/WirelessSignalSimulator", 100 );

  this->rosQueueThread =
      std::thread( std::bind( &DroneEngine::QueueThread, this ) );

  this->gpsService = this->rosNode->advertiseService(
      gps_TopicName, &DroneEngine::get_location, this );

  ROS_INFO( "Loaded DroneEngine Plugin connect to topicname:[%s]",
            this->model->GetName( ).c_str( ) );
  informWirelessSimulator( );
 }

 void DroneEngine::OnUpdate( )
 {
  if ( !this->hasGoal ) { return; }
  while ( !atGoal( ) && !moving ) {
   // TODO implement a better check because now we cant use multiple waypoints
   // or replan during flight
   BootDroneMovement( );
  }
  if ( moving ) { informWirelessSimulator( ); }
  if ( this->atGoal( ) ) {
   moving = false;
   hasGoal = false;
  }
 }

 bool DroneEngine::atGoal( )
 {
  const float presision = 0.01;
  return ( goal.Pos( ).X( ) < pose.Pos( ).X( ) + presision ) &&
         ( goal.Pos( ).X( ) > pose.Pos( ).X( ) - presision ) &&
         ( goal.Pos( ).Y( ) < pose.Pos( ).Y( ) + presision ) &&
         ( goal.Pos( ).Y( ) > pose.Pos( ).Y( ) - presision ) &&
         ( goal.Pos( ).Z( ) < pose.Pos( ).Z( ) + presision ) &&
         ( goal.Pos( ).Z( ) > pose.Pos( ).Z( ) - presision );
 }

 void DroneEngine::BootDroneMovement( )
 {
  ROS_WARN( "call move model" );
  if ( moving ) return;
  ROS_WARN( "exec move model" );

  moving = true;
  ignition::math::Vector3d curVec( pose.Pos( ).X( ), pose.Pos( ).Y( ),
                                   pose.Pos( ).Z( ) );
  ignition::math::Vector3d goalVec( goal.Pos( ).X( ), goal.Pos( ).Y( ),
                                    goal.Pos( ).Z( ) );

  float totalTravellingTime = curVec.Distance( goalVec );
  totalTravellingTime /= this->speed;

  gazebo::common::PoseAnimationPtr anim( new gazebo::common::PoseAnimation(
      "movement", totalTravellingTime, false ) );

  gazebo::common::PoseKeyFrame *key;

  key = anim->CreateKeyFrame( 0 );
  key->Translation( ignition::math::Vector3d( curVec ) );

  key = anim->CreateKeyFrame( totalTravellingTime * 0.1 );
  key->Translation( ignition::math::Vector3d(
      curVec.X( ), curVec.Y( ), ( curVec.Z( ) + totalTravellingTime * 0.5 ) ) );

  key = anim->CreateKeyFrame( totalTravellingTime * 0.9 );
  key->Translation( ignition::math::Vector3d(
      goalVec.X( ), goalVec.Y( ),
      ( goalVec.Z( ) + totalTravellingTime * 0.5 ) ) );

  key = anim->CreateKeyFrame( totalTravellingTime );
  key->Translation( ignition::math::Vector3d( goalVec ) );
  this->model->SetAnimation( anim );
 }

 void DroneEngine::informWirelessSimulator( )
 {
  pose = this->model->WorldPose( );
  abstract_drone::nodeInfo nI;
  nI.nodeID = this->drone_id;
  nI.position[0] = pose.Pos( ).X( );
  nI.position[1] = pose.Pos( ).Y( );
  nI.position[2] = pose.Pos( ).Z( );
  nI.sub = "";
  nI.on = true;
  wirelessSimulatorPub.publish( nI );
 }

 void DroneEngine::setGoal( const float latitude, const float longitude,
                            const float height )
 {
  goal.Pos( ).X( longitude );
  goal.Pos( ).Y( latitude );
  goal.Pos( ).Z( height );
  this->hasGoal = true;
 }

 void DroneEngine::OnRosMsg_Pos( const abstract_drone::LocationConstPtr &_msg )
 {
  this->setGoal( _msg->latitude, _msg->longitude, _msg->height );
 }

 bool DroneEngine::get_location( abstract_drone::RequestGPS::Request &req,
                                 abstract_drone::RequestGPS::Response &res )
 {
  res.longitude = pose.Pos( ).X( );
  res.latitude = pose.Pos( ).Y( );
  res.height = pose.Pos( ).Z( );
  return true;
 }

 void DroneEngine::QueueThread( )
 {
  static const double timeout = 0.01;
  while ( this->rosNode->ok( ) ) {
   this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
  }
 }
};  // namespace DroneSimulation
}  // namespace gazebo
