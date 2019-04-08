#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "abstract_drone/Location.h"
#include "abstract_drone/nodeInfo.h"

namespace gazebo
{
class DroneEngine : public ModelPlugin
{
public:
 void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
 {
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind( &DroneEngine::OnUpdate, this ) );

  this->old_secs = ros::Time::now( ).toSec( );

  std::string move_to_TopicName;
  std::string gps_TopicName;
  std::string ModelName = this->model->GetName( );
  // Check that the velocity element exists, then read the value
  if ( _sdf->HasElement( "DroneID" ) ) {
   this->drone_id = _sdf->Get< int >( "DroneID" );
   move_to_TopicName = "/Drones/" + std::to_string( this->drone_id ) + "/goal";
   gps_TopicName = "/Drones/" + std::to_string( this->drone_id ) + "/gps";
  } else {
   move_to_TopicName = ModelName + "/goal";
   gps_TopicName = "/Drones/" + std::to_string( this->drone_id ) + "/gps";
  }

  // Create a topic name

  // Initialize ros, if it has not already been initialized.
  if ( !ros::isInitialized( ) ) {
   int argc = 0;
   char **argv = NULL;
   ros::init( argc, argv, "drone", ros::init_options::NoSigintHandler );
  }
  ROS_INFO( "modelName =%s", move_to_TopicName.c_str( ) );
  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset( new ros::NodeHandle( "drone" ) );

  // Plannar Pose
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create< abstract_drone::Location >(
          move_to_TopicName, 100,
          boost::bind( &DroneEngine::OnRosMsg_Pos, this, _1 ), ros::VoidPtr( ),
          &this->rosQueue );
  this->rosSub = this->rosNode->subscribe( so );

  this->rosPostmasterPub = this->rosNode->advertise< abstract_drone::nodeInfo >(
      "/postMaster", 100 );
  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread( std::bind( &DroneEngine::QueueThread, this ) );

  ROS_WARN( "Loaded DroneEngine Plugin with parent...%s",
            this->model->GetName( ).c_str( ) );
  informPostmaster( );
 }

 // Called by the world update start event
public:
 void OnUpdate( )
 {
  if ( !this->hasGoal ) { return; }
  while ( !atGoal( ) && !moving ) {
   MoveModelsPlane( );
  }
  if ( moving ) { informPostmaster( ); }
  if ( this->atGoal( ) ) { moving = false; hasGoal = false;}
 }

 bool atGoal( )
 {
  float presision = 0.01;
  return ( goal.Pos( ).X( ) < pose.Pos( ).X( ) + presision ) &&
         ( goal.Pos( ).X( ) > pose.Pos( ).X( ) - presision ) &&
         ( goal.Pos( ).Y( ) < pose.Pos( ).Y( ) + presision ) &&
         ( goal.Pos( ).Y( ) > pose.Pos( ).Y( ) - presision ) &&
         ( goal.Pos( ).Z( ) < pose.Pos( ).Z( ) + presision ) &&
         ( goal.Pos( ).Z( ) > pose.Pos( ).Z( ) - presision );
 }

 void MoveModelsPlane( )
 {
  ROS_WARN( "call move model" );
  if ( moving ) return;
  ROS_WARN( "exec move model" );

  moving = true;
  ignition::math::Vector3d curVec( pose.Pos( ).X( ), pose.Pos( ).Y( ),
                                   pose.Pos( ).Z( ) );
  ignition::math::Vector3d goalVec( goal.Pos( ).X( ), goal.Pos( ).Y( ),
                                    goal.Pos( ).Z( ) );

  float distance = curVec.Distance( goalVec );
  distance /= this->speed;
  std::string model_name = this->model->GetName( );

  // create the animation
  gazebo::common::PoseAnimationPtr anim(
      // name the animation "test",
      // make it last 10 seconds,
      // and set it on a repeat loop
      new gazebo::common::PoseAnimation( "movement", distance, false ) );

  gazebo::common::PoseKeyFrame *key;

  key = anim->CreateKeyFrame( 0 );
  key->Translation( ignition::math::Vector3d( curVec ) );

  key = anim->CreateKeyFrame( distance * 0.1 );
  key->Translation( ignition::math::Vector3d( curVec.X( ), curVec.Y( ),
                                              ( curVec.Z( ) + 10 ) ) );

  key = anim->CreateKeyFrame( distance * 0.9 );
  key->Translation( ignition::math::Vector3d( goalVec.X( ), goalVec.Y( ),
                                              ( goalVec.Z( ) + 10 ) ) );

  key = anim->CreateKeyFrame( distance );
  key->Translation( ignition::math::Vector3d( goalVec ) );
  this->model->SetAnimation( anim );
 }

 void informPostmaster( )
 {
  pose = this->model->WorldPose( );
  abstract_drone::nodeInfo nI;
  nI.nodeID = this->drone_id;
  nI.position[0] = pose.Pos( ).X( );
  nI.position[1] = pose.Pos( ).Y( );
  nI.position[2] = pose.Pos( ).Z( );
  nI.sub = "";
  nI.on = true;
  rosPostmasterPub.publish( nI );
 }

 void setGoal( float longitude, float latitude, float height )
 {
  goal.Pos( ).X( longitude );
  goal.Pos( ).Y( latitude );
  goal.Pos( ).Z( height );
  this->hasGoal = true;
 }

public:
 void OnRosMsg_Pos( const abstract_drone::LocationConstPtr &_msg )
 {
  this->setGoal( _msg->longitude, _msg->latitude, _msg->height );
 }

 /// \brief ROS helper function that processes messages
private:
 void QueueThread( )
 {
  static const double timeout = 0.01;
  while ( this->rosNode->ok( ) ) {
   this->rosQueue.callAvailable( ros::WallDuration( timeout ) );
  }
 }

 // Pointer to the model
private:
 physics::ModelPtr model;
 bool hasGoal = false;

private:
 bool moving = false;

private:
 float speed = 2.0;
 // Pointer to the update event connection
private:
 event::ConnectionPtr updateConnection;

private:
 int drone_id;
 // Time Memory
 double old_secs;
 /// \brief A node use for ROS transport
private:
 std::unique_ptr< ros::NodeHandle > rosNode;
 /// \brief A ROS publisher
private:
 ros::Publisher rosPub;
 /// \brief A ROS postmasterpublisher
private:
 ros::Publisher rosPostmasterPub;
 /// \brief A ROS subscriber
private:
 ros::Subscriber rosSub;
 /// \brief A ROS callbackqueue that helps process messages
private:
 ros::CallbackQueue rosQueue;
 /// \brief A thread the keeps running the rosQueue
private:
 std::thread rosQueueThread;

private:
 ignition::math::Pose3d goal;

private:
 ignition::math::Pose3d pose;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( DroneEngine )
}  // namespace gazebo