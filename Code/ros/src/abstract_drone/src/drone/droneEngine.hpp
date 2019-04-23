/**
 * @file droneEngine.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief header file for the droneEngine
 * @version 1.0
 * @date 2019-04-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef DRONEENGINE
#define DRONEENGINE

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "abstract_drone/Location.h"
#include "abstract_drone/RequestGPS.h"

namespace gazebo
{
namespace DroneSimulation
{
 class DroneEngine : public ModelPlugin
 {
 public:
  /**
   * @brief Construct a new Drone Engine object
   *
   */
  DroneEngine( );
  /**
   * @brief Set the Goal for this drone to fly towards
   *
   * @param longitude to fly to
   * @param latitude to fly to
   * @param height to fly to
   */
  void setGoal( const float longitude, const float latitude,
                const float height );
  /**
   * @brief Get the location of this drone, usable as a service
   *
   * @param req UNUSED PAR
   * @param res Holds the coordinates of this drone
   * @return true Succesfull service call
   * @return false Service call had an error
   */
  bool get_location( abstract_drone::RequestGPS::Request &req,
                     abstract_drone::RequestGPS::Response &res );

 private:
  /**
   * @brief This function is called when loading the component
   *        - It starts ros if it is not yet running
   *        - Extracts DroneID
   *
   * @param _parent Pointer to this plugin
   * @param _sdf Pointer to the sdf of this plugin
   */
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
  /**
   * @brief Called every simulation cycle, triggers drone movement and informs
   * the wireless simulator about updated positions
   *
   */
  void OnUpdate( );
  /**
   * @brief Check if the drone is at the goal position with an margin of 0.1
   * because of float precision
   *
   * @return true at goal
   * @return false not at goal
   */
  bool atGoal( );
  /**
   * @brief Informs the wireless signal simulator about the position of the
   * drone
   *
   */
  void informWirelessSimulator( );
  /**
   * @brief Sets the goal of the drone to move towards.
   *
   * @param _msg Location message
   */
  void OnRosMsg_Pos( const abstract_drone::LocationConstPtr &_msg );
  /**
   * @brief Starts the queueing of ros messages
   *
   */
  void QueueThread( );
  /**
   * @brief plans movement for the drone towards the goal waypoint
   *        - calculates the distance between the two points
   *        - plans an animation with an vertical take off and loading
   *        - plays the animation
   *
   */
  void BootDroneMovement( );

 private:
  uint8_t drone_id;
  physics::ModelPtr model;

  bool hasGoal = false;
  bool moving = false;

  float speed = 2.0;

  std::unique_ptr< ros::NodeHandle > rosNode;

  event::ConnectionPtr updateConnection;
  ros::ServiceServer gpsService;

  std::thread rosQueueThread;
  ros::CallbackQueue rosQueue;

  ros::Publisher wirelessSimulatorPub;
  ros::Publisher rosPub;

  ros::Subscriber rosSub;

  ignition::math::Pose3d goal;
  ignition::math::Pose3d pose;
 };
}  // namespace DroneSimulation
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( DroneSimulation::DroneEngine )
}  // namespace gazebo
#endif  // DRONEENGINE
