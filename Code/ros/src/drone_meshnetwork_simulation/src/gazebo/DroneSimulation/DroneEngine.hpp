/**
 * @file DroneEngine.hpp
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
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "drone_meshnetwork_simulation/Location.h"
#include "drone_meshnetwork_simulation/RequestGPS.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"

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
   * @param latitude to fly to
   * @param longitude to fly to
   * @param height to fly to
   */
  void setGoal( const float latitude, const float longitude,
                const float height );
  /**
   * @brief Get the location of this drone, usable as a service
   *
   * @param req UNUSED PAR
   * @param res Holds the coordinates of this drone
   * @return true Succesfull service call
   * @return false Service call had an error
   */
  bool get_location( drone_meshnetwork_simulation::RequestGPS::Request &req,
                     drone_meshnetwork_simulation::RequestGPS::Response &res );

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
  void OnRosMsg_Pos(
      const drone_meshnetwork_simulation::LocationConstPtr &_msg );
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
  void ExecuteDroneMovement( );

 private:
  /// \brief The DroneID of this engine
  uint8_t drone_id;
  /// \brief Pointer to this model plugin
  physics::ModelPtr model;
  /// \brief Boolean if this engine has an active goal
  bool hasGoal = false;
  /// \brief Boolean if this drone is moving
  bool moving = false;
  /// \brief The speed this drone uses to move
  float speed = 2.0;
  /// \brief Pointer to the NodeHandler of this plugin
  std::unique_ptr< ros::NodeHandle > rosNode;
  /// \brief Pointer to the connection to Gazebo, used for iterating each
  /// simulation cycle
  event::ConnectionPtr updateConnection;
  /// \brief Service to request the location of this engine
  ros::ServiceServer gpsService;
  /// \brief Thread to process the ros messages
  std::thread rosQueueThread;
  /// \brief Queue for ros messages
  ros::CallbackQueue rosQueue;
  /// \brief Publisher to the WirelessSignalSimulator for the whereabouts of
  /// this drone
  ros::Publisher wirelessSimulatorPub;
  /// \brief Subscriber to recieve messages from the connected
  /// MeshnetworkComponent
  ros::Subscriber rosSub;
  /// \brief The goal for this engine to fly towards
  ignition::math::Pose3d goal;
  /// \brief The current location of this engine
  ignition::math::Pose3d pose;
 };
}  // namespace DroneSimulation
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( DroneSimulation::DroneEngine )
}  // namespace gazebo
#endif  // DRONEENGINE
