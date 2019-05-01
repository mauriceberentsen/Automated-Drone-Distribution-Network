/**
 * @file VirtualDrone.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for all drone classes
 * @version 1.0
 * @date 2019-04-18
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "VirtualDrone.hpp"

namespace gazebo
{
namespace DroneSimulation
{
 /*static*/ int VirtualDrone::droneID = 0;

 VirtualDrone::VirtualDrone( const float _x, const float _y, const float _z,
                             physics::WorldPtr _parent )
     : x( _x ), y( _y ), z( _z ), parent( _parent )
 {
 }

 VirtualDrone::~VirtualDrone( )
 {
 }

 RouterDrone::RouterDrone( const float _x, const float _y, const float _z,
                           physics::WorldPtr _parent )
     : VirtualDrone( _x, _y, _z, _parent )
 {
  std::stringstream ss;
  ss << "<sdf version ='1.6'>\
          <model name ='router_drone'>\
           <static>1</static>\
            <pose>0 0 0.5 0 0 0</pose>\
            <link name ='link'>\
              <inertial>\
              <pose>"
     << x << " " << y << " " << z + 0.5 << " "
     << "0 0 0</pose>\
              </inertial>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
                <material>\
            <script>\
              <uri>file://media/materials/scripts/gazebo.material</uri>\
              <name>Gazebo/Green</name>\
            </script>\
          </material>\
              </visual>\
            </link>\
            <plugin name=\"MeshnetworkRouter\" filename=\"libMeshnetworkRouter.so\">\
            <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            <nodeID>"
     << std::to_string( droneID ) << "</nodeID>\
            <Debug>true</Debug>\
            </plugin>\
     <plugin filename = 'libDroneEngine.so' name ='DroneEngine'>\
     <DroneID>"
     << std::to_string( droneID ) << " </DroneID> </plugin>\
     </model ></sdf>";
  SdfString = ss.str( );

  sdf::SDF boxSDF;
  boxSDF.SetFromString( SdfString );
  // Demonstrate using a custom model name.
  sdf::ElementPtr model = boxSDF.Root( )->GetElement( "model" );
  model->GetAttribute( "name" )->SetFromString( "drone" +
                                                std::to_string( droneID ) );
  parent->InsertModelSDF( boxSDF );
  ++droneID;
 }

 RouterDrone::~RouterDrone( )
 {
 }

 GatewayDrone::GatewayDrone( const float _x, const float _y, const float _z,
                             physics::WorldPtr _parent )
     : VirtualDrone( _x, _y, _z, _parent )
 {
  std::stringstream ss;
  ss << "<sdf version ='1.6'>\
          <model name ='gateway_drone'>\
           <static>1</static>\
            <pose>0 0 0.5 0 0 0</pose>\
            <link name ='link'>\
              <inertial>\
              <pose>"
     << x << " " << y << " " << z + 0.5 << " "
     << "0 0 0</pose>\
              </inertial>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
                <material>\
            <script>\
              <uri>file://media/materials/scripts/gazebo.material</uri>\
              <name>Gazebo/Purple</name>\
            </script>\
          </material>\
              </visual>\
            </link>\
            <plugin name=\"MeshnetworkGateway\" filename=\"libMeshnetworkGateway.so\">\
            <nodeID>"
     << std::to_string( droneID ) << "</nodeID>\
              <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            <Debug>true</Debug>\
            </plugin>\
            <plugin filename='libDroneEngine.so' name='DroneEngine'>\
             <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            </plugin>\
          </model>\
        </sdf>";
  SdfString = ss.str( );

  sdf::SDF boxSDF;
  boxSDF.SetFromString( SdfString );
  sdf::ElementPtr model = boxSDF.Root( )->GetElement( "model" );
  model->GetAttribute( "name" )->SetFromString( "drone" +
                                                std::to_string( droneID ) );
  parent->InsertModelSDF( boxSDF );
  ++droneID;
 }

 GatewayDrone::~GatewayDrone( )
 {
 }
}  // namespace DroneSimulation
}  // namespace gazebo