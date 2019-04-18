#include "Drone.hpp"

namespace gazebo
{
namespace DroneSimulation
{
 /*static*/ int Drone::droneID = 0;

 Drone::Drone( const float _x, const float _y, const float _z,
               physics::WorldPtr _parent )
     : x( _x ), y( _y ), z( _z ), parent( _parent )
 {
 }

 Drone::~Drone( )
 {
 }

 RouterDrone::RouterDrone( const float _x, const float _y, const float _z,
                           physics::WorldPtr _parent )
     : Drone( _x, _y, _z, _parent )
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
            <plugin name=\"meshnetworkCom\" filename=\"libmeshnetworkCom.so\">\
            <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            <nodeID>"
     << std::to_string( droneID ) << "</nodeID>\
     </plugin>\
     <plugin filename = 'libdroneEngine.so' name ='droneEngine'>\
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
     : Drone( _x, _y, _z, _parent )
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
            <plugin name=\"meshnetworkGateway\" filename=\"libmeshnetworkGateway.so\">\
            <nodeID>"
     << std::to_string( droneID ) << "</nodeID>\
              <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            </plugin>\
            <plugin filename='libdroneEngine.so' name='droneEngine'>\
             <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            </plugin>\
          </model>\
        </sdf>";
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

 GatewayDrone::~GatewayDrone( )
 {
 }
}  // namespace DroneSimulation
}  // namespace gazebo