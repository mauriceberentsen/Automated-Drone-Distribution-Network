#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <sstream>
#include <iostream>

namespace gazebo
{
class DroneFactory : public WorldPlugin
{
private:
 std::string createDroneSDFstring( uint8_t droneID, float x, float y, float z )
 {
  std::stringstream ss;
  ss << "<sdf version ='1.4'>\
          <model name ='drone'>\
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
            <NodeID>"
     << std::to_string( droneID ) << "</NodeID>\
              <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            </plugin>\
            <plugin filename='libdroneEngine.so' name='droneEngine'>\
             <DroneID>"
     << std::to_string( droneID ) << "</DroneID>\
            </plugin>\
          </model>\
        </sdf>";
  return ss.str( );
 }

  private: int ID = 1;
public:
 void createDrone( float x, float y, float z, physics::WorldPtr _parent )
 {
  sdf::SDF boxSDF;
  boxSDF.SetFromString( createDroneSDFstring( ID, x, y, z ) );
  // Demonstrate using a custom model name.
  sdf::ElementPtr model = boxSDF.Root( )->GetElement( "model" );
  model->GetAttribute( "name" )->SetFromString( "drone" +
                                                std::to_string( ID ) );
  _parent->InsertModelSDF( boxSDF );
  ++this->ID;
 }

public:
 void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf )
 {
  int amount = 0;

  if ( _sdf->HasElement( "amountOfDrones" ) ) {
   amount = _sdf->Get< int >( "amountOfDrones" );
  }
  if ( amount < 10 ) {
   for ( int i = 1; i < amount + 1; i++ ) {
    // start with 1 since the gateway already is number 0
    createDrone( 0, 0, 0, _parent );
   }
  } else {
   int rows, left;
   rows = std::floor( amount / 10 );
   left = ( amount % 10 ) + 1;
   for ( int i = 0; i < rows; i++ ) {
    for ( int j = 0; j < 10; j++ ) {
     if ( i + j == 0 ) continue;
     createDrone( i * 5, j * 5, 0.5, _parent );
    }
   }
   //  for(int i = amount - left; i <= amount; i++)
   //  {
   //   createDrone( i, rows*5+1,i*5 , 0.5, _parent );
   //  }
  }
 }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN( DroneFactory )
}  // namespace gazebo
