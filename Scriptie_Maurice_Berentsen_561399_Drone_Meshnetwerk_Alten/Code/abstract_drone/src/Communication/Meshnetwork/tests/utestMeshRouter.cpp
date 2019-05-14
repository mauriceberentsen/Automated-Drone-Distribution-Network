/****************************/
/*                          */
/*     !!!WARNING!!!        */
/*    Test isn't ready      */
/*                          */
/****************************/

#define IGNORE

#ifndef IGNORE

// Bring in my package's API, which is what I'm testing
#include "../MeshnetworkRouter.hpp"
#include "../../Meshnetwork/MeshnetworkRouter.hpp"
#include "../../RoutingTechnique/HybridLMRoutingProtocol.hpp"
#include "../../../ros/RosDroneEngineConnector.hpp"
#include "../../../ros/WirelessSimulation/VirtualNRF24.hpp"
#include "../../../ros/WirelessSimulation/WirelessSignalSimulator.hpp"
// Bring in gtest
#include <gtest/gtest.h>
#include "gmock/gmock.h"

#include <ros/ros.h>
#include <chrono>
#include <thread>

// To use a test fixture, derive a class from testing::Test.
class TestRouterModule : public testing::Test
{
protected:  // You should make the members protected s.t. they can be
            // accessed from sub-classes.
 // virtual void SetUp() will be called before each test is run.  You
 // should define it if you need to initialize the variables.
 // Otherwise, this can be skipped.

 void SetUp( ) override
 {
  // meshnetworkRouter1->Init( );
  // meshnetworkRouter2->Init( );
 }

 // virtual void TearDown() will be called after each test is run.
 // You should define it if there is cleanup work to do.  Otherwise,
 // you don't have to provide it.
 //
 virtual void TearDown( )
 {
  meshnetworkRouter1->Stop( );
  meshnetworkRouter2->Stop( );
  wss->Stop( );
 }

 ros::WirelessSimulation::WirelessSignalSimulator* wss =
     new ros::WirelessSimulation::WirelessSignalSimulator( 9.0 );

 bool debug = false;

 Communication::RoutingTechnique::HybridLMRoutingProtocol* routing =
     new Communication::RoutingTechnique::HybridLMRoutingProtocol( );

 ros::Drone::RosDroneEngineConnector* engine =
     new ros::Drone::RosDroneEngineConnector( 1 );

 ros::WirelessSimulation::VirtualNRF24* NRF24 =
     new ros::WirelessSimulation::VirtualNRF24( );

 Communication::Meshnetwork::MeshnetworkRouter* meshnetworkRouter1 =
     new Communication::Meshnetwork::MeshnetworkRouter( 1, 1, debug, routing,
                                                        engine, NRF24 );
 /*************************************************************/
 Communication::RoutingTechnique::HybridLMRoutingProtocol* routing2 =
     new Communication::RoutingTechnique::HybridLMRoutingProtocol( );

 ros::Drone::RosDroneEngineConnector* engine2 =
     new ros::Drone::RosDroneEngineConnector( 2 );

 ros::WirelessSimulation::VirtualNRF24* NRF242 =
     new ros::WirelessSimulation::VirtualNRF24( );

 Communication::Meshnetwork::MeshnetworkRouter* meshnetworkRouter2 =
     new Communication::Meshnetwork::MeshnetworkRouter( 2, 2, debug, routing2,
                                                        engine2, NRF242 );
};

// Run all the tests that were declared with TEST()
int main( int argc, char** argv )
{
 ros::init( argc, argv, "test_talker" );
 ros::NodeHandle nh;
 testing::InitGoogleTest( &argc, argv );
 return RUN_ALL_TESTS( );
}
#endif