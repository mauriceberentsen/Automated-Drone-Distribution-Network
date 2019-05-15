/**
 * @file WirelessSignalSimulator.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Source file for the WirelessSignalSimulatorService
 * @version 1.0
 * @date 2019-04-30
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "WirelessSignalSimulator.hpp"

int main( int argc, char **argv )
{
 ros::WirelessSimulation::WirelessSignalSimulator WSS( 9.0 );

 ROS_INFO( "Drone manager online" );
 ros::spin( );

 return 0;
}
