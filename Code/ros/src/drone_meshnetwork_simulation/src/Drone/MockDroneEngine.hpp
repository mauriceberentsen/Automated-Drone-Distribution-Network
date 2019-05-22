/**
 * @file MockDroneEngine.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief 
 * @version 1.0
 * @date 2019-05-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef MOCKDRONENGINE
#define MOCKDRONENGINE
#include "IDroneEngine.hpp"

class MockDroneEngine : public Drone::IDroneEngine
{
private:
Vector3< float > pos;
bool on = true;
    /* data */
public:
    /**
  * @brief Turns the engine on
  *
  */
 void turnOn( );
 /**
  * @brief Turns the engine off
  *
  */
 void turnOff( );
 /**
  * @brief Set the Goal for the drone to fly to
  *
  * @param latitude
  * @param longitude
  * @param height
  */
 void setGoal( const float latitude, const float longitude,
                       const float height );
 /**
  * @brief Get the Location of the drone
  *
  * @return const Vector3< float > Current location of the
  * drone
  */
 const Vector3< float > getLocation( );
    MockDroneEngine(/* args */);
    ~MockDroneEngine();
};







#endif //MOCKDRONENGINE