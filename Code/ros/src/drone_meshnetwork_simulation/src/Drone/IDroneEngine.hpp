/**
 * @file IDroneEngine.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief Interface for Drone engine
 * @version 1.0
 * @date 2019-04-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef IDRONEENGINE
#define IDRONEENGINE
#include <ignition/math/Vector3.hh>

namespace Drone
{
class IDroneEngine
{
public:
 /**
  * @brief Turns the engine on
  *
  */
 virtual void turnOn( ) = 0;
 /**
  * @brief Turns the engine off
  *
  */
 virtual void turnOff( ) = 0;
 /**
  * @brief Set the Goal for the drone to fly to
  *
  * @param latitude
  * @param longitude
  * @param height
  */
 virtual void setGoal( const float latitude, const float longitude,
                       const float height ) = 0;
 /**
  * @brief Get the Location of the drone
  *
  * @return const ignition::math::Vector3< float > Current location of the
  * drone
  */
 virtual const ignition::math::Vector3< float > getLocation( ) = 0;

protected:
private:
};
}  // namespace Drone

#endif  // IDRONEENGINE