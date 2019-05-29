/**
 * @file MockDroneEngine.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief 
 * @version 1.0
 * @date 2019-05-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <iostream>
#include "MockDroneEngine.hpp"


MockDroneEngine::MockDroneEngine(/* args */)
{
}

MockDroneEngine::~MockDroneEngine()
{
}

void MockDroneEngine::turnOn(/* args */)
{
    on = true;
}

void MockDroneEngine::turnOff(/* args */)
{
    on = false;
}

void MockDroneEngine::setGoal(const float latitude, const float longitude,
                       const float height)
{
    pos.X(latitude);
    pos.Y(longitude);
    pos.Z(height);
    std::cout<<"Move yourself towards: latitude[" << latitude <<"] longitude["<< longitude <<"] height["<< height <<"]"<<std::endl;
}

 const Vector3< float > MockDroneEngine::getLocation(/* args */)
{
    return pos;
}
