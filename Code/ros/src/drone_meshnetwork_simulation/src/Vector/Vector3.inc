/**
 * @file Vector3.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief 
 * @version 1.0
 * @date 2019-05-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "Vector3.hpp"
#include <cmath>

template <typename T> 
Vector3<T>::Vector3(T defaultValue)
:x(defaultValue),y(defaultValue),z(defaultValue)
{
}

template <typename T> 
Vector3<T>::Vector3(const Vector3<T>& rhs)
:x(rhs.X()),y(rhs.Y()),z(rhs.Z())
{
}


template <typename T>
Vector3<T>::Vector3(const T aX,const T aY,const T aZ )
:x(aX), y(aY), z(aZ)
{
    
}

template <typename T>
const T Vector3<T>::X()const 
{
    return x;
}
template <typename T>
const T Vector3<T>::Y()const 
{
    return y;
}
template <typename T>
const T Vector3<T>::Z()const 
{
    return z;
}

template <typename T>
const T Vector3<T>::X(const T aX)
{
    x = aX;
    return x;
}
template <typename T>
const T Vector3<T>::Y(const T aY)
{
    y = aY;
    return y;
}

template <typename T>
const T Vector3<T>::Z(const T aZ)
{
    z = aZ;
    return z;
}

template <typename T>
Vector3<T>& Vector3<T>::operator=(const Vector3<T>& rhs)
{
    if (this != &rhs)
    {
        x = rhs.X();
        y = rhs.Y();
        z = rhs.Z();
    }
    return *this;
}

template <typename T>
Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& rhs)
{
    this->x += rhs.X();
    this->y += rhs.Y();
    this->z += rhs.Z();
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator+(const Vector3<T>& rhs) const
{
    Vector3<T> temp(*this);
    temp += rhs;
    return temp;
}

template <typename T>
Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& rhs)
{
    this->x -= rhs.X();
    this->y -= rhs.Y();
    this->z -= rhs.Z();
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator-(const Vector3<T>& rhs) const
{
    Vector3<T> temp(*this);
    temp -= rhs;
    return temp;
}


template <typename T>
bool Vector3<T>::operator!=(const Vector3<T>& rhs)
{
    return (this->x != rhs.X()) &&(this->z != rhs.Z()) &&(this->y != rhs.Y());
}
template <typename T>
bool Vector3<T>::operator==(const Vector3<T>& rhs)
{
    return (this->x == rhs.X()) &&(this->z == rhs.Z()) &&(this->y == rhs.Y());
}

template <typename T>
float Vector3<T>::Distance(const Vector3<T>& rhs)
{
    Vector3<T> difference(*this);
    difference -= rhs;
    return std::sqrt( std::pow( difference.X(), 2 ) + std::pow( difference.Y(), 2 ) + std::pow( difference.Z(), 2 ) );
}