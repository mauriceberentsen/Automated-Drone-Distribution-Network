/**
 * @file Vector3.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief 
 * @version 1.0
 * @date 2019-05-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef VECTOR3
#define VECTOR3
#include <type_traits>


template <typename T>
class Vector3
{

public:
    static_assert(std::is_arithmetic<T>::value, "Value T must be integral T or a floating-point T.");

    Vector3(T defaultValue = 0);
    Vector3(const Vector3<T>& rhs);
    Vector3(const T aX,const T aY,const T aZ );
    ~Vector3(){};
    const T X() const;
    const T Y() const;
    const T Z() const;
    
    const T X(const T aX);
    const T Y(const T aY);
    const T Z(const T aZ);
    


    Vector3<T>& operator=(const Vector3<T>& rhs);
    Vector3<T> operator+(const Vector3<T>& rhs) const;
    Vector3<T>& operator+=(const Vector3<T>& rhs);
    Vector3<T> operator-(const Vector3<T>& rhs) const;
    Vector3<T>& operator-=(const Vector3<T>& rhs);
    bool operator!=(const Vector3<T>& rhs);
    bool operator==(const Vector3<T>& rhs);


    float Distance(const Vector3<T>& rhs);

private:
    T x;
    T y;
    T z;
    /* data */
};


#include "Vector3.inc"

#endif //VECTOR3
