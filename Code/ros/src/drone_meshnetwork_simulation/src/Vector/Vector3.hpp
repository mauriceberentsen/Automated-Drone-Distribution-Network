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
    //We only want vectors holding real value's
    static_assert(std::is_arithmetic<T>::value, "Value T must be integral T or a floating-point T.");
    /**********************************************/
    /*                                            */
    /*                CONSTRUCTORS                */
    /*                                            */
    /**********************************************/
    
    /**
     * @brief Deafult constructor for a Vector3
     * 
     * @param defaultValue value that fills X,Y,Z
     */
    Vector3(T defaultValue = 0);
    /**
     * @brief Vector3 copy constructor
     * 
     * @param rhs the Vector3 to copy
     */
    Vector3(const Vector3<T>& rhs);
    /**
     * @brief Construct a new Vector 3 object from different values
     * 
     * @param aX X
     * @param aY Y
     * @param aZ Z
     */
    Vector3(const T aX,const T aY,const T aZ );
    ~Vector3(){};
    /**********************************************/
    /*                                            */
    /*                GETTERS                     */
    /*                                            */
    /**********************************************/
    
    /**
     * @brief Used for getting X
     * 
     * @return const value of X 
     */
    const T X() const;
    /**
     * @brief Used for getting Y
     * 
     * @return const value of Y 
     */
    const T Y() const;
    /**
     * @brief Used for getting Z
     * 
     * @return const value of Z
     */
    const T Z() const;
    /**********************************************/
    /*                                            */
    /*                SETTERS                     */
    /*                                            */
    /**********************************************/
    
    /**
     * @brief Sets the value of X
     * 
     * @param aX new value for X
     * @return const T new set value of X
     */
    const T X(const T aX);
    /**
    * @brief Sets the value of Y
    * 
    * @param aY new value for Y
    * @return const T new set value of Y
    */
    const T Y(const T aY);
    /**
     * @brief Sets the value of Z
     * 
     * @param aZ new value for Z
     * @return const T new set value of Z
     */
    const T Z(const T aZ);
    /**********************************************/
    /*                                            */
    /*                OPERATORS                   */
    /*                                            */
    /**********************************************/
    
    /**
     * @brief copys the content of the rhs Vector3 into this
     * 
     * @param rhs Vector3 to copy
     * @return Vector3<T>&  reference to this Vector3 
     */
    Vector3<T>& operator=(const Vector3<T>& rhs);
    /**
     * @brief sums this Vector3 up with the rhs Vector3 
     * returns the sum as a new Vector3 object
     * 
     * @param rhs the Vector3 to add up
     * @return Vector3<T> The new Vector3 a sum of this and rhs
     */
    Vector3<T> operator+(const Vector3<T>& rhs) const;
    /**
     * @brief add the rhs vector3 values to this 
     * 
     * @param rhs the Vector3 to add
     * @return Vector3<T>& reference to this
     */
    Vector3<T>& operator+=(const Vector3<T>& rhs);
     /**
     * @brief Substracts this Vector3 with the rhs Vector3 
     * returns the result as a new Vector3 object
     * 
     * @param rhs the Vector3 to substract
     * @return Vector3<T> The new Vector3 result
     */
    Vector3<T> operator-(const Vector3<T>& rhs) const;
     /**
     * @brief substract the rhs vector3 values to this 
     * 
     * @param rhs the Vector3 to substract
     * @return Vector3<T>& reference to this
     */
    Vector3<T>& operator-=(const Vector3<T>& rhs);
    /**
     * @brief Compares the values of the Vector3 lhs against rhs
     *        * lhs.x != rhs.x
     *        * lhs.y != rhs.y
     *        * lhs.z != rhs.z
     * 
     * @param rhs the vector3 to compare
     * @return true if not equal
     * @return false if equal
     */
    bool operator!=(const Vector3<T>& rhs);
    /**
     * @brief Compares the values of the Vector3 lhs against rhs
     *        * lhs.x == rhs.x
     *        * lhs.y == rhs.y
     *        * lhs.z == rhs.z
     * 
     * @param rhs the vector3 to compare
     * @return true if  equal
     * @return false if not equal
     */    
    bool operator==(const Vector3<T>& rhs);

    /**********************************************/
    /*                                            */
    /*                EXTRA FUNCTIONS             */
    /*                                            */
    /**********************************************/
    
    /**
     * @brief Returns the distance betweens this Vector3 and other 
     * 
     * @param rhs The other Vector3
     * @return float distance
     */
    float Distance(const Vector3<T>& rhs) const;

private:
    /// \brief X value of the Vector3
    T x;
    /// \brief Y value of the Vector3
    T y;
    /// \brief Z value of the Vector3
    T z;
};


#include "Vector3.inc"

#endif //VECTOR3
