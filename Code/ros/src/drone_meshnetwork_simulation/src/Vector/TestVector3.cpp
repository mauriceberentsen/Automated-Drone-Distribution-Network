// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>
#include "Vector3.hpp"
#include <cmath>

TEST( Vector3Testing, defaultConstructor )
{
 Vector3<float> vec;
 EXPECT_EQ( 0.0, vec.X( ) );
 EXPECT_EQ( 0.0, vec.Y( ) );
 EXPECT_EQ( 0.0, vec.Z( ) );
}

TEST( Vector3Testing, FiveConstructor )
{
 Vector3<int> vec(5);
 EXPECT_EQ( 5, vec.X( ) );
 EXPECT_EQ( 5, vec.Y( ) );
 EXPECT_EQ( 5, vec.Z( ) );
}

TEST( Vector3Testing, OneTwoTreeConstructor )
{
 Vector3<int> vec(1,2,3);
 EXPECT_EQ( 1, vec.X( ) );
 EXPECT_EQ( 2, vec.Y( ) );
 EXPECT_EQ( 3, vec.Z( ) );
}

TEST( Vector3Testing, CopyConstructor )
{
 Vector3<int> vec1(5);
 
 auto vec2(vec1);
 
 
 EXPECT_EQ( 5, vec2.X( ) );
 EXPECT_EQ( 5, vec2.Y( ) );
 EXPECT_EQ( 5, vec2.Z( ) );
}

TEST( Vector3Testing, setX )
{
 Vector3<int> vec;
 vec.X(42);
 
 EXPECT_EQ( 42, vec.X( ) );
 EXPECT_EQ( 0, vec.Y( ) );
 EXPECT_EQ( 0, vec.Z( ) );
}

TEST( Vector3Testing, setY )
{
 Vector3<int> vec;
 vec.Y(42);
 
 EXPECT_EQ( 0, vec.X( ) );
 EXPECT_EQ( 42, vec.Y( ) );
 EXPECT_EQ( 0, vec.Z( ) );
}

TEST( Vector3Testing, setZ )
{
 Vector3<int> vec;
 vec.Z(42);
 
 EXPECT_EQ( 0, vec.X( ) );
 EXPECT_EQ( 0, vec.Y( ) );
 EXPECT_EQ( 42, vec.Z( ) );
}

TEST( Vector3Testing, operatorIs )
{
 Vector3<int> vec(12);
 Vector3<int> vec2;

 vec2 = vec;
 
 EXPECT_EQ( 12, vec2.X( ) );
 EXPECT_EQ( 12, vec2.Y( ) );
 EXPECT_EQ( 12, vec2.Z( ) );
}

TEST( Vector3Testing, operatorPlusIs )
{
 Vector3<int> vec(12);
 Vector3<int> vec2(30);

 vec += vec2;
 
 EXPECT_EQ( 42, vec.X( ) );
 EXPECT_EQ( 42, vec.Y( ) );
 EXPECT_EQ( 42, vec.Z( ) );
}

TEST( Vector3Testing, operatorPlus )
{
 Vector3<int> vec(12);
 Vector3<int> vec2(30);

 auto vec3 = vec + vec2;
 
 EXPECT_EQ( 42, vec3.X( ) );
 EXPECT_EQ( 42, vec3.Y( ) );
 EXPECT_EQ( 42, vec3.Z( ) );
}

TEST( Vector3Testing, operatorMinusIs )
{
 Vector3<int> vec(32);
 Vector3<int> vec2(30);

 vec -= vec2;
 
 EXPECT_EQ( 2, vec.X( ) );
 EXPECT_EQ( 2, vec.Y( ) );
 EXPECT_EQ( 2, vec.Z( ) );
}

TEST( Vector3Testing, operatorMinus )
{
 Vector3<int> vec(32);
 Vector3<int> vec2(30);

 Vector3<int>  vec3 = vec - vec2;
 
 EXPECT_EQ( 2, vec3.X( ) );
 EXPECT_EQ( 2, vec3.Y( ) );
 EXPECT_EQ( 2, vec3.Z( ) );
}

TEST (Vector3Testing, operatorEqual)
{
 Vector3<int> vec(32);
 Vector3<int> vec2(30);
 Vector3<int> vec3(32);


 EXPECT_TRUE(vec == vec3);
 EXPECT_FALSE(vec == vec2);   
}

TEST (Vector3Testing, operatorNotEqual)
{
 Vector3<int> vec(32);
 Vector3<int> vec2(30);
 Vector3<int> vec3(32);


 EXPECT_TRUE(vec != vec2);
 EXPECT_FALSE(vec != vec3);
}

TEST (Vector3Testing, distancePositiveFive)
{
 Vector3<int> vec(10);
 Vector3<int> vec2(5);

 float dif = std::pow(5,2) * 3;   
 float result = std::sqrt(dif);

 EXPECT_FLOAT_EQ(vec.Distance(vec2), result);
}

TEST (Vector3Testing, distanceNegativeFive)
{
 Vector3<int> vec(-10);
 Vector3<int> vec2(-5);

 float dif = std::pow(5,2) * 3;   
 float result = std::sqrt(dif);

 EXPECT_FLOAT_EQ(vec.Distance(vec2), result);
}

TEST (Vector3Testing, distanceNegaPosFive)
{
 Vector3<int> vec(5);
 Vector3<int> vec2(-5);

 float dif = std::pow(10,2) * 3;   
 float result = std::sqrt(dif);

 EXPECT_FLOAT_EQ(vec.Distance(vec2), result);
}

TEST (Vector3Testing, distanceDifferentNumbers)
{
 Vector3<int> vec(0);
 Vector3<int> vec2(5,8,13);

 float dif = std::pow(5,2) + std::pow(8,2) +std::pow(13,2);   
 float result = std::sqrt(dif);

 EXPECT_FLOAT_EQ(vec.Distance(vec2), result);
}

// Run all the tests that were declared with TEST()
int main( int argc, char** argv )
{
 testing::InitGoogleTest( &argc, argv );
 return RUN_ALL_TESTS( );
}