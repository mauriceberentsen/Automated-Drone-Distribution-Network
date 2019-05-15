// Bring in my package's API, which is what I'm testing
#include "../HybridLMRoutingProtocol.hpp"
// Bring in gtest
#include <gtest/gtest.h>

using namespace Communication::RoutingTechnique;
// Declare a test
TEST( TestSuiteChildTableTree, addNeighbours )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 3 );
}

TEST( TestSuiteChildTableTree, addDoubleNeighbours )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 3 );
}

TEST( TestSuiteChildTableTree, cantCommunicate )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 CTT.cantCommunicateWithNode( 1 );
 CTT.cantCommunicateWithNode( 2 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 1 );
}

TEST( TestSuiteChildTableTree, addGranChildToUnknownChild )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 6 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 3 );
 EXPECT_TRUE( CTT.getTableSize( ) == 6 );
}

TEST( TestSuiteChildTableTree, addGranChildToKnownChild )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );
 CTT.canCommunicateWithNode( 4 );

 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 6 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 4 );
 EXPECT_TRUE( CTT.getTableSize( ) == 7 );
}

TEST( TestSuiteChildTableTree, addSameGrandChildToAll )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 4 );
 CTT.OtherCanCommunicateWithNode( 3, 4 );

 EXPECT_TRUE( CTT.getAmountOfChildren( ) == 3 );
 EXPECT_TRUE( CTT.getTableSize( ) == 6 );
}

TEST( TestSuiteChildTableTree, findRouteToNeighbour )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 EXPECT_TRUE( CTT.getDirectionToNode( 3 ) == 3 );
}

TEST( TestSuiteChildTableTree, noRoutePossible )
{
 HybridLMRoutingProtocol CTT;
 CTT.canCommunicateWithNode( 1 );
 CTT.canCommunicateWithNode( 2 );
 CTT.canCommunicateWithNode( 3 );

 EXPECT_TRUE( CTT.getDirectionToNode( 4 ) == UINT8_MAX );
}

TEST( TestSuiteChildTableTree, findRouteToGrandChild )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 5 );

 EXPECT_TRUE( CTT.getDirectionToNode( 5 ) == 2 );
}

TEST( TestSuiteChildTableTree, removeGrandchildRoute )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 6 );

 CTT.OtherCantCommunicateWithNode( 2, 5 );

 EXPECT_TRUE( CTT.getDirectionToNode( 5 ) == UINT8_MAX );
}

TEST( TestSuiteChildTableTree, alternativeRouteToGrandchild )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 5 );

 CTT.OtherCantCommunicateWithNode( 2, 5 );

 EXPECT_TRUE( CTT.getDirectionToNode( 5 ) == 3 );
}

TEST( TestSuiteChildTableTree, noRouteToGrandchildBecauseMissingParent )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 6 );

 CTT.cantCommunicateWithNode( 2 );

 EXPECT_TRUE( CTT.getDirectionToNode( 5 ) == UINT8_MAX );
}

TEST( TestSuiteChildTableTree, NodeMovedLocation )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );
 CTT.OtherCanCommunicateWithNode( 2, 5 );
 CTT.OtherCanCommunicateWithNode( 3, 6 );

 CTT.NodeMovedLocation( );

 EXPECT_TRUE( CTT.getTableSize( ) == 0 );
}

TEST( TestSuiteChildTableTree, empty )
{
 HybridLMRoutingProtocol CTT;

 EXPECT_TRUE( CTT.empty( ) );
}

TEST( TestSuiteChildTableTree, notempty )
{
 HybridLMRoutingProtocol CTT;
 CTT.OtherCanCommunicateWithNode( 1, 4 );

 EXPECT_FALSE( CTT.empty( ) );
}

// Run all the tests that were declared with TEST()
int main( int argc, char** argv )
{
 testing::InitGoogleTest( &argc, argv );
 return RUN_ALL_TESTS( );
}