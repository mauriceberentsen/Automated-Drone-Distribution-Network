#include "MockNetworkComponent.hpp"


int main(int argc, char const *argv[])
{
    uint8_t id = 3;
    uint8_t other = 12;
    MockNetworkComponent mock(id,other);
    
    NRF24HighLevelInterface high;
    
    while (true)
    {
        mock.DoStuf();
    }
    
    return 0;
}
