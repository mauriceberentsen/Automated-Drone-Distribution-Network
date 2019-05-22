#include "MockNetworkComponent.hpp"


int main(int argc, char const *argv[])
{
    uint8_t id = 12;
    uint8_t other = 3;
    MockNetworkComponent mock(id,other);
    
    NRF24HighLevelInterface high;
    
    while (true)
    {
        mock.DoStuf();
    }
    
    return 0;
}
