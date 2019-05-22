#include "NRF24HighLevelInterface.hpp"
#include <iostream>
int main(int argc, char const *argv[])
{
    NRF24HighLevelInterface high;
    high.StartAntenna(12);
    uint8_t buf[32] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3}; 

    while (true)
    {
        static int D = 0;
        ++D;
        if(D > 1000000000)
        {
	std::cout<<"Send"<<std::endl;
        if(high.SendMessageTo(buf))
	{std::cout<<"Success"<<std::endl;}
        D = 0;
        }
    }
    
    return 0;
}
