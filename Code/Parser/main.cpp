#include "message.hpp"
#include <inttypes.h>
#include <iostream>
#include <iomanip>

int main(int argc, char const *argv[])
{
    uint8_t ID = 122;
    float latitude = 179.98631;
    float longitude = -178.91209 ;
    uint16_t height = 1500;
    uint32_t time = 1552897409;

    char buffer[32];
    
    locationMessage myMessage(ID,latitude,longitude,height,time);

    myMessage.toPayload(buffer);

    locationMessage newMessage(101,buffer);
    std::cout<<myMessage.toString() << std::endl << newMessage.toString();
    

    return 0;
}                    
            