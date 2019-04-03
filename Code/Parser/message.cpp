#include "message.hpp"
#include <sstream>
#include <iomanip>

message::message(uint8_t _ID, uint8_t _messagetype)
:ID(_ID), type(_messagetype)
{
}

message::~message()
{
}

void message::CopyToCharArray(char* value, uint16_t size, char* arr, uint16_t start)
{
    for(uint16_t i = 0; i < size; i++)
    {
        arr[i+start] = value[i];
    }
}

void message::CopyFromCharArray(char* value, uint16_t size, char* arr, uint16_t start)
{
    for(uint16_t i = 0; i < size; i++)
    {
        value[i] = arr[i+start];
    }
}

locationMessage::locationMessage(uint8_t _ID, float _latitude, float _longitude, int16_t _height, uint32_t _timeSincePosix)
:message(_ID, LOCATION), latitude(_latitude), longitude(_longitude), height(_height), timeSincePosix(_timeSincePosix)
{
}

locationMessage::locationMessage(uint8_t _ID, char* payload)
:message(_ID, LOCATION)
{
    int counter = 0;
    counter += sizeof(ID);
    counter += sizeof(type);
    CopyFromCharArray((char*)&latitude, sizeof(latitude), payload, counter);
    counter += sizeof(latitude);
    CopyFromCharArray((char*)&longitude, sizeof(longitude), payload, counter);
    counter += sizeof(longitude);
    CopyFromCharArray((char*)&height, sizeof(height), payload, counter);
    counter += sizeof(height);
    CopyFromCharArray((char*)&timeSincePosix, sizeof(timeSincePosix), payload, counter);
}

locationMessage::~locationMessage()
{
}


void locationMessage::toPayload(char* payload)
{
    int counter = 0;
    CopyToCharArray((char*)&ID, sizeof(ID), payload, counter);
    counter += sizeof(ID);
        CopyToCharArray((char*)&type, sizeof(type), payload, counter);
    counter += sizeof(type);
        CopyToCharArray((char*)&latitude, sizeof(latitude), payload, counter);
    counter += sizeof(latitude);
        CopyToCharArray((char*)&longitude, sizeof(longitude), payload, counter);
    counter += sizeof(longitude);
        CopyToCharArray((char*)&height, sizeof(height), payload, counter);
    counter += sizeof(height);
        CopyToCharArray((char*)&timeSincePosix, sizeof(timeSincePosix), payload, counter);
}

std::string locationMessage::toString()
{
    std::stringstream ss;
    ss <<std::setprecision(8) << std::dec << "ID[" << (int)ID << "] Type[" << (int)type << "]"<<std::endl
    << "latitude["<< latitude << "] longitude["<< longitude << "] height["<< height <<
    "] timeSincePosix[" << timeSincePosix << "]" << std::endl;

    return ss.str();
}