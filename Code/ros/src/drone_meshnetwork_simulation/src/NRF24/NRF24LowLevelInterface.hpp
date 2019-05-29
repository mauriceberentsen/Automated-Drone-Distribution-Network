/**
 * @file NRF24LowLevelInterface.hpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief NRF24 HighLevelInterface.hpp
 * @version 1.0
 * @date 2019-05-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef NRF24LOWLEVELINTERFACE
#define NRF24LOWLEVELINTERFACE


#include <RF24/RF24.h>
#include <thread>


//#include "NRF24HighLevelInterface.hpp"

class NRF24HighLevelInterface;
enum state {SENDING,RECEIVING,OFF};

class NRF24LowLevelInterface
{
public:
    explicit NRF24LowLevelInterface(NRF24HighLevelInterface* _highLevelInterface);
    ~NRF24LowLevelInterface();

    void Start(const uint64_t _NodeIdReadAddress,const  uint64_t _broadcastAddress);
    void BroadcastMessage(const  uint8_t* message);
    bool SendMessage(const uint64_t sendAddress,const  uint64_t ackAddress,const uint8_t* message);

private:
    void runAntenna();
    void HandleIncomingMessages ();
    void setNodeIdReadAddress(const uint64_t _NodeIdReadAddress);
    void setBroadcastAddress(const uint64_t _broadcastAddress);
    state currentState = OFF;
    RF24 radio;
    std::thread antennaThread;
    uint64_t NodeIdReadAddress;
    uint64_t broadcastAddress;
    NRF24HighLevelInterface* highLevelInterface;
};




#endif //NRF24LOWLEVELINTERFACE
