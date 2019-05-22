/**
 * @file HighLevelInterface.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief NRF24 HighLevelInterface.hpp
 * @version 1.0
 * @date 2019-05-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "NRF24HighLevelInterface.hpp"
#include "../Communication/Messages/Message.hpp"
#include "NRF24LowLevelInterface.hpp"

using namespace Communication::Messages;

NRF24HighLevelInterface::NRF24HighLevelInterface()
:lowLevelInterface(new NRF24LowLevelInterface(this))
{
}

NRF24HighLevelInterface::~NRF24HighLevelInterface()
{
}

void NRF24HighLevelInterface::StartAntenna( Communication::Wireless::IMeshNetwork* IMN )
{
    meshnetworkComponent = IMN;
    uint8_t node = meshnetworkComponent->getNodeID();
    lowLevelInterface->Start(addressRange+node,addressRange);
    on = true;
}

void NRF24HighLevelInterface::StopAntenna()
{

}

bool NRF24HighLevelInterface::SendMessageTo(const uint8_t *msg)
{
    return lowLevelInterface->SendMessage(addressRange+msg[TO],addressRange+msg[TO]+ackStart, msg);
}

void NRF24HighLevelInterface::BroadcastMessage(const uint8_t *msg) 
{
    lowLevelInterface->BroadcastMessage(msg);

}

const bool NRF24HighLevelInterface::On(){ return on; }

 void NRF24HighLevelInterface::DebugingMode(
     Communication::Wireless::IMeshDebugInfo* debug, const bool on )
 {
     debuginfo = debug;
 }

 void NRF24HighLevelInterface::OnMessage(const uint8_t* message)
 {
     meshnetworkComponent->OnMsg(message);
 }

