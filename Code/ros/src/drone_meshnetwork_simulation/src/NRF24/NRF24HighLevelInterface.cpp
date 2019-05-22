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


NRF24HighLevelInterface::NRF24HighLevelInterface(/* args */)
{
}

NRF24HighLevelInterface::~NRF24HighLevelInterface()
{
}

void NRF24HighLevelInterface::StartAntenna( uint8_t nodeID/*Communication::Wireless::IMeshNetwork* IMN*/ )
{
    low.Start(addressRange+nodeID,addressRange);
    on = true;
}

void NRF24HighLevelInterface::StopAntenna()
{

}

bool NRF24HighLevelInterface::SendMessageTo(const uint8_t *msg)
{
    return low.SendMessage(addressRange+msg[3],addressRange+msg[3]+120, msg);
}

void NRF24HighLevelInterface::BroadcastMessage(const uint8_t *msg) 
{
    low.BroadcastMessage(msg);

}

const bool NRF24HighLevelInterface::On() { return on; }