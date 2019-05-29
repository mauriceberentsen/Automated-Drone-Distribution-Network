/**
 * @file NRF24LowLevelInterface.cpp
 * @author M.W.J. Berentsen (mauriceberentsen@live.nl)
 * @brief NRF24 HighLevelInterface.hpp
 * @version 1.0
 * @date 2019-05-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <iostream>
#include <thread>

#include "../Communication/Messages/Message.hpp"

#include "NRF24HighLevelInterface.hpp"
#include "NRF24LowLevelInterface.hpp"

using namespace Communication::Messages;

NRF24LowLevelInterface::NRF24LowLevelInterface(NRF24HighLevelInterface* _highLevelInterface)
    : radio(RF24(25, 8, BCM2835_SPI_SPEED_8MHZ)),
         highLevelInterface(_highLevelInterface)
{
}

NRF24LowLevelInterface::~NRF24LowLevelInterface()
{
}

void NRF24LowLevelInterface::Start(const uint64_t _NodeIdReadAddress, 
                                   const uint64_t _broadcastAddress)
{
    radio.begin(); // Setup and configure rf radio
    setNodeIdReadAddress(_NodeIdReadAddress);
    setBroadcastAddress(_broadcastAddress);
    radio.setChannel(1);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(1);            // Ensure autoACK is enabled
    radio.setRetries(5, 15);        // Optionally, increase the delay
                                    // between retries & # of retries
    radio.setCRCLength(RF24_CRC_8); // Use 8-bit CRC for performance
    radio.printDetails();
    currentState = RECEIVING;
    this->antennaThread = std::thread(std::bind(
                        &NRF24LowLevelInterface::runAntenna, this));
}
void NRF24LowLevelInterface::BroadcastMessage(const uint8_t *message) 
{
    std::cout << "Broadcast" <<std::endl;
    currentState = SENDING;
    radio.openWritingPipe(broadcastAddress);
    radio.stopListening();
    radio.write(message, MAX_PAYLOAD, true);
    radio.startListening();
    currentState = RECEIVING;
}

bool NRF24LowLevelInterface::SendMessage(const uint64_t sendAddress,
                                         const uint64_t ackAddress,
                                         const uint8_t *message)
{
    std::cout << "Send a msg" <<std::endl;
    bool success;
    currentState = SENDING;
    radio.openReadingPipe(3, ackAddress);
    radio.openWritingPipe(sendAddress);
    radio.stopListening();
    radio.flush_tx();
    success = radio.write(message, MAX_PAYLOAD, false);
    radio.txStandBy();
    radio.startListening();
    radio.openWritingPipe(broadcastAddress);
    currentState = RECEIVING;
    return success;
}

void NRF24LowLevelInterface::runAntenna()
{
    while (currentState != OFF)
    {
        if (currentState == RECEIVING)
        {
            HandleIncomingMessages();
        }
        if (currentState == SENDING)
        {
        }
    }
}

void NRF24LowLevelInterface::HandleIncomingMessages()
{

    radio.startListening();
    static uint8_t data[100][MAX_PAYLOAD];

    if (radio.available())
    {
        static int i = 0;
        radio.read(&data[i], MAX_PAYLOAD);
        highLevelInterface->OnMessage(data[i]);
        ++i;
        i = i % 100;
    }
}

void NRF24LowLevelInterface::setNodeIdReadAddress(const uint64_t
                                                 _NodeIdReadAddress)
{
    this->NodeIdReadAddress = _NodeIdReadAddress;
    radio.openReadingPipe(2, _NodeIdReadAddress);
}
void NRF24LowLevelInterface::setBroadcastAddress(const uint64_t 
                                                _broadcastAddress)
{
    this->broadcastAddress = _broadcastAddress;
    radio.openReadingPipe(1, _broadcastAddress);
}
