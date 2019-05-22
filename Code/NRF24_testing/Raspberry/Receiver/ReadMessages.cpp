/*
TMRh20 2014

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/** General Data Transfer Rate Test
 * This example demonstrates basic data transfer functionality with the
 updated library. This example will display the transfer rates acheived using
 the slower form of high-speed transfer using blocking-writes.
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <RF24/RF24.h>
#include <unistd.h>
#include "Message.hpp"

using namespace std;
using namespace Communication::Messages;
//
// Hardware configuration
//

/****************** Raspberry Pi ***********************/

RF24 radio(25, 8, BCM2835_SPI_SPEED_8MHZ);


// Radio pipe addresses for the 2 nodes to communicate.
//const uint64_t addresses[2] = {0xABCDABCE51LL,0xABCDABCD71LL};
//const uint64_t others[2] = {0x544d52687CLL, 0x544d52617CLL};
//const uint64_t addresses[2] = { 0xABCDABCD71LL, 0x544d52687CLL };
const uint64_t addresses[3] = { 0x544d526802LL, 0x544d526801LL,0x7878787878LL };

uint8_t data[32];
unsigned long startTime, stopTime, counter, rxTimer = 0;

int main(int argc, char **argv)
{
	

	// Print preamble:

	cout << "RF24/examples/Transfer/\n";

	radio.begin(); // Setup and configure rf radio
	radio.setChannel(1);
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(1);			// Ensure autoACK is enabled
	radio.setRetries(2, 15);		// Optionally, increase the delay between retries & # of retries
	radio.setCRCLength(RF24_CRC_8); // Use 8-bit CRC for performance
	radio.printDetails();
	/********* Executable ***********/

	radio.openReadingPipe(1, addresses[0]);
	radio.openReadingPipe(2, addresses[2]);
	radio.openWritingPipe(addresses[1]);
	radio.startListening();
	while (1)
	{

     	while(radio.available()){
      	radio.read(&data,32);
      	counter++;
     	
	GoToLocationMessage msg(data);
	 cout << msg.toString() <<std::endl;
}
   }
  }
