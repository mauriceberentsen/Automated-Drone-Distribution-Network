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
const uint64_t addresses[1] = {0xABCDABCD71LL};
const uint64_t others[2] = {0xABCDABCD51LL, 0x544d52617CLL};

uint8_t data[32];
unsigned long startTime, stopTime, counter, rxTimer = 0;
GoToLocationMessage msg(2, 3, 5, 4, 56.789, 0.123, 4);
int main(int argc, char **argv)
{
	msg.toPayload(data);

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

	while (1)
	{
		cout << "send: ";
		for (int i = 0; i < 32; i++)
		cout << (int)data[i];
		cout << endl;

		radio.openWritingPipe(others[0]);
		radio.stopListening();

		if (!radio.write(&data, 32))
		{			   //Write to the FIFO buffers
			counter++; //Keep count of failed payloads
		}
		radio.startListening();

		radio.openWritingPipe(others[1]);
		radio.txStandBy(); // This gives the PLL time to sync back up

		radio.stopListening();
		if (!radio.write(&data, 32))
		{			   //Write to the FIFO buffers
			counter++; //Keep count of failed payloads
		}

	} // loop
} // main
