
/*
  Getting Started example sketch for nRF24L01+ radios
  This is an example of how to send data from one node to another using data structures
  Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Message.hpp"

uint64_t addresses[2] = {0x544d52617CLL, 0x7878787878LL};


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9, 10);
/**********************************************************/


// Used to control whether this node is sending or receiving
bool role = 0;
Communication::Messages::GoToLocationMessage msg(6, 6, 6, 6, 66.666, 6.666, 6);


/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
uint8_t myData[32];


void setup() {

  Serial.begin(115200);
  radio.begin(); // Setup and configure rf radio
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(2, 15);        // Optionally, increase the delay between retries & # of retries
  radio.setCRCLength(RF24_CRC_8); // Use 8-bit CRC for performance
  radio.printDetails();


    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();
}




void loop() {
    msg.toPayload(myData);




    if ( radio.available()) {
      // Variable for the received timestamp


      radio.stopListening();                               // First, stop listening so we can talk
      // Increment the float value
      radio.write( &myData, 32 );              // Send the final one back.
      radio.startListening();
      
    }

} // Loop
