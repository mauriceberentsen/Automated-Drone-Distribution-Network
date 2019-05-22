
/*
  Getting Started example sketch for nRF24L01+ radios
  This is an example of how to send data from one node to another using data structures
  Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Message.hpp"

uint64_t addresses[2] = {0x544d52617CLL, 0xABCDABCD71LL};


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9, 10);
/**********************************************************/


// Used to control whether this node is sending or receiving
bool role = 0;

/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
uint8_t myData[32];
uint8_t* dataPointer;

void setup() {

  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted_HandlingData"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  dataPointer = myData;
  radio.begin(); // Setup and configure rf radio
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(2, 15);        // Optionally, increase the delay between retries & # of retries
  radio.setCRCLength(RF24_CRC_8); // Use 8-bit CRC for performance
  radio.printDetails();


  // Open a writing and reading pipe on each radio, with opposite addresses
  if (radioNumber) {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  } else {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();
}




void loop() {






  /****************** Pong Back Role ***************************/

  if ( role == 0 )
  {

    if ( radio.available()) {
      // Variable for the received timestamp
      while (radio.available()) {                          // While there is data ready
        radio.read( &myData, 32 );             // Get the payload
      }

      radio.stopListening();                               // First, stop listening so we can talk
      // Increment the float value
      radio.write( &myData, 32 );              // Send the final one back.
      radio.startListening();
      Communication::Messages::GoToLocationMessage msg(dataPointer);// Now, resume listening so we catch the next packets.
      Serial.print(msg.getCreator());
      Serial.print(" ");
      Serial.print(msg.getFrom());
      Serial.print(" ");
      Serial.print(msg.getMessageType());
      Serial.print(" ");
      Serial.print(msg.getForward());
      Serial.print(" ");
      Serial.print(msg.getLatitude());
      Serial.print(" ");
      Serial.print(msg.getLongitude());
      Serial.print(" ");
      Serial.println(msg.getHeight());

    }
  }




  /****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ) {
      Serial.print(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)

    } else if ( c == 'R' && role == 1 ) {
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      role = 0;                // Become the primary receiver (pong back)
      radio.startListening();

    }
  }


} // Loop
