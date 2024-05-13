/*radio.h*/
#ifndef DRONE_RADIO_H_GUARD
#define DRONE_RADIO_H_GUARD

#include <RF24.h>
#include <nRF24L01.h>
#include "..\..\include\common.h"

// SPI Radio Addresses
const byte controllerAddress[6] = "00002"; /*Address of this module*/
const byte droneAddress[6] = "00001"; /*Address of the other module*/

extern RF24 radio;   

/*Function Prototypes*/
void init_spi();
void rx_spi(Controller_Data_Message *msgbuf);
void tx_spi(Drone_Data_Message *msgbuf);

#endif