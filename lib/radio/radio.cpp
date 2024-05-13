#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "radio.h"
#include "..\..\include\common.h"

void init_spi()
{
    radio.begin();
    radio.setDataRate(RF24_2MBPS); // Set data rate to 2 Mbps
    radio.setPALevel(RF24_PA_MIN);
   
    //radio.openWritingPipe(droneAddress);
    radio.openReadingPipe(1, controllerAddress);

    radio.startListening();
}

void rx_spi(Controller_Data_Message *msgbuf)
{
    Serial.println("In rx_spi");
    if (radio.available()) {
        radio.startListening();
        radio.read(msgbuf, sizeof(Controller_Data_Message));
    }
    else
        Serial.println("No Radio");
}

void tx_spi(Drone_Data_Message *msgbuf)
{
    radio.stopListening();
    radio.write(&msgbuf, sizeof(Drone_Data_Message));
}
