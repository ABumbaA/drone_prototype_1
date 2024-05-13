/*setup_init.h*/
#ifndef DRONE_SETUPINIT_H_GUARD
#define DRONE_SETUPINIT_H_GUARD

#include <Arduino.h>

#include "common.h"

void init_pins()
{
    // Set ESC pins as output
    // Turn off all ESCs

    /*TODO: Change to Servo library*/
}

void init_serial()
{
    Serial.begin(9600);
}

#endif
