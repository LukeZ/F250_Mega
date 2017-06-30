#ifndef F250_SETTINGS_H
#define F250_SETTINGS_H

#include <Arduino.h>


// ------------------------------------------------------------------------------------------------------------------------------------------------------->>
// SERIAL PORTS
// ------------------------------------------------------------------------------------------------------------------------------------------------------->>
    // USB Baud rate is fixed for now
    #define USB_BAUD_RATE               115200
    #define DISPLAY_BAUD_RATE           115200

// ------------------------------------------------------------------------------------------------------------------------------------------------------->>
// SIMPLE TIMER
// ------------------------------------------------------------------------------------------------------------------------------------------------------->>
    // We use the F250_SimpleTimer class for convenient timing functions throughout the project, it is a modified and improved version 
    // of SimpleTimer: http://playground.arduino.cc/Code/SimpleTimer
    // The class needs to know how many simultaneous timers may be active at any one time. We don't want this number too low or operation will be eratic, 
    // but setting it too high will waste RAM. Each additional slot costs 19 bytes of global RAM. 

    #define MAX_SIMPLETIMER_SLOTS       20          
    


#endif
