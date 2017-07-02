
// Adafruit Ultimate GPS
// https://learn.adafruit.com/adafruit-ultimate-gps
// - Red LED on the unit blinks at 1 Hz on / 1 Hz off while searching for satellites, blinks about once every 15 seconds once a fix is obtained, to save energy. 
// - GPS_EN  - Pull enable pin to ground to turn off the unit
// - GPS_FIX - pin is attached to the same source as the onboard Red LED. So when a fix is available it will pulse high 200mS every 15 seconds, otherwise up and down once every second (one second on, one second off)
// - GPS_PPS - Pulse-per-second output. Most of the time it is at logic low (ground) and then it pulses high (3.3V) once a second, for 50-100ms
// - GPS sentence details: https://learn.adafruit.com/adafruit-ultimate-gps/direct-computer-wiring
// - Time is GMT HHMMSS.milliseconds (24 hr time)
// - N and E are positive, S and W are negative, Google maps requires +/- instead of NSWE
// - Despite appearances, the geolocation data is NOT in decimal degrees. It is in degrees and minutes in the following format: 
//      Latitude: DDMM.MMMM (The first two characters are the degrees.) 
//      Longitude: DDDMM.MMMM (The first three characters are the degrees.)
// - Current date comes in DDMMYY format
// - External antenna is automatically detected. You can request PGTOP sentence, $PGTOP,11,x*YZ where x is the status number. 
//      x = 3 means it is using the external antenna
//      x = 2 using the internal antenna 
//      x = 1 there was an antenna short or problem.
//   You can actually just use the parser command GPS.sendCommand(PGCMD_ANTENNA) during setup to request this sentence or PGCMD_NOANTENNA to stop it
//   The external antenna is more sensitive (and necessary in my case), but it does draw an extra 10-20mA current when GPS is enabled. 

void TurnOnGPS()
{
    // Set the Enable pin high
    digitalWrite(GPS_EN, LOW);
        delay(500);
    digitalWrite(GPS_EN, HIGH);        
        delay(100);

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
    GPS.begin(9600);

    // We do this so there is no chance it's going to start back up at 57600, we can always be sure it's at 9600
    // EDIT: it appears this isn't necessary, the rest of my code is working fine even with the battery backup and begin-ing at 9600. 
    // Warm/hot starts or whatever is default are much, much faster and more convenient. 
//    GPS.sendCommand(PMTK_CMD_COLD_START);

    // We want a faster baud rate, and might as well go all out! 
    // Finally got it to work using these delays and order, but flushed about an entire day down the drain to do so. https://forums.adafruit.com/viewtopic.php?f=25&t=47279
    // One thing I discovered is you can't use Adafruit's simple 1 mS interrupt, it's too slow to keep up with five fixes each second. The interrupt is fine for reading sentences, but doesn't leave you time for parsing. 
    // In the Adafruit support forums the suggestion was to move the GPS.read() statement to the main loop, assuming the main loop takes less than 1 mS to loop. That's fine but not very sophisticated. 
    // I got it to work by creating an interrupt on the Compare A of Timer 1 and setting Timer 1 to tick every 1/2 uS (that's 2000 times faster than every 1mS). You can choose how often you want the compare to interrupt
    // by adding some number to TCNT1, but at 1/2 uS per tick there is a very fine resolution. It seems 
    // I got the parsing to work at 5Hz doing the reading in the main loop. But who only knows what side effects this might have on other tasks... 1 Hz is definitely the safest route. 
    delay(1000);
    GPS.sendCommand(PMTK_SET_BAUD_115200);
    delay(1000);
    GPS_Serial.end();
    delay(1000);
        
    // Now reset the port to 115200
    GPS.begin(115200);
        delay(100);
        
    // You can do both RMC (recommended minimum) and GGA (fix data including altitude) or just RMC   -- We want both because I intend to use altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    
        delay(200);
   
    // Set the update rate. Note that this only controls the rate at which the position is echoed, to actually speed up the
    // position fix you must also send one of the position fix rate commands (below)
    if (GPS_FAST_UPDATE)    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);              // 5 Hz update rate
    else                    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);              // 1 Hz update rate
        delay(200);
    
    // Here we tell the GPS to fix our position at 5hz as well. 
    if (GPS_FAST_UPDATE)   GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);               // 5 Hz fix rate, the fastest possible
    else                   GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);               // 1 Hz fix rate
   
    // Adafruit note - For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
    // Luke note - since we aren't asking for all and we aren't printing it all out either (just speed, direction and altitude to the screen), and since we are using
    // hardware serial and not some bit-banged software crap, and since I would like the speedo to update faster than once per second, I am setting the above to the 
    // fastest possible. Even so, I will probably only send updates to the display twice a second instead of trying to keep up with 5Hz. But if you have problems, 
    // set GPS_FAST_UPDATE to false in the main tab.
    
    // Request updates on antenna status, comment out to keep quiet. We will turn this on at the beginning but once antenna status has been determined we will 
    // turn it off since we don't need it continuously (use PGCMD_NOANTENNA)
    GPS.sendCommand(PGCMD_ANTENNA);     
        delay(100); 
    GPS.sendCommand(PGCMD_ANTENNA);     // This doesn't always work on the first try   
   
    // We set up an interrupt on Timer 1 Compare A, you can set the frequency it trips with the GPS_CHECK_uS define on the main tab. 
    // The Adafruit interrupt using Timer 0 at 1 mS was not fast enough if you go above 1 Hz update rates!
    Enable_GPS_Interrupt(true);                            
    
    // Ask for firmware version - I DON'T THINK I CARE ABOUT THIS
//        delay(1000);
//        GPS_Serial.println(PMTK_Q_RELEASE);

    if (DEBUG)
    {
        DebugSerial->print(F("GPS turned on (Update rate: ")); 
        if (GPS_FAST_UPDATE) DebugSerial->println(F("5 Hz)"));
        else                 DebugSerial->println(F("1 Hz)"));
    }
}


// ISR
// Timer1 Output Compare A interrupt service routine
// Interrupt is called at some interval you define by adding a number to TCNT1. 
// When the ISR trips, it looks for new GPS data on the serial port, and stores it if any is found (this doesn't parse sentences)
ISR(TIMER1_COMPA_vect)
{
    char c = GPS.read();            // We don't really care about the returned char. All we're doing here is adding to the sentence in the GPS library with each char. 
                                    // The read() routine will set a library-internal flag called recvdflag to true when a complete sentence is received. This will cause
                                    // cause GPS.newNMEAreceived() to return true, after which the loop can then decide to call parse() and we can see if the sentence was
                                    // valid. If we don't ask to parse that's fine, the library will keep collecting new sentences whether we use them or not. 
    
    // But if you want to debug, this is a good time to do it! This will only occur if GPSECHO is set to true
#ifdef UDR0
    if (GPSECHO)
        if (c) UDR0 = c;            // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time. 
#endif        

    // This sets the interrupt to occur again. 
    OCR1A = TCNT1 + (GPS_CHECK_uS * TIMER1_TICKS_PER_uS);       // Interrupt again in some number of uS 
}


void Enable_GPS_Interrupt(boolean i) 
{
    if (i)
    {
        // We set up Timer 1 in Normal Mode: count starts from BOTTOM (0), goes to TOP (0xFFFF / 65,535), then rolls over. 
        // We set prescaler to 8. With a 16MHz clock that gives us 1 clock tick every 0.5 uS (0.0000005 seconds).
        // The rollover will occur roughly every 32.7 mS (0.0327 seconds). These settings are dictated by TCCR1A and TCCR1B.
        // We also clear all interrupt flags to start (write 1 to respective bits of TIFR1). 
        // And we start off with all interrupts disabled (write 0 to all bits in TIMSK1). 
            TCCR1A = 0x00;       
            TCCR1B = 0x02;       
            TIFR1 =  0x2F;       
            TIMSK1 = 0x00;       
            TCNT1 = 0;        
        
        // Enable Timer 1 Output Compare A interrupt
        TIMSK1 |= (1 << OCIE1A);        // TIMSK1, bit OCIE1A = Output Compare Interrupt Enable 1 A. Set bit.
                                        // Set this flag to one to enable an interrupt to occur when the TCNT1 equals the Timer 1 Compare A value (OCR1A)
        OCR1A = TCNT1 + (GPS_CHECK_uS * TIMER1_TICKS_PER_uS);       // Interrupt again in some number of uS 
    }
    else
    {
        // Clear any interrupt flags
        TIFR1 =  0x2F;
        // Disable Timer 1 Output Compare A interrupt
        TIMSK1 &= ~(1 << OCIE1A);       // TIMSK1, bit OCIE1A = Output Compare Interrupt Enable 1 A. Clear bit.
                                        // Set this flag to zero to disable an interrupt to occur when the TCNT1 equals the Timer 1 Compare A value (OCR1A)
    }
}

void CheckGPS_ForData(void)
{
    // If a complete GPS sentence has been received see if we can parse it
    if (GPS.newNMEAreceived()) 
    {   
        if (GPS.parse(GPS.lastNMEA())) UpdateGPSData();     // GPS.parse also sets the newNMEAreceived() flag to false. UpdateGPSData will save data to our local variables.
    }                                                       // This means our local variables are udpated as soon as new data comes in. We can choose a different rate for 
                                                            // sending them to the display. 
}

void UpdateGPSData(void)
{
    // This function gets called when the car is on and Polls finds new NMEA sentence has been received and successfully parsed. 
    // All we do here is save the cleaned up data to our local variables, but we will use a different function to actually send
    // the info to the display, so we can do that at a different rate. 

/*    
    if (DEBUG)
    {
        DebugSerial->print("\nTime: ");
        DebugSerial->print(GPS.hour, DEC); DebugSerial->print(':');
        DebugSerial->print(GPS.minute, DEC); DebugSerial->print(':');
        DebugSerial->print(GPS.seconds, DEC); DebugSerial->print('.');
        DebugSerial->println(GPS.milliseconds);
        DebugSerial->print("Date: ");
        DebugSerial->print(GPS.day, DEC); DebugSerial->print('/');
        DebugSerial->print(GPS.month, DEC); DebugSerial->print("/20");
        DebugSerial->println(GPS.year, DEC);
        DebugSerial->print("Fix: "); DebugSerial->print((int)GPS.fix);
        DebugSerial->print(" quality: "); DebugSerial->println((int)GPS.fixquality); 
    }
*/

    if (GPS_Antenna_Status_Known == false)
    {   
        GPS_Antenna_Status = GPS.getAntennaStatus();
        if (GPS_Antenna_Status != ANTENNA_STAT_UNKNOWN)
        {   
            if (DEBUG) { DebugSerial->print(F("GPS antenna status: ")); }
            switch (GPS_Antenna_Status)
            {
                case ANTENNA_STAT_ERROR:    if (DEBUG) { DebugSerial->println(F("Error!"));   } break;
                case ANTENNA_STAT_INTERNAL: if (DEBUG) { DebugSerial->println(F("Internal")); } break;
                case ANTENNA_STAT_EXTERNAL: if (DEBUG) { DebugSerial->println(F("External")); } break;
            }
            // Now that we know it, we can tell it to quit reporting this information
            GPS.sendCommand(PGCMD_NOANTENNA);
            // Now we know
            GPS_Antenna_Status_Known = true;
        }
    }
    
    if (GPS.fix && GPS_FirstFix == false)   
    {
        // This is the first sentence with a fix. Check the antenna status.         
        GPS_FirstFix = true;    
        if (DEBUG) DebugSerial->println(F("GPS Fix Acquired")); 
    }

    if (GPS.fix)
    {
        if (DEBUG)
        {
            /*
            DebugSerial->print("Location: ");
            DebugSerial->print(GPS.latitude, 4); DebugSerial->print(GPS.lat);
            DebugSerial->print(", "); 
            DebugSerial->print(GPS.longitude, 4); DebugSerial->println(GPS.lon);
            DebugSerial->print("Location in degrees: ");
            DebugSerial->print(GPS.latitudeDegrees, 4);
            DebugSerial->print(", "); 
            DebugSerial->println(GPS.longitudeDegrees, 4);
            
            DebugSerial->print("Speed (knots): "); DebugSerial->println(GPS.speed);
            DebugSerial->print("Angle: "); DebugSerial->println(GPS.angle);
            DebugSerial->print("Altitude: "); DebugSerial->println(GPS.altitude);
            DebugSerial->print("Satellites: "); DebugSerial->println((int)GPS.satellites);
            */
        }
    }
}

void TurnOffGPS(void) 
{
    digitalWrite(GPS_EN, LOW);          // Turn off the GPS
    Enable_GPS_Interrupt(false);        // Stop the routine interrupt that checks for serial data on the GPS serial port (there won't be any anyway)
    GPS_FirstFix = false;               // Reset the fix flag
    GPS_Antenna_Status_Known = false;   // Reset the antenna known flag
    if (DEBUG) DebugSerial->println(F("GPS turned off")); 
}



