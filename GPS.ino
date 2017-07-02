
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
    GPS.sendCommand(PMTK_CMD_COLD_START);
    
    //II TRY THIS
    delay(1000);
    GPS.sendCommand(PMTK_SET_BAUD_57600); 
    delay(1000);
    GPS_Serial.end();
    delay(1000);
        
    // Now reset the port to 57600
    GPS.begin(57600); 
        delay(100);
        
    // You can do both RMC (recommended minimum) and GGA (fix data including altitude) or just RMC   -- We want both because I intend to use altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    
        delay(200);
   
    // Set the update rate. Note that this only controls the rate at which the position is echoed, to actually speed up the
    // position fix you must also send one of the position fix rate commands (below)
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);              // 1 Hz update rate
     GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);              // 5 Hz update rate
        delay(200);
    
    // Here we tell the GPS to fix our position at 5hz as well. 
     GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);              // 5 Hz fix rate, the fastest possible
   
    // Adafruit note - For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
    // Luke note - since we aren't asking for all and we aren't printing it all out either (just speed, direction and altitude to the screen), and since we are using
    // hardware serial and not some bit-banged software crap, and since I would like the speedo to update faster than once per second, I am setting the above to the 
    // fastest possible. Even so, I will probably only send updates to the display twice a second instead of trying to keep up with 5Hz. But if you have problems, 
    // drop the above down to 1Hz. 
    
    // Request updates on antenna status, comment out to keep quiet. We will turn this on at the beginning but once antenna status has been determined we will 
    // turn it off since we don't need it continuously (use PGCMD_NOANTENNA)
    GPS.sendCommand(PGCMD_ANTENNA);     // This doesn't always work on the first try
        delay(100); 
    GPS.sendCommand(PGCMD_ANTENNA);        
   
    // The nice thing about this code is you can have a timer0 interrupt go off every 1 millisecond and read data from the GPS for you. 
    // That makes the loop code a heck of a lot easier!
//    Enable_GPS_Interrupt(true);
    Enable_GPS_Interrupt(false);

    // Ask for firmware version - I DON'T THINK I CARE ABOUT THIS
//        delay(1000);
//        GPS_Serial.println(PMTK_Q_RELEASE);
    
}


// ISR
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
ISR(TIMER0_COMPA_vect) 
{
    char c = GPS.read();            // We don't really care about the returned char. All we're doing here is adding to the sentence in the GPS library with each char. 
                                    // The read() routine will set a library-internal flag called recvdflag to true when a complete sentence is received. This will cause
                                    // cause GPS.newNMEAreceived() to return true, after which the loop can then decide to call parse() and we can see if the sentence was
                                    // valid. If we don't ask to parse that's fine, the library will keep collecting new sentences whether we use them or not. 
    // If you want to debug, this is a good time to do it! This will only occur if GPSECHO is set to true
#ifdef UDR0
    if (GPSECHO)
        if (c) UDR0 = c;            // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time. 
#endif    
}

void Enable_GPS_Interrupt(boolean i) 
{
    if (i) 
    {
        // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the "Compare A" function above
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
        GPS_Interrupt_Active = true;
    } 
    else 
    {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        GPS_Interrupt_Active = false;
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
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
*/

    if (GPS_Antenna_Status_Known == false)
    {   
        GPS_Antenna_Status = GPS.getAntennaStatus();
        if (GPS_Antenna_Status != ANTENNA_STAT_UNKNOWN)
        {   
            Serial.print(F("GPS Antenna status: ")); 
            switch (GPS_Antenna_Status)
            {
                case ANTENNA_STAT_ERROR:    Serial.println(F("Error!"));  break;
                case ANTENNA_STAT_INTERNAL: Serial.println(F("Internal"));  break;
                case ANTENNA_STAT_EXTERNAL: Serial.println(F("External"));  break;
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
        Serial.println(F("GPS Fix Acquired")); 
    }

    if (GPS.fix)
    {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", "); 
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", "); 
        Serial.println(GPS.longitudeDegrees, 4);
        
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}

void TurnOffGPS(void) 
{
    digitalWrite(GPS_EN, LOW);          // Turn off the GPS
    Enable_GPS_Interrupt(false);               // Stop the routine interrupt that checks for serial data on the GPS serial port (there won't be any anyway)
}



