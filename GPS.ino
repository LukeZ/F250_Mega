
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

    // Finally, we start a routine to send data to the display at some routine interval. Until we get a GPS fix it won't have much to send, but it will take care of that. 
    if (TimerID_GPSSender == 0) TimerID_GPSSender = timer.setInterval(GPS_SEND_FREQ, SendGPSInfo);            
    else if (timer.isEnabled(TimerID_GPSSender) == false) timer.enable(TimerID_GPSSender);

    // Let us know
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

    // Antenna status
    // ------------------------------------------------------------------------------------------------------------------------------------->>
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
    
    // First Fix Tasks
    // ------------------------------------------------------------------------------------------------------------------------------------->>
    if (GPS.fix && GPS_FirstFix == false)   
    {
        // This is the first sentence with a fix. 
        GPS_FirstFix = true;    
        GPS_Fixed = true;

        // The first fix can be a bit wonky. We wait a little bit for a steady connection, then evaluate our position against the saved home position
        timer.setTimeout(2000, CheckLocationAgainstHome);

        if (DEBUG) DebugSerial->println(F("GPS Fix Acquired")); 
    }

    // Routine Tasks (with Fix)
    // ------------------------------------------------------------------------------------------------------------------------------------->>    
    if (GPS.fix)
    {
        // Calculate MPH
        // ------------------------------------------------------------------------------------------------
        GPS_Avg_Speed_Knots = fir_basic(GPS.speed, GPS_SPEED_NTAPS, GPSSpeedFIR);           // Update averaged reading (in knots)
        MPH = constrain((uint8_t)(GPS_Avg_Speed_Knots + 0.5), 0, 255);                      // Round to nearest MPH and constrain to 255
        if (MPH < Minimum_MPH) MPH = 0;                                                     // If below a threshold amount, we ignore the speed as GPS noise

        // Save coordinates
        // ------------------------------------------------------------------------------------------------
        Current_Latitude = GPS.latitudeDegrees;
        Current_Longitude = GPS.longitudeDegrees;

        // Altitude
        // ------------------------------------------------------------------------------------------------
        GPS_Altitude_Meters = GPS.altitude;
    }
}

void SendGPSInfo(void)
{
    // Here we send GPS data to the display, if there is any. This is not the same as collecting, parsing, massaging data from the GPS, which takes care of itself elsewhere

    // Fix Status Change
    // ------------------------------------------------------------------------------------------------------------------------------------->>        
    if (GPS.fix != GPS_Fixed)
    {
        if (GPS.fix)
        {
            // Fix acquired or re-acquired
        }
        else
        {
            // Fix lost
        }
        GPS_Fixed = GPS.fix;
        
    }


    // Fix Status (we always send this regardless)
    // ------------------------------------------------------------------------------------------------------------------------------------->>        
    SendDisplay(CMD_GPS_FIX, GPS.fix, GPS.fixquality);          // Fix is true or false (0 or 1), Modifier holds fix quality (0 = no fix, 1 = GPS, 2 = DGPS meaning very accurate, it shows my actual bedroom on Google maps if quality = 2)


    // Stuff we send only with Fix
    // ------------------------------------------------------------------------------------------------------------------------------------->>    
    if (GPS_Fixed)
    {
        // Satellites
        // ------------------------------------------------------------------------------------------------
        SendDisplay(CMD_GPS_SATELLITES, GPS.satellites);

        // Date and time
        // ------------------------------------------------------------------------------------------------
        AdjustGPSDateTime();                                    // This saves an adjusted time/date in local variables, in case we care. Accounts for time zone, DST and leap year
        SendDisplay(CMD_GPS_YEAR, DateTime.Year);               // Value holds year after 2000
        SendDisplay(CMD_MONTH_DAY, DateTime.Month, DateTime.Day);       // Value holds month (1-12), Modifier holds day (0-31)
        SendDisplay(CMD_HOUR_MINUTE, DateTime.Hour, DateTime.Minute);   // Value holds hour (0-23), Modifier holds minute (0-59)
        
        // Speed & Max speed 
        // ------------------------------------------------------------------------------------------------
        if (MPH > Max_MPH)
        {
            SendDisplay(CMD_SPEED_MPH, MPH, 1);                 // Modifier 1 means this is a new max
            Max_MPH = MPH;
        }
        else
        {
            SendDisplay(CMD_SPEED_MPH, MPH, 0);                 // Modifier 0 means this is regular speed
        }

        // Altitude
        // ------------------------------------------------------------------------------------------------
        int16_t iAlt = (int16_t)(abs(MetersToFeet(GPS_Altitude_Meters)) + 0.5);      // Convert to rounded absolute integer in feet
        uint8_t Alt_Tens = iAlt % 100;                          // Number of feet under 100
        uint8_t Alt_Hundreds = iAlt / 100;                      // Number of feet over 100, divided by 100
        if (GPS_Altitude_Meters < 0) SendDisplay(CMD_GPS_ALTITUDE_POS, Alt_Hundreds, Alt_Tens); 
        else                         SendDisplay(CMD_GPS_ALTITUDE_NEG, Alt_Hundreds, Alt_Tens); 
        // Test putting the number back together - yes, it works
        // if (DEBUG) { Serial.print(F("Altitude (feet): ")); Serial.println((Alt_Hundreds * 100) + Alt_Tens); }

        // Heading in degrees and course
        // ------------------------------------------------------------------------------------------------
        // 


        if (DEBUG)
        {
            DebugSerial->print(F("Fix: ")); DebugSerial->print((int)GPS.fix); DebugSerial->print(F(" quality: ")); DebugSerial->print((int)GPS.fixquality); DebugSerial->print(F(" satellites: ")); DebugSerial->println((int)GPS.satellites);
            DebugSerial->print(DateTime.Month, DEC); DebugSerial->print('/'); DebugSerial->print(DateTime.Day, DEC); DebugSerial->print("/20"); DebugSerial->print(DateTime.Year, DEC);
            DebugSerial->print(" ");            
            DebugSerial->print(DateTime.Hour, DEC); DebugSerial->print(':'); DebugSerial->print(DateTime.Minute, DEC); DebugSerial->print(':'); DebugSerial->println(DateTime.Second, DEC); 
            DebugSerial->print(F("Coordinates: ")); DebugSerial->print(GPS.latitudeDegrees, 4); DebugSerial->print(F(", ")); DebugSerial->println(GPS.longitudeDegrees, 4);
            DebugSerial->print(F("MPH: ")); DebugSerial->println(MPH);
            DebugSerial->print(F("Angle: ")); DebugSerial->println((int16_t)(GPS.angle + 0.5));
            DebugSerial->print(F("Course: ")); DebugSerial->println(cardinal(GPS.angle));
            DebugSerial->print(F("Altitude: ")); DebugSerial->print(MetersToFeet(GPS.altitude), 0); DebugSerial->println(" ft");
            DebugSerial->println();
        }
    }
}

void TurnOffGPS(void) 
{
    digitalWrite(GPS_EN, LOW);          // Turn off the GPS
    Enable_GPS_Interrupt(false);        // Stop the routine interrupt that checks for serial data on the GPS serial port (there won't be any anyway)
    GPS_FirstFix = false;               // Reset the fix flag
    GPS_Antenna_Status_Known = false;   // Reset the antenna known flag
    if (TimerID_GPSSender > 0) timer.disable(TimerID_GPSSender);   // If the GPS is turned off we don't need to send info the display (GPS off is not the same as GPS doesn't have a fix)
    if (DEBUG) DebugSerial->println(F("GPS turned off")); 
}


void CheckLocationAgainstHome(void)
{
    // See how close we are to home, and load our home-based saved altitude adjustment if we are near enough
    float dist_to_home = distanceBetween(eeprom.ramcopy.Lat_Home, eeprom.ramcopy.Lon_Home, GPS.latitudeDegrees, GPS.longitudeDegrees);  // returns distance in meters
    
    if ((uint32_t)abs(dist_to_home) <= MAX_DISTANCE_COUNTS_AS_HOME)
    {
        startAtHome = true;
        // DO ALTITUDE STUFF HERE
        // ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
        // ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
        // ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    }

    if (DEBUG) 
    {
        DebugSerial->print(F("Distance from home: "));
        if (dist_to_home < 1600.0)
        {
            DebugSerial->print(MetersToFeet(dist_to_home),0);
            DebugSerial->println(F(" feet"));                
        }
        else
        {
            DebugSerial->print(MetersToMiles(dist_to_home),1);
            DebugSerial->println(F(" miles"));
        }
    }
}

// Distance between two GPS coordinates
// From TinyGPSPlus https://github.com/mikalhart/TinyGPSPlus
// For lat/long, use the versions in signed degrees from Adafruit's library
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *cardinal(double course)
{
  // Returns a friendly course name if passed a heading where North = 0, West = 270
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

float MetersToMiles(float meters)
{
    float dist = MetersToKm(meters);    // Convert to km
    return (dist * 0.621371);           // Convert to miles
}

float MetersToKm(float meters)
{
    return (meters / 1000.0);           // Convert to km
}

float MetersToFeet(float meters)
{
    return (meters * 3.28084);
}

float KnotsToMPH(float knots)
{
    return (knots * 1.150779);
}

void AdjustGPSDateTime()
{
    // Only call this if you have a GPS fix
    
    DateTime.Hour = GPS.hour;
    DateTime.Minute = GPS.minute;
    DateTime.Second = GPS.seconds;
    DateTime.Year = GPS.year;       // 2-digit
    DateTime.Month = GPS.month;
    DateTime.Day = GPS.day;


    // Leap year adjustment
    if (DateTime.Year % 4 == 0) DaysInMonth[1] = 29; 
          
    //Time zone adjustment
    DateTime.Hour += UTC_Offset[eeprom.ramcopy.Timezone];  // Timezone is AKST, PST, MST, CST, EST, and the array holds the UTC offsets for each one

    //DST adjustment
    if (DateTime.Month * 100 + DateTime.Day >= DSTbegin[DateTime.Year - 13] && DateTime.Month * 100 + DateTime.Day < DSTend[DateTime.Year - 13]) DateTime.Hour += 1;
    
    if (DateTime.Hour < 0)
    {
        DateTime.Hour += 24;
        DateTime.Day -= 1;
        if (DateTime.Day < 1)
        {
            if (DateTime.Month == 1)
            {
                DateTime.Month = 12;
                DateTime.Year -= 1;
            }
            else
            {
                DateTime.Month -= 1;
            }
            DateTime.Day = DaysInMonth[DateTime.Month - 1];
        }
    }
    
    if (DateTime.Hour >= 24)
    {
        DateTime.Hour -= 24;
        DateTime.Day += 1;
        if (DateTime.Day > DaysInMonth[DateTime.Month - 1])
        {
            DateTime.Day = 1;
            DateTime.Month += 1;
            if (DateTime.Month > 12) DateTime.Year += 1;
        }
    }

    // To see current adjusted date/time to make sure it works - yes, it does
    /*
    if (DEBUG)
    {
        DebugSerial->print(F("Current Adjusted Time: "));
        DebugSerial->print(DateTime.Month, DEC);
        DebugSerial->print('/');
        DebugSerial->print(DateTime.Day, DEC); 
        DebugSerial->print("/20");
        DebugSerial->print(DateTime.Year, DEC);
        DebugSerial->print(" ");            
        DebugSerial->print(DateTime.Hour, DEC); DebugSerial->print(':');
        DebugSerial->print(DateTime.Minute, DEC); DebugSerial->print(':');
        DebugSerial->println(DateTime.Second, DEC); 
    }
    */
}


void TestDistCalcs(void)
{
// Coordinates - Mom and Dad's house    According to Google, 3.62 miles from their house to my apartment
// Latitude: 37.65356   (North)
// Longitude: -97.45796 (West)

// Coordinates - Huntington Park Apartments
// Latitude: 37.7060    (North)
// Longitude: -97.4643  (West)

// Coordinates - Storage Depot Wichita
// Latitude 37.6683     (North)
// Longitude -97.44399  (West)
    float dist; 

    // My apartment
    float flat1 = 37.7060; 
    float flon1 = -97.4643;

    // Mom's house
    float flat2 = 37.65356;
    float flon2 = -97.45796;

    uint32_t currentTime;

    currentTime = micros();
    dist = distanceBetween(flat1, flon1, flat2, flon2);
    Serial.print("Distance ");
    Serial.print(dist, 0); 
    Serial.print(" meters, ");
    Serial.print(MetersToMiles(dist), 2); 
    Serial.print(" miles, time: ");
    Serial.println(micros() - currentTime); 
    
    currentTime = micros();
    dist = distanceBetween_Alt(flat1, flon1, flat2, flon2);
    Serial.print("Distance Alt ");
    Serial.print(dist, 0); 
    Serial.print(" meters, ");
    Serial.print(MetersToMiles(dist), 2); 
    Serial.print(" miles, time: ");
    Serial.println(micros() - currentTime); 
    
    float course = courseTo(flat1, flon1, flat2, flon2);
    Serial.print("Course to: ");
    Serial.print(course,1);
    Serial.print(" Cardinal: ");
    Serial.println(cardinal(course));
}


// Found this one in some forum post, it gives a very close result to the TinyGPS function above, it was about 2 meters short
// relative to the other over a distance of 4 miles. In a test it took over twice as long to calculate as the TinyGPS version
// 2192 micros vs 920
float distanceBetween_Alt(float flat1, float flon1, float flat2, float flon2)
{
float dist_calc = 0;
float dist_calc2 = 0;
float diflat = 0;
float diflon = 0;

    diflat = radians(flat2-flat1);
    flat1 = radians(flat1);
    flat2 = radians(flat2);
    diflon = radians((flon2)-(flon1));
    
    dist_calc  = (sin(diflat/2.0)*sin(diflat/2.0));
    dist_calc2 =  cos(flat1);
    dist_calc2 *= cos(flat2);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc  += dist_calc2;
    
    dist_calc = (2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
    
    dist_calc*=6371000.0; //Converting to meters
    
    //Serial.println(dist_calc);
    return dist_calc;
}




