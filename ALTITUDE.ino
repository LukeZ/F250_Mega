
void StartPressureReadings()
{   
    // This function sends the current averaged pressure-altitude reading to the display on a routine schedule
    if (TimerID_PressureAltitudeSender == 0) TimerID_PressureAltitudeSender = timer.setInterval(PRESSURE_ALT_SEND_FREQ, SendPressureAltitude);
    else if (timer.isEnabled(TimerID_PressureAltitudeSender) == false) timer.enable(TimerID_PressureAltitudeSender);

    // This function updates the pressure sensor reading, averages it, and converts to pressure altitude
    if (TimerID_PressureAltitudeCheck == 0) TimerID_PressureAltitudeCheck = timer.setInterval(PRESSURE_ALT_CHECK_FREQ, UpdatePressureAltitude);
    else if (timer.isEnabled(TimerID_PressureAltitudeCheck) == false) timer.enable(TimerID_PressureAltitudeCheck);     

    if (DEBUG) DebugSerial->println(F("Pressure Sensor Readings Started"));
}

void PausePressureReadings()
{
    if (TimerID_PressureAltitudeSender > 0) timer.disable(TimerID_PressureAltitudeSender);  // Pause the routine submission of voltage data to the display
    if (TimerID_PressureAltitudeCheck > 0)  timer.disable(TimerID_PressureAltitudeCheck);   // Pause the routine update of averaged voltage readings
    
    if (DEBUG) DebugSerial->println(F("Pressure Sensor Readings Paused"));
}

void UpdatePressureAltitude()
{
    // Takea a pressure reading, average kPa, save to global Pressure
    Pressure = fir_basic(ReadPressure_kPa(), PA_NTAPS, PA_line);

    // Make sure we don't need to change atmospheric regions
    // VerifyRegion();  // TRUST ME, WE WON'T

    // Convert Pressure in kPa to altitude in meters for the correct region
    Pressure_Altitude_Meters = kPa_To_Meters(Pressure);
    
    // Convert to feet and round to nearest integer
    Pressure_Altitude_Feet = (int16_t)(MetersToFeet(Pressure_Altitude_Meters) + 0.5);

    // Let's also calculate inches of mercury
    inHg = convert_kPa_inHG(Pressure);
}


void SendPressureAltitude()
{
static int16_t Pressure_Altitude_Feet_Prior = 0;

    uint8_t Alt_Tens = abs(Pressure_Altitude_Feet) % 100;                     // Number of feet under 100
    uint8_t Alt_Hundreds = abs(Pressure_Altitude_Feet) / 100;                 // Number of feet over 100, divided by 100
    
    if (Pressure_Altitude_Feet < 0) SendDisplay(CMD_PRESSURE_ALTITUDE_NEG, Alt_Hundreds, Alt_Tens); 
    else                            SendDisplay(CMD_PRESSURE_ALTITUDE_POS, Alt_Hundreds, Alt_Tens); 

    float tmp_inHg = inHg + 0.005;           // Round up to nearest one-hundredth
    uint8_t Pres_Tens = (uint8_t)(tmp_inHg);
    uint8_t Pres_Fract = (int16_t)(tmp_inHg * 100.0) % 100;
    SendDisplay(CMD_PRESSURE_MERCURY, Pres_Tens, Pres_Fract);
    
    
    if (DEBUG && Pressure_Altitude_Feet != Pressure_Altitude_Feet_Prior)
    {
        // You can get temp from that sensor as well
        DebugSerial->print("Temperature = ");
        DebugSerial->print(bmp.readTemperature() * 1.8 + 32.0);
        DebugSerial->print(" *F   ");
 
        DebugSerial->print(F("Pressure Altitude: "));
        DebugSerial->print(Pressure_Altitude_Feet);
        DebugSerial->print(F(" ft (")); 
        DebugSerial->print(Pressure, 3);
        DebugSerial->print(F(" kPa / "));
        DebugSerial->print(inHg, 3); 
        DebugSerial->println(F(" in Hg)"));
        
        Pressure_Altitude_Feet_Prior = Pressure_Altitude_Feet;
    }
}


void InitAltimeter(void)
{
    // BMP180 sensor object
    bmp.begin();  

    // Takea a reading in pA and convert to kPa
    Pressure = ReadPressure_kPa();

    // Init filter line used for pressure
    setFIR(PA_NTAPS, PA_line, Pressure); 

    // Adjust p1 with whatever is saved in EEPROM
    // Later if the GPS determines we're at home an adjustment will be
    // made based on the home altitude.
    AdjustP1();
}


float ReadPressure_kPa(void)
{   // Read the BMP sensor and convert returned pA into kPa
    return (float)bmp.readPressure() / 1000.0;
}


/********************************************************************************************
*    Function Name:  CorrectP1                                                              *
*    Return Value:   void                                                                   *
*    Parameters:    -MeasuredP: current pressure measurement in kPa                         *
*                   -KnownAltitude: the known pressure altitude in METERS of the location   *
*                    where MeasuredP is taken.                                              *
*    Description:    This routine re-calculates the p1 variables used in the altitude       *
*                    calculations: p1 is typically set to the pressure at sea level on a    *
*                    standard day. However, since the standard day conditions rarely exist  *
*                    this sets p1 equal to the pressure at sea level for today.             *
*                    The result should be that pressure measurements taken at the known     *
*                    location should read exactly the same value as KnownAltitude.          *
*                                                                                           *
********************************************************************************************/
void CorrectP1(float MeasuredP, float KnownAltitude)    // KnownAltitude must come in Meters
{
// The standard-day value of p1 is corrected for current conditions (known starting height and pressure measurement)
// Returns nothing, but sets value of float p1_Tsphere and p1_Tpause
    
    // KnownAltitude = known height of starting location in meters 
    // MeasuredP = Current pressure in kPa as measured by sensor
    
    //  Here we set p1 to its adjusted value using Equation 6 from the Theory section. p1 exists in RAM only and is used to calculate altitude from pressure
    p1 = MeasuredP * (pow( ((T1+(a*(float)KnownAltitude))/T1) , (g/(a*R)) )); 

    // Calculate an adjustment factor which is the compensated sea level pressure, minus the default sea level pressure. 
    // The adjustment gets saved to ramcopy (though we don't use it anywhere), and also the EEPROM for our next load. 
    // Note this function gets called when: 
    // - You manually set an adjustment using the display (either manually to a specified altitude or when using the GPS as a reference altitude), or
    // - When the car starts at a location near home, which is likely to be most days you drive it.
    // However EEPROM has a write life-cycle of 100,000. If you start the car 5 times a day you shouldn't wear it out until 54 years. So I am not going to worry about it. 
    eeprom.ramcopy.p1_Adjust = (p1 - p1_default); 
    EEPROM.updateFloat(offsetof(_eeprom_data, p1_Adjust), eeprom.ramcopy.p1_Adjust);
    // We also save the time when this update was made
    CopyDateTime(CurrentDateTime, &eeprom.ramcopy.lastAltitudeAdjust);
    EEPROM.updateBlock(offsetof(_eeprom_data, lastAltitudeAdjust), CurrentDateTime);
}

// This does not CORRECT p1, it simply adjusts it by our offset amount. 
void AdjustP1()
{
    p1 = p1_default + eeprom.ramcopy.p1_Adjust;
}

void DecideAltitudeSource()
{
    UsePressureAltitude = false; 
    
    if (startAtHome)
    {   // If we started this session from near home, we can use pressure altitude because we were able to adjust it to a known starting point
        UsePressureAltitude = true;
    }
    else
    {
        // We didn't start at home. Have we had a manual altitude adjustment within the last 2 days? 
        if ((DayOfYear(CurrentDateTime) - DayOfYear(eeprom.ramcopy.lastAltitudeAdjust)) < 3)
        {
            UsePressureAltitude = true;
        }
    }

    // All other cases we use GPS altitude

    return UsePressureAltitude;
}


float kPa_To_Meters(float kPa)
{
    float mtr;

    // This is Equation 3 from the Theory section
    mtr = (((pow((kPa/p1),(-(a*R)/g))) * T1) - T1) / a ; 
    
    return mtr;
}

float FeetToMeters(int16_t feet)
{
    return (feet * 0.3048);
}

int16_t MetersToFeet(int16_t meters)
{
    return (meters * 3.28084);
}

float convert_kPa_inHG(float kPa)
{
    return (kPa * 0.2953);   
}

float convert_inHG_kPa(float hg)
{
    return (hg * 3.38639);   
}


