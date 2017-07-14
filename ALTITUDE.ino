
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
    VerifyRegion();

    // Convert Pressure in kPa to altitude in meters for the correct region
    Pressure_Altitude_Meters = kPa_To_Meters(Pressure, Region);
    
    // Convert to feet and round to nearest integer
    Pressure_Altitude_Feet = (int16_t)((Pressure_Altitude_Meters * 3.281) + 0.5);
}


void SendPressureAltitude()
{
static int16_t Pressure_Altitude_Feet_Prior = 0;
    
    if (DEBUG && Pressure_Altitude_Feet != Pressure_Altitude_Feet_Prior)
    {
        // You can get temp from that sensor as well
        DebugSerial->print("Temperature = ");
        DebugSerial->print(bmp.readTemperature() * 1.8 + 32.0);
        DebugSerial->print(" *F   ");
 
        DebugSerial->print(F("Pressure Altitude: "));
        DebugSerial->print(Pressure_Altitude_Feet);
        DebugSerial->print(F(" ft (")); 
        DebugSerial->print(Pressure);
        DebugSerial->println(F(" kPa)"));
        
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
}


float ReadPressure_kPa(void)
{   // Read the BMP sensor and convert returned pA into kPa
    return (float)bmp.readPressure() / 1000.0;
}


/********************************************************************************************
*    Function Name:  CorrectP1                                                              *
*    Return Value:   void                                                                   *
*    Parameters:    -MeasuredP: current pressure measurement in kPa                         *
*                   -KnownAltitude: the known pressure altitude in meters of the location   *
*                    where MeasuredP is taken.                                              *
*    Description:    This routine re-calculates the p1 variables used in the altitude       *
*                    calculations: p1 is typically set to the pressure at sea level on a    *
*                    standard day. However, since the standard day conditions rarely exist  *
*                    this sets p1 equal to the pressure at sea level for today.             *
*                    The result should be that pressure measurements taken at the known     *
*                    location should read exactly the same value as KnownAltitude.          *
*                    p1 for the Tropopause is also adjusted.                                *
********************************************************************************************/
void CorrectP1(float MeasuredP, int16_t KnownAltitude)
{
// The standard-day value of p1 is corrected for current conditions (known starting height and pressure measurement)
// Returns nothing, but sets value of float p1_Tsphere and p1_Tpause
    
    // KnownAltitude = known height of starting location in meters 
    // MeasuredP = Current pressure in kPa as measured by sensor
    
    //  p1_Tsphere and p1_Tpause are global variables
    //  p1_Tsphere = p1 Troposphere
    //  p1_Tpause = p1 Tropopause

    //  Here we set p1_TSphere to its adjusted value using Equation 6 from the Theory section
    p1_Tsphere = MeasuredP * (pow( ((T1+(a*(float)KnownAltitude))/T1) , (g/(a*R)) )); 

    //  We calculate p1_Tpause by using the formula for pressure in the troposphere, and setting
    //  h (height) to 11,000 meters. But since we use our corrected p1_TSphere, p1_TPause will
    //  also be corrected (see Example 3 in the Theory section)
    p1_Tpause = p1_Tsphere * (pow(((T1 + (a*11000))/T1), (-(g/(a*R)))));

    return;
}








/********************************************************************************************
*    Function Name: VerifyRegion                                                            *
*    Return Value:  Region char, if wanted                                                  *
*    Parameters:    - current air pressure, just uses Pressure global variable              *
*    Description:   This routine compares the current air pressure to the air pressure at   *
*                   the Tropopause/Troposphere border. If kPa is slightly above the         *
*                   border, Region is changed to Tropopause. If slightly below, it is       *
*                   changed to Troposphere. There is a band of 0.8 kPa between which the    *
*                   change will not be triggered, to keep from fluctuating back and forth   *
*                   when right on the line.                                                 *
*                   Because the Region variable is global, the calling function could pick  *
*                   it up without it being returned, if they even wanted to know what it    *
*                   was (mostly they will just want to set it but don't care what it is),   *
*                   but it is returned anyways for convenience's sake.                      *
********************************************************************************************/
unsigned char VerifyRegion(void)
{
// Checks the current global variable "Pressure" and compares it to known values
// for Troposphere and Tropopause - if the value is Tropopausal but
// current region is Tropospherical, it will change the Region 
// variable, as it will for the other way around. The Region
// variable is used in kPa_To_Meters to determine which altitude
// formula should be used. Region values are defined below (defines)
    
    if (Region == Troposphere)              // We're currently in troposphere
    { 
        if (Pressure <= (p1_Tpause - 0.4))  // (p1_Tpause - 0.4 kPa) is approximately 100 m. up 
        {                                   // into tropopause
            Region = Tropopause;            // So, we change regions
        }
    }   
    else                                    // We're in tropopause
    {
        if (Pressure >= (p1_Tpause + 0.4))  // (p1_Tpause + 0.4 kPa) is approximately 100 m. down
        {                                   // into troposphere
            Region = Troposphere;           // So, we change back
        }
    }

    return Region;
}


/********************************************************************************************
*    Function Name: kPa_To_Meters                                                           *
*    Return Value:  Meters                                                                  *
*    Parameters:    kPa - current air pressure                                              *
*                   MyRegion - Current Region (could have just looked at global var "Region"*
*    Description:   Here we calculate present altitude in meters based on the formula for   *
*                   the given region.                                                       *
********************************************************************************************/
float kPa_To_Meters(float kPa, unsigned char MyRegion)
{
    float mtr;

    if (MyRegion == Troposphere)
    { 
        // This is Equation 3 from the Theory section
        mtr = (((pow((kPa/p1_Tsphere),(-(a*R)/g))) * T1) - T1) / a ; 
    }
    else    // We're in the Tropopause
    { 
        // This is Equation 5 from the Theory section
        mtr = -(log(kPa/p1_Tpause)/(g/(R*217.15)))+11000;        
    }

    return mtr;
}


