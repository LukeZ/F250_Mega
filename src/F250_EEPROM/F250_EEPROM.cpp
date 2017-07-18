#include "F250_EEPROM.h"


// Static variables must be declared outside the class
    _eeprom_data F250_EEPROM::ramcopy;


//------------------------------------------------------------------------------------------------------------------------>>
// CONSTRUCTOR, BEGIN, and FIRST STUFF
//------------------------------------------------------------------------------------------------------------------------>>

// Constructor
F250_EEPROM::F250_EEPROM(void) { }

// Begin
boolean F250_EEPROM::begin(void)
{
    // Check if EEPROM has ever been initalized, if not, do so
    long Temp = EEPROM.readLong(offsetof(_eeprom_data, InitStamp));// Get our EEPROM initialization stamp code
    if(Temp != EEPROM_INIT)                                         // EEPROM_INIT is set at the top of OP_EEPROM.h. It must be changed to a new number  
    {   Initialize_EEPROM();                                        // if changes have been made to the OP_EEPROM_Struct.h file. 
        return true;        // If we initialized EEPROM, return true
    }
    else
    {
        // In this case, the values in EEPROM are what we want. We load them all to RAM
        loadRAMcopy();
        return false;
    }

}

// This takes all variables from the eeprom struct, and puts them into the RAM copy struct
void F250_EEPROM::loadRAMcopy(void)
{
    EEPROM.readBlock(EEPROM_START_ADDRESS, ramcopy);
}
    



//------------------------------------------------------------------------------------------------------------------------>>
// INITIALIZE EEPROM
//------------------------------------------------------------------------------------------------------------------------>>
void F250_EEPROM::factoryReset(void)
{
    Initialize_EEPROM();
}

void F250_EEPROM::Initialize_EEPROM(void) 
{   
    // The way we do this is set the values in our ramcopy struct, then write the entire struct to EEPROM (actually "update" instead of "write")
    Initialize_RAMcopy();                               // Set RAM variables to sensible defaults
    ramcopy.InitStamp = EEPROM_INIT;                    // Set the InitStamp
    EEPROM.updateBlock(EEPROM_START_ADDRESS, ramcopy);  // Now write it all to EEPROM. We use the "update" function so as not to 
                                                        // unnecessarily writebytes that haven't changed. 
}

// THIS IS WHERE EEPROM DEFAULT VALUES ARE SET
void F250_EEPROM::Initialize_RAMcopy(void) 
{   // If EEPROM has not been used before, we initialize to some sensible, yet conservative, default values.
    
    // OneWire temp sensor addresses
        for (uint8_t i=0; i<8; i++)
        {
            ramcopy.TempAddress_Int[i] = 0;
            ramcopy.TempAddress_Ext[i] = 0;
            ramcopy.TempAddress_Aux[i] = 0;
        }

    // Time stuff
        ramcopy.Timezone = CST;

    // Temp stuff
        // Absolute min/max internal
        ramcopy.SavedInternalTemp.AbsoluteMin = 200;                // Initialize to something that will quickly get over-written with actual data
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.day = 1;
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.year = 17;
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.hour = 12;
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.minute = 0;
        ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp.second = 0;
        ramcopy.SavedInternalTemp.AbsoluteMax = -50;
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.day = 1;
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.year = 17;
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.hour = 12;
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.minute = 0;
        ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp.second = 0;
        // Absolute min/max external
        ramcopy.SavedExternalTemp.AbsoluteMin = 200;                // Initialize to something that will quickly get over-written with actual data
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.day = 1;
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.year = 17;
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.hour = 12;
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.minute = 0;
        ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp.second = 0;
        ramcopy.SavedExternalTemp.AbsoluteMax = -50;
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.day = 1;
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.year = 17;
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.hour = 12;
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.minute = 0;
        ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp.second = 0;
        // Absolute min/max aux
        ramcopy.SavedAuxTemp.AbsoluteMin = 200;                // Initialize to something that will quickly get over-written with actual data
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.day = 1;
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.year = 17;
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.hour = 12;
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.minute = 0;
        ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp.second = 0;
        ramcopy.SavedAuxTemp.AbsoluteMax = -50;
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.month = 1;   // Timestamp 1/1/2017 at 12:00:00 noon
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.day = 1;
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.year = 17;
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.hour = 12;
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.minute = 0;
        ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp.second = 0;

    
    // Altitude stuff
        ramcopy.p1_Adjust = 0.0;                                    // Initialize to no adjustment
        ramcopy.lastAltitudeAdjust.month = 1;                       // Initialize date to long ago
        ramcopy.lastAltitudeAdjust.day = 1;
        ramcopy.lastAltitudeAdjust.year = 0;
        ramcopy.lastAltitudeAdjust.hour = 0;
        ramcopy.lastAltitudeAdjust.minute = 0;
        ramcopy.lastAltitudeAdjust.second = 0;
        ramcopy.lastAltitudeAdjust.timezone = 3;
        
        
    // GPS Stuff
    // Recall N and E are positive, S and W are negative
        // Coordinates at Huntington Park apartments, Wichita
        // Use the "degrees" version of Adafruit's lat/long
        ramcopy.Lat_Home = 37.7060;     // North 
        ramcopy.Lon_Home = -97.4643;    // West
        ramcopy.Alt_Home = 409;         // Altitude in METERS - Huntington Park Apartments, Wichita (1,342 feet)
       
}



