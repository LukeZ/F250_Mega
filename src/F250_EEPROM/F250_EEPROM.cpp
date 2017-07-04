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
    
    // Altitude stuff
        // Current adjustment, based on an altitude received from the display
        
    // GPS Stuff
    // Recall N and E are positive, S and W are negative
        // Coordinates at Huntington Park apartments, Wichita
        // Use the "degrees" version of Adafruit's lat/long
        ramcopy.Lat_Home = 37.7060;     // North 
        ramcopy.Lon_Home = -97.4643;    // West
        ramcopy.Alt_Home = 409;         // Altitude in meters - Huntington Park Apartments, Wichita
       
}



