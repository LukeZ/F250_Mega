#ifndef F250_EEPROM_STRUCT_H
#define F250_EEPROM_STRUCT_H

// EEPROM DATA STRUCTURE
//--------------------------------------------------------------------------------------------------------------------------------------->>
// This defines every variable held in EEPROM
//--------------------------------------------------------------------------------------------------------------------------------------->>

#include "../F250_Settings/F250_Settings.h"     

struct _eeprom_data 
{

// OneWire temp sensor addresses
    byte TempAddress_Int[8];
    byte TempAddress_Ext[8];
    byte TempAddress_Aux[8];

// Time stuff
    uint8_t Timezone;

// Temp stuff
    _saved_tempdata SavedInternalTemp;
    _saved_tempdata SavedExternalTemp;
    _saved_tempdata SavedAuxTemp;

// Altitude stuff
    float p1_Adjust;                    // Current p1 adjustment. Add to default p1 to get corrected p1
    _datetime lastAltitudeAdjust;       // When was the last adjustment made. 

// GPS stuff
    float Lat_Home;                     // Coordinates of home location
    float Lon_Home;
    int16_t Alt_Home;                   // Home altitude in METERS, will use to adjust barometric pressure on startup



// Marker
    uint32_t InitStamp;                          
};


#endif  // Define F250_EEPROM_STRUCT_H