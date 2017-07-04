#ifndef F250_EEPROM_STRUCT_H
#define F250_EEPROM_STRUCT_H

// EEPROM DATA STRUCTURE
//--------------------------------------------------------------------------------------------------------------------------------------->>
// This defines every variable held in EEPROM
//--------------------------------------------------------------------------------------------------------------------------------------->>



struct _eeprom_data 
{

// OneWire temp sensor addresses
    byte TempAddress_Int[8];
    byte TempAddress_Ext[8];
    byte TempAddress_Aux[8];

// Time stuff
    uint8_t Timezone;

// Altitude stuff
    // Current adjustment, based on an altitude received from the display

// GPS stuff
    float Lat_Home;     // Coordinates of home location
    float Lon_Home;
    int16_t Alt_Home;   // Home altitude, will use to adjust barometric pressure on startup



// Marker
    uint32_t InitStamp;                          
};


#endif  // Define F250_EEPROM_STRUCT_H