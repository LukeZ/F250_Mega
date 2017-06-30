#ifndef F250_EEPROM_STRUCT_H
#define F250_EEPROM_STRUCT_H

// EEPROM DATA STRUCTURE
//--------------------------------------------------------------------------------------------------------------------------------------->>
// This defines every variable held in EEPROM
//--------------------------------------------------------------------------------------------------------------------------------------->>


struct _eeprom_data 
{

// OneWire Temp Sensors
    byte TempAddress_Int[8];
    byte TempAddress_Ext[8];
    byte TempAddress_Aux[8];

// Marker
    uint32_t InitStamp;                          
};


#endif  // Define F250_EEPROM_STRUCT_H