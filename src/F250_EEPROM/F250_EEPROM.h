#ifndef F250_EEPROM_H
#define F250_EEPROM_H

#include <Arduino.h>
#include "../EEPROMex/EEPROMex.h"               // We use the extended version, not Arduino's built-in EEPROM library. 
#include "../F250_Settings/F250_Settings.h"     //
#include "F250_EEPROM_Struct.h"                 // This one is important. It has the definition of our eeprom struct


//--------------------------------------------------------------------------------------------------------------------------------------->>
//=======================================================================================================================================>>
// VERY IMPORTANT ! 
//=======================================================================================================================================>>
// If any changes are made to the _eeprom_data struct in OP_EEPROM_Struct.h, the EEPROM_INIT definition below must be changed to a new number.

    #define EEPROM_INIT             0x1022        // Modified 8/4/2017
//
//
//=======================================================================================================================================>>
//--------------------------------------------------------------------------------------------------------------------------------------->>


#define EEPROM_START_ADDRESS    0



// Class F250_EEPROM
class F250_EEPROM
{   public:
        F250_EEPROM(void);                          // Constructor
        static boolean begin(void);                 // This function will initialize EEPROM to default values if it hasn't been before.
                                                    // After initialization, the EEPROM_INIT number above will be written to EEPROM, so
                                                    // next time we'll know it's been done. This prevents us from writing to the EEPROM over and over
                                                    // every time we turn on the Arduino. 
        // Vars
        static _eeprom_data ramcopy;                // This is the complete eeprom struct that we will store in RAM
        
        // Functions
        static void loadRAMcopy(void);              // This will load the eeprom data from eeprom into our ramcopy struct in RAM
        
        static void factoryReset(void);             // This will force a call to Initialize_EEPROM(). All eeprom vars will be rest to default values. 

    protected:

        // Functions
        static void Initialize_RAMcopy(void);       // Called by Initilize_EEPROM. It sets all the RAM variables to default values. 
        static void Initialize_EEPROM(void);        // This copies all variables (at default values) from RAM into eeprom.
        
        
};




#endif
