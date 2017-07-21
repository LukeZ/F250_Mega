
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// SERIAL COMMANDS - TRANSMITTING
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void SendDisplay(byte command, byte value, byte modifier)
{
    DisplaySerial.write(DISPLAY_ADDRESS);       // Sending to the DISPLAY
    DisplaySerial.write(command);
    DisplaySerial.write(value);
    DisplaySerial.write(modifier);
    DisplaySerial.write((DISPLAY_ADDRESS + command + value + modifier) & B01111111);
}

void SendDisplay(byte command, byte value)
{
    SendDisplay(command, value, 0); 
}

void SendDisplay(byte command)
{
    SendDisplay(command, 0, 0);
}


// -------------------------------------------------------------------------------------------------------------------------------------------------->
// SERIAL COMMANDS - RECEIVING
// -------------------------------------------------------------------------------------------------------------------------------------------------->
boolean CheckSerial(void)   // THIS IS US READING COMMANDS _FROM_ THE TEENSY DISPLAY CONTROLLER
{
    byte ByteIn;
    static char input_line[SENTENCE_BYTES];     // An array to store incoming bytes
    boolean SentenceReceived = false;           // Start off false, will get set to true if a valid sentence was received
    static boolean addressReceived = false;     // Have we received a byte that matches our address
    static uint8_t numBytes = 0;                // Start off with no data received
    static DataSentence Sentence;               // A struct to store incoming commands

    // Read all the bytes that are available, starting with the first byte that matches our address
    while(DisplaySerial.available())               
    {
        ByteIn = DisplaySerial.read();
        if (ByteIn == MASTER_ADDRESS)       // We are the MASTER
        {
            addressReceived = true;         // Matching address
            input_line[0] = ByteIn;         // Save it in our array
            numBytes = 1;                   // Subsequent bytes will be added to the array until we have enough to compare against INIT_STRING
        }
        else if (addressReceived)
        {
            input_line[numBytes++] = ByteIn;
            if (numBytes >= SENTENCE_BYTES) break;  // We have enough bytes for a full sentence, so evaluate it
        }
    }

    // If we have enough bytes for a full sentence, save it
    if (numBytes >= SENTENCE_BYTES)
    {   // We have enough bytes for a full sentence
        Sentence.Address  = input_line[0];
        Sentence.Command  = input_line[1];
        Sentence.Value    = input_line[2];
        Sentence.Modifier = input_line[3];
        Sentence.Checksum = input_line[4];

        // Now verify the checksum
        if (ChecksumValid(&Sentence))
        {
            SentenceReceived = true;        // Yes, a valid sentence has been received!  
            ProcessCommand(&Sentence);      // Do whatever we're told
        }

        // Start everything over
        input_line[0] = '\0';
        addressReceived = false;
        numBytes = 0;
    }

    return SentenceReceived;
}

boolean ChecksumValid(DataSentence * sentence)
{
    uint8_t check = (sentence->Address + sentence->Command + sentence->Value + sentence->Modifier) & B01111111;

    if (check == sentence->Checksum) return true;
    else                             return false;
}

void ProcessCommand(DataSentence * sentence)
{
uint8_t cmd;  

    switch (sentence->Command)
    {
        case RCV_CMD_SET_HOME_COORD:
            // Save current coordinates as Home, if we have a fix. The display should only know to try this if we have a fix. 
            if (GPS.fix)
            {   
                EEPROM.updateFloat(offsetof(_eeprom_data, Lat_Home), Current_Latitude);
                EEPROM.updateFloat(offsetof(_eeprom_data, Lon_Home), Current_Longitude);
                SendDisplay(CMD_ACTION_TAKEN);
                if (DEBUG) DebugSerial->println(F("Home coordinates set")); 
            }
            break;

        case RCV_CMD_SET_GPS_ALT:
            // Set current altitude adjustment based on current GPS altitude - ie, we assume (or know) the GPS is correct
            if (GPS_Fixed)  // No point in doing this if we don't have a GPS fix, but the display shouldn't let us anyway
            {   // Here we use GPS_Altitude as the correction. We don't receive this from the display since we already have it, and we already have it in Meters - float, so no conversion needed either. 
                CorrectP1(Pressure, GPS_Altitude_Meters);      // This takes our current barometric pressure in kPa and a known altitude in Meters, corrects p1 and saves the adjustment factor to EEPROM
                SendDisplay(CMD_ACTION_TAKEN);
                if (DEBUG) { DebugSerial->print(F("Altitude correction set based on current GPS altitude: ")); DebugSerial->println(GPS_Altitude_Feet); }
            }
            // Otherwise if no fix do nothing, the display will figure out shortly it didn't work. 
            break;

        case RCV_CMD_SET_CURRENT_ALT:
            // We are being given a current altitude, but what we actually save in EEPROM is an adjustment
            {
                int16_t alt = (sentence->Value * 100) + sentence->Modifier;    // Altitude given IN FEET
                CorrectP1(Pressure, FeetToMeters(alt));      // This takes our current barometric pressure in kPa and a known altitude in Meters, corrects p1 and saves the adjustment factor to EEPROM
                SendDisplay(CMD_ACTION_TAKEN);
                if (DEBUG) { DebugSerial->print(F("Altitude correction set based on manual altitude entry: ")); DebugSerial->print(alt); Serial.print(F("' Corrected p1: ")); Serial.print(p1,3); Serial.print(F(" Adjustment factor: ")); Serial.println(eeprom.ramcopy.p1_Adjust,3); }
            }
            break; 
        
        case RCV_CMD_SET_HOME_ALT:
            // Save home altitude to EEPROM
            {
                eeprom.ramcopy.Alt_Home = (sentence->Value * 100) + sentence->Modifier;    
                EEPROM.updateInt(offsetof(_eeprom_data, Alt_Home), eeprom.ramcopy.Alt_Home);
                SendDisplay(CMD_ACTION_TAKEN);
                if (DEBUG) { DebugSerial->print(F("Home altitude set to: ")); DebugSerial->println(eeprom.ramcopy.Alt_Home); }
            }
            break;
            
        case RCV_CMD_SET_TIMEZONE:
            // Update timezone
            CurrentDateTime.timezone = sentence->Value;                             // Ram copy, we have two of them
            eeprom.ramcopy.Timezone = sentence->Value;                              // We don't actually probably use this one
            EEPROM.updateByte(offsetof(_eeprom_data, Timezone), sentence->Value);   // EEPROM copy. We do save this here, we don't save an entire date struct
            SendDisplay(CMD_ACTION_TAKEN);  
            if (DEBUG) 
            { 
                DebugSerial->print(F("Current timezone set to: ")); 
                switch (sentence->Value)
                {
                    case AKST:  DebugSerial->println(F("AKST (Alaska)"));   break;  // Alaska timezone - it may use different dates for DST if it even observes it, I don't know. 
                    case PST:   DebugSerial->println(F("PST (Pacific)"));   break;  // Pacific standard time
                    case MST:   DebugSerial->println(F("MST (Mountain)"));  break;  // Mountain standard time
                    case CST:   DebugSerial->println(F("CST (Central)"));   break;  // Central standard time
                    case EST:   DebugSerial->println(F("EST (Eastern)"));   break;  // Eastern standard time
                    default:    DebugSerial->println(F("Unknown"));         break;
                }
            }
            break;

        case RCV_CMD_RESET_ABS_TEMP:
            // Erase prior all-time min/max temperatures and set them to current values. Leave session temps unchanged. 
            // You could potentially also pass a flag in Value to only clear min or max rather than both... 
            switch (sentence->Modifier)
            {
                case 0: // Internal sensor
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMax), InternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMin), InternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                    SendDisplay(CMD_ACTION_TAKEN);
                    // We also send back the values we've just cleared it to, in order that the Teensy will save it on its end as well
                    EEPROM.readBlock(offsetof(_eeprom_data, SavedInternalTemp), eeprom.ramcopy.SavedInternalTemp);
                    eeprom.ramcopy.SavedInternalTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedInternalTemp.AbsoluteMin, InternalTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp);
                    eeprom.ramcopy.SavedInternalTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedInternalTemp.AbsoluteMax, InternalTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp);                    
                    if (DEBUG) DebugSerial->println(F("All-Time absolute temps re-set for Internal sensor")); 
                    break;
                case 1: // External sensor
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMax), ExternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMin), ExternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                    SendDisplay(CMD_ACTION_TAKEN);
                    // We also send back the values we've just cleared it to, in order that the Teensy will save it on its end as well
                    EEPROM.readBlock(offsetof(_eeprom_data, SavedExternalTemp), eeprom.ramcopy.SavedExternalTemp);
                    eeprom.ramcopy.SavedExternalTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedExternalTemp.AbsoluteMin, ExternalTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp);
                    eeprom.ramcopy.SavedExternalTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedExternalTemp.AbsoluteMax, ExternalTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp);                    
                    if (DEBUG) DebugSerial->println(F("All-Time absolute temps re-set for External sensor")); 
                    break;
                case 2: // Aux sensor
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMax), AuxTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMin), AuxTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                    SendDisplay(CMD_ACTION_TAKEN);
                    // We also send back the values we've just cleared it to, in order that the Teensy will save it on its end as well
                    EEPROM.readBlock(offsetof(_eeprom_data, SavedAuxTemp), eeprom.ramcopy.SavedAuxTemp);
                    eeprom.ramcopy.SavedAuxTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedAuxTemp.AbsoluteMin, AuxTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp);
                    eeprom.ramcopy.SavedAuxTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
                    SendDisplay(cmd, eeprom.ramcopy.SavedAuxTemp.AbsoluteMax, AuxTemp.sensorName);
                    SendDisplayDateTime(eeprom.ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp);                    
                    if (DEBUG) DebugSerial->println(F("All-Time absolute temps re-set for Aux sensor")); 
                    break;                    
            }
            break;

        default:
            break;
    }
}


