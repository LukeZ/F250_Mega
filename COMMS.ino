
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
    switch (sentence->Command)
    {
        case RCV_CMD_SET_HOME_COORD:
            // Save current coordinates as Home, if we have a fix
            if (GPS.fix)
            {
                EEPROM.updateFloat(offsetof(_eeprom_data, Lat_Home), Current_Latitude);
                EEPROM.updateFloat(offsetof(_eeprom_data, Lon_Home), Current_Longitude);
            }
            break;

        case RCV_CMD_SET_HOME_ALT:
            // Save home altitude to EEPROM
            {
                uint16_t alt = (sentence->Value * 100) + sentence->Modifier;    
                EEPROM.updateInt(offsetof(_eeprom_data, Alt_Home), alt);
            }
            break;

        case RCV_CMD_SET_CURRENT_ALT:
            // We are being given a current altitude, but what we actually save in EEPROM is an adjustment
            {
                uint16_t alt = (sentence->Value * 100) + sentence->Modifier;    // Altitude given
            }
            break; 
        
        case RCV_CMD_SET_TIMEZONE:
            EEPROM.updateByte(offsetof(_eeprom_data, Timezone), sentence->Value);        
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
                    break;
                case 1: // External sensor
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMax), ExternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMin), ExternalTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                    break;
                case 2: // Aux sensor
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMax), AuxTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                    EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMin), AuxTemp.constrained_temp);
                    EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                    break;                    
            }
            break;

        default:
            break;
    }
}


