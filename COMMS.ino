
// -------------------------------------------------------------------------------------------------------------------------------------------------->
// SERIAL COMMANDS - TRANSMITTING
// -------------------------------------------------------------------------------------------------------------------------------------------------->
void SendDisplay(byte command, byte value, byte modifier)
{
    DisplaySerial.write(DISPLAY_ADDRESS);
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
        if (ByteIn == MASTER_ADDRESS)
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
/*
        case CMD_ENGINE_START:         StartEngine();                                      break;
        case CMD_ENGINE_STOP:          StopEngine();                                       break;
        case CMD_USER_SOUND_PLAY:      PlayUserSound(sentence->Modifier, true, false);     break;  // Modifier indicates which sound, "true" for "start", "false" for "don't repeat"
        case CMD_SQUEAK_SET_MIN:       SetSqueakMin(sentence->Value, sentence->Modifier);  break;
        case CMD_BEEP_ONCE:            Beep(1);                                            break;        
        case CMD_BEEP_X:               Beep(sentence->Value);                              break;        
        case CMD_SET_VOLUME:           UpdateVolume_Serial(sentence->Value);               break;
        case CMD_BRAKE_SOUND:          BrakeSound();                                       break;
*/
        default:
            break;
    }
}


