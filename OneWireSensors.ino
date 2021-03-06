
void StartTempReadings(void)
{
    // This will attempt to detect and identify any temperature sensors on the bus
    SetupOneWireSensors();                                          
    
    // Start a temperature reading process that will repeat itself automatically
    TempHandlerEnabled = true;
    TempHandler();                      // The handler cycles through each sensor on the bus and continuously takes readings and averages them, updates the sensor structs, etc... 
                                        // It only needs to be called once to start (with TempHandlerEnabled = true) and from there it will keep going forever until TempHandlerEnabled = false.         
    
    // This function sends temperature data to the Teensy at the set interval
    if (TimerID_TempSender == 0) TimerID_TempSender = timer.setInterval(TEMP_SEND_FREQ, SendTempInfo);            
    else if (timer.isEnabled(TimerID_TempSender) == false) timer.enable(TimerID_TempSender);

    // This function sends all-time min/max temps to the display once on startup
    SendDisplayAllTimeTemps();

    // This function re-checks the sensors
    if (TimerID_TempSensorCheck == 0) TimerID_TempSensorCheck = timer.setInterval(TEMP_SENSOR_CHECK_FREQ, SetupOneWireSensors);
    else if (timer.isEnabled(TimerID_TempSensorCheck) == false) timer.enable(TimerID_TempSensorCheck); 

    if (DEBUG) DebugSerial->println(F("Temp Readings Started"));
}

void PauseTempReadings(void)
{
    TempHandlerEnabled = false;                                                 // Disable the handler that takes readings
    if (TimerID_TempSender > 0)      timer.disable(TimerID_TempSender);         // Pause the routine submission of temperature data to the display
    if (TimerID_TempSensorCheck > 0) timer.disable(TimerID_TempSensorCheck);    // Pause the routine bus check to detect devices
    if (DEBUG) DebugSerial->println(F("Temp Readings Paused"));
}

/* Here we try to detect and identify the connected sensors */
void SetupOneWireSensors()
{
byte type_s;
byte addr[8];
boolean correctType = false; 

boolean Found[NUM_TEMP_SENSORS] = {false, false, false};
_tempsensor *ts; 
_saved_tempdata *sd;
    
    // See if anything is attached
    while(oneWire.search(addr) == true)
    {
        // the first ROM byte indicates which chip, here we only want to save temp sensors
        correctType = false; 
        switch (addr[0]) 
        {
            case 0x10:      type_s = 1; correctType = true;     break;  // DS18S20 or old DS1820
            case 0x28:      type_s = 2; correctType = true;     break;  // DS18B20
            case 0x22:      type_s = 3; correctType = true;     break;  // DS1822
        }    

        if (correctType)
        {
            // See if we have this one saved already
            // Internal? 
            if (memcmp(addr, eeprom.ramcopy.TempAddress_Int, 8) == 0)
            {   // Yes, this is the internal sensor
                InternalTemp.type_s = type_s;
                Found[TS_INTERNAL] = true; 
            }
            // External? 
            else if (memcmp(addr, eeprom.ramcopy.TempAddress_Ext, 8) == 0)
            {   // Yes, this is the external sensor
                ExternalTemp.type_s = type_s;
                Found[TS_EXTERNAL] = true;
            }
            // Aux? 
            else if (memcmp(addr, eeprom.ramcopy.TempAddress_Aux, 8) == 0)
            {   // Yes, this is the aux sensor
                AuxTemp.type_s = type_s;
                Found[TS_AUX] = true;
            }
            else
            {   // No, we haven't seen this sensor before. We save the first previously-unseen sensor as internal, the second as external, third as aux.
                // To do a first setup, first clear all EEPROM addresses by clearing EEPROM entirely (change EEPROM_INIT value in \src\F250_EEPROM\F250_EEPROM.h)
                // Plug in your Internal sensor first, and wait for it to be found (watch the Debug screen). After it has been found, plug in the external, wait again, 
                // then you can do Aux last. You do not need to reboot between adding sensors, but you do need to wait for the one you have just plugged in to register
                // before you plug in the next one. 
                // After the addresses have been saved they are not overwritten again and only those three sensors will be read from then on, unless you clear EEPROM 
                // again which requires a flash of new firmware. 
                if (InternalTemp.known == false)   
                {
                    PrintTempSensorName(TS_INTERNAL);
                    DebugSerial->print(F(" temp sensor address saved ("));
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Int[i] = addr[i]; InternalTemp.address[i] = addr[i]; DebugSerial->print(addr[i]); DebugSerial->print(" ");}
                    DebugSerial->println(")");
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Int), InternalTemp.address);
                    InternalTemp.type_s = type_s;
                    InternalTemp.known = true;
                    Found[TS_INTERNAL] = true;
                }
                else if (ExternalTemp.known == false)
                {
                    PrintTempSensorName(TS_EXTERNAL);
                    DebugSerial->print(F(" temp sensor address saved ("));                    
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Ext[i] = addr[i]; ExternalTemp.address[i] = addr[i]; DebugSerial->print(addr[i]); DebugSerial->print(" ");}
                    DebugSerial->println(")");
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Ext), ExternalTemp.address);
                    ExternalTemp.type_s = type_s;
                    ExternalTemp.known = true;
                    Found[TS_EXTERNAL] = true;
                }
                else if (AuxTemp.known == false)
                {
                    PrintTempSensorName(TS_AUX);
                    DebugSerial->print(F(" temp sensor address saved ("));                    
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Aux[i] = addr[i]; AuxTemp.address[i] = addr[i]; DebugSerial->print(addr[i]); DebugSerial->print(" ");}
                    DebugSerial->println(")");
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Aux), AuxTemp.address);
                    AuxTemp.type_s = type_s;
                    AuxTemp.known = true;
                    Found[TS_AUX] = true; 
                }                
            }
        }
    }

    for (uint8_t i=0; i<NUM_TEMP_SENSORS; i++)

    {
        if (i == TS_INTERNAL) { ts = &InternalTemp; sd = &eeprom.ramcopy.SavedInternalTemp; }
        if (i == TS_EXTERNAL) { ts = &ExternalTemp; sd = &eeprom.ramcopy.SavedExternalTemp; }
        if (i == TS_AUX)      { ts = &AuxTemp;      sd = &eeprom.ramcopy.SavedAuxTemp;      }
                
        if (Found[i] && !ts->present)       // This is a new find
        {
            ts->present = true; 
            ts->sensorLost = false; 
            ts->shoeHorn = false;           // Undo the shoeHorn workaround
            if (DEBUG) 
            {
                PrintTempSensorName(ts->sensorName);
                DebugSerial->print(F(" temp sensor found ("));
                PrintTempSensorType(ts->type_s);
                DebugSerial->print(F(") Abs Max ")); 
                DebugSerial->print(sd->AbsoluteMax);
                DebugSerial->print(F("*F at "));
                DebugPrintDateTime(sd->AbsoluteMaxTimeStamp);
                DebugSerial->print(F(", Abs Min: ")); 
                DebugSerial->print(sd->AbsoluteMin);
                DebugSerial->print(F("*F at "));
                DebugPrintDateTime(sd->AbsoluteMinTimeStamp);
                DebugSerial->println();
            }
        }   

        if (!Found[i] && ts->present)       // This is a new loss - at one time the sensor was found, but is no longer
        {
            ts->present = false; 
            ts->sensorLost = true; 
            ts->firstReadingAfterFound = true;              // Reset this flag
            SendDisplay(CMD_TEMP_LOST, ts->sensorName);     // We also want to tell the Teensy about the loss (command, value = sensorname)
            if (DEBUG)
            {
                PrintTempSensorName(ts->sensorName);
                DebugSerial->println(F(" temp sensor lost!"));
            }
        }

        // Otherwise if Found = Present, then no change has occurred and we don't need to print a message
    }
}


void InitTempStructs(void)
{
    InternalTemp.present = false;
    InternalTemp.known = false;             // Will be set below depending on whether an adddress exits in EEPROM
    InternalTemp.type_s = 0;
    InternalTemp.temperature = 999;
    InternalTemp.readingStarted = false;
    InternalTemp.newData = false; 
    InternalTemp.lastMeasure = 0;
    InternalTemp.sensorName = TS_INTERNAL;
    InternalTemp.constrained_temp = 0;
    InternalTemp.constrained_temp_prior = 0;
    InternalTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    InternalTemp.maxSessionTemp = -50;        
    InternalTemp.sensorLost = false;
    InternalTemp.firstReadingAfterFound = true;
    InternalTemp.shoeHorn = false;
    clearFIR(TEMP_NTAPS, InternalTemp.fir);
    
    ExternalTemp.present = false;
    ExternalTemp.known = false;             // Will be set below depending on whether an adddress exits in EEPROM    
    ExternalTemp.type_s = 0;
    ExternalTemp.temperature = 999;
    ExternalTemp.readingStarted = false;
    ExternalTemp.newData = false; 
    ExternalTemp.lastMeasure = 0;
    ExternalTemp.sensorName = TS_EXTERNAL;
    ExternalTemp.constrained_temp = 0;
    ExternalTemp.constrained_temp_prior = 0;
    ExternalTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    ExternalTemp.maxSessionTemp = -50;        
    ExternalTemp.sensorLost = false;
    ExternalTemp.firstReadingAfterFound = true;
    ExternalTemp.shoeHorn = false;
    clearFIR(TEMP_NTAPS, ExternalTemp.fir);

    AuxTemp.present = false;
    AuxTemp.known = false;                  // Will be set below depending on whether an adddress exits in EEPROM    
    AuxTemp.type_s = 0;
    AuxTemp.temperature = 999;
    AuxTemp.readingStarted = false;
    AuxTemp.newData = false; 
    AuxTemp.lastMeasure = 0;
    AuxTemp.sensorName = TS_AUX;
    AuxTemp.constrained_temp = 0;
    AuxTemp.constrained_temp_prior = 0;
    AuxTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    AuxTemp.maxSessionTemp = -50;        
    AuxTemp.sensorLost = false;
    AuxTemp.firstReadingAfterFound = true;
    AuxTemp.shoeHorn = false;
    clearFIR(TEMP_NTAPS, AuxTemp.fir);

    // Set addresses to what we have saved in EEPROM
    uint16_t IntAddressSum = 0;
    uint16_t ExtAddressSum = 0;
    uint16_t AuxAddressSum = 0;
    // We also sum all 8 bytes of the address, and if the total at the end is not zero, then we know what the device should be (whether detected or not)
    for( uint8_t i = 0; i < 8; i++) 
    {        
        InternalTemp.address[i] = eeprom.ramcopy.TempAddress_Int[i];
            IntAddressSum += eeprom.ramcopy.TempAddress_Int[i];
        
        ExternalTemp.address[i] = eeprom.ramcopy.TempAddress_Ext[i];
            ExtAddressSum += eeprom.ramcopy.TempAddress_Ext[i];
        
        AuxTemp.address[i] = eeprom.ramcopy.TempAddress_Aux[i];
            AuxAddressSum += eeprom.ramcopy.TempAddress_Aux[i];
    }
    
    if (IntAddressSum > 0) InternalTemp.known = true;
    else {DebugSerial->println(F("Internal temp not yet saved."));}
    if (ExtAddressSum > 0) ExternalTemp.known = true;
    else {DebugSerial->println(F("External temp not yet saved."));}
    if (AuxAddressSum > 0) AuxTemp.known = true;
    else {DebugSerial->println(F("Aux temp not yet saved."));}
}


void TempHandler(void)
{
    static _tempsensor *ts = &InternalTemp;   // Start with internal, then we cycle infinitely through the remainder and back again
    boolean swap = false; 

    // If we've disabled the handler, exit. This means the handler will stop until re-started with a new call. 
    if (TempHandlerEnabled == false) return;

    //Otherwise...
    if (ts->present)
    {
        if (ts->readingStarted == false && ts->newData == false)
        {
            ts->readingStarted = true;
            ts->newData = false;
            StartConversion(ts); 
        }
        else
        {
            if (ts->newData)
            {
                // Reading complete
                ts->newData = false;
                swap = true;
            }
            else
            {
                // This is the second part that we return to after some delay
                UpdateTemp(ts);
            }
        }
    }
    else
    {   // Sensor not present, do the next one
        swap = true;    
    }

    // Increment to next sensor
    if (swap)
    {
        switch (ts->sensorName)
        {
            case TS_INTERNAL:   
                // Next is external
                ts = &ExternalTemp; 
                break;

            case TS_EXTERNAL:
                // Next is aux
                ts = &AuxTemp;
                break;

            case TS_AUX:
                // Next is internal (complete the loop)
                ts = &InternalTemp;
                break; 
        }

        // Come back and do the swap
        timer.setTimeout(100, TempHandler); 
    }
}


void StartConversion(_tempsensor *ts)
{
    oneWire.reset();
    oneWire.select(ts->address);
    oneWire.write(0x44, 1);             // start conversion, with parasite power on at the end

    timer.setTimeout(1000, TempHandler);    // maybe 750ms is enough, maybe not
}


uint16_t UpdateTemp(_tempsensor *ts)
{
byte present = 0;
byte data[12];
float celsius, fahrenheit;
int16_t constrained;

    // This function should only be called after a conversion has been started and 1 second waited
    present = oneWire.reset();
    oneWire.select(ts->address);    
    oneWire.write(0xBE);                // Read Scratchpad
    
    for ( uint8_t i=0; i<9; i++)        // we need 9 bytes
    {   
        data[i] = oneWire.read();
        //if (DEBUG)
        //{
            //DebugSerial->print(data[i], HEX);
            //DebugSerial->print(" ");
        //}
    }
    //if (DEBUG)
    //{
        //DebugSerial->print(" CRC=");
        //DebugSerial->print( OneWire::crc8( data, 8), HEX);
        //DebugSerial->println();
    //}
        
    // Convert the data to actual temperature
    // Because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];

    if (ts->type_s == 1) 
    {
        raw = raw << 3;                 // 9 bit resolution default
        if (data[7] == 0x10) 
        {   // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } 
    else if (ts->type_s > 1)
    {
        byte cfg = (data[4] & 0x60);
        // At lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;        // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3;   // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1;   // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }

    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    if (ts->firstReadingAfterFound)
    {
        // Fill the FIR with our first reading
        setFIR(TEMP_NTAPS, ts->fir, fahrenheit);        
    }
    else
    {
        // Otherwise calculate the rolling average
        fahrenheit = fir_basic(fahrenheit, TEMP_NTAPS, ts->fir);   // Update average reading
    }
    constrained = constrain((int16_t)(fahrenheit + 0.5), -255, 255);
    
    if (ts->firstReadingAfterFound == false && (abs(ts->constrained_temp - constrained) > 20))
    {
        // If we end up at 0 all of a sudden (or 32 exactly fahrenheit), but the prior reading was 
        // something far away from that, ignore it. This is probably a disconnect. The constrained
        // temperature (which is average) should not change 10 degree fahrenheit in one reading cycle.
        // Of course it might be even more different than that if this is the first reading, but then 
        // again the odds of raw being 0 at the same time shouldn't happen. 
    }
    else
    {
        ts->temperature = fahrenheit;
        ts->constrained_temp = constrained;
        ts->readingStarted = false;
        ts->newData = true;
        ts->lastMeasure = millis();
        if (ts->sensorLost == true)
        {
            ts->sensorLost = false;         // We have a reading, sensor is not lost
            ts->present = true;
            ts->shoeHorn = false;           // Undo the shoeHorn workaround
            if (DEBUG)
            {
                PrintTempSensorName(ts->sensorName);
                DebugSerial->println(F(" temp sensor restored"));
            }
        }
    }
    // Either way, we've done some kind of reading, so set this to false until the next time it's lost
    ts->firstReadingAfterFound = false;
    
    /*
    if (DEBUG)
    {
        PrintTempSensorName(ts->sensorName);
        DebugSerial->print(F(" temp: ")); 
        DebugSerial->print(ts->constrained_temp); 
        // DebugSerial->print(fahrenheit,1);      // Precise version
        DebugSerial->println(F("* F")); 
    }
    */    
    // Go back to the handler
    TempHandler();
}

void SendTempInfo(void)
{
    const uint32_t INVALID_TIME = 30000;          // If we haven't gotten a reading in more than 30 seconds, consider the sensor disconnected (lost)
    
    _tempsensor *ts; 
    _saved_tempdata *sd;

    for (uint8_t i=0; i<NUM_TEMP_SENSORS; i++)
    {
        if (i == 0) { ts = &InternalTemp; sd = &eeprom.ramcopy.SavedInternalTemp; }
        if (i == 1) { ts = &ExternalTemp; sd = &eeprom.ramcopy.SavedExternalTemp; }
        if (i == 2) { ts = &AuxTemp;      sd = &eeprom.ramcopy.SavedAuxTemp; }

        // If sensor is present
        if (ts->present || ts->shoeHorn)
        {   // If we have a recent reading
            if (millis() - ts->lastMeasure < INVALID_TIME)
            {   
                if (ts->constrained_temp >= 0) 
                {
                    // Send curent positive temp
                    SendDisplay(CMD_TEMP_POSITIVE, ts->constrained_temp, ts->sensorName);       // command, value = temp, modifier = sensorname
                    
                    // Check for new session max temp
                    if (ts->constrained_temp > ts->maxSessionTemp) 
                    {
                        ts->maxSessionTemp = ts->constrained_temp;
                        SendDisplay(CMD_TEMP_MAX_POS, ts->maxSessionTemp, ts->sensorName);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New Session Max: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    // Check for new session min temp
                    if (ts->constrained_temp < ts->minSessionTemp) 
                    {
                        ts->minSessionTemp = ts->constrained_temp;
                        SendDisplay(CMD_TEMP_MIN_POS, ts->minSessionTemp, ts->sensorName);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New Session Min: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    
                    // Check for new all time positive max temp
                    if (ts->constrained_temp > sd->AbsoluteMax) 
                    {
                        // Update RAM
                        sd->AbsoluteMax = ts->constrained_temp;
                        CopyDateTime(CurrentDateTime, &sd->AbsoluteMaxTimeStamp);
                        // Update EEPROM
                        switch (ts->sensorName) // offsetof doesn't like pointers
                        {   case TS_INTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break;
                            case TS_EXTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break;
                            case TS_AUX:        
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break; 
                        }
                        SendDisplay(CMD_TEMP_ALLTIME_MAX_POS, ts->constrained_temp, ts->sensorName);
                        SendDisplayDateTime(CurrentDateTime);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New All-Time Max: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    
                    // Check for new all time positive min temp
                    if (ts->constrained_temp < sd->AbsoluteMin) 
                    {
                        // Update RAM
                        sd->AbsoluteMin = ts->constrained_temp;     
                        CopyDateTime(CurrentDateTime, &sd->AbsoluteMinTimeStamp);
                        // Update EEPROM
                        switch (ts->sensorName) // offsetof doesn't like pointers
                        {   case TS_INTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMin), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                                break;
                            case TS_EXTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMin), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                                
                                break;
                            case TS_AUX:        
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMin), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMinTimeStamp), CurrentDateTime);
                                break; 
                        }
                        SendDisplay(CMD_TEMP_ALLTIME_MIN_POS, ts->constrained_temp, ts->sensorName);                    
                        SendDisplayDateTime(CurrentDateTime);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New All-Time Min: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                }
                else        // NEGATIVE TEMPERATURES
                {
                    // Send current negative temp
                    SendDisplay(CMD_TEMP_NEGATIVE, ts->constrained_temp, ts->sensorName);
                    
                    // Check for new session max temp (which is below zero, unlikely)
                    if (ts->constrained_temp > ts->maxSessionTemp) 
                    {
                        ts->maxSessionTemp = ts->constrained_temp;
                        SendDisplay(CMD_TEMP_MAX_NEG, -ts->maxSessionTemp, ts->sensorName); // Re-minus
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New Session Max: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    
                    // Check for new session min temp (below zero, could happen)
                    if (ts->constrained_temp < ts->minSessionTemp) 
                    {
                        ts->minSessionTemp = ts->constrained_temp;
                        SendDisplay(CMD_TEMP_MIN_NEG, -ts->minSessionTemp, ts->sensorName); // Re-minus
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New Session Min: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    
                    // Check for new all time negative max temp (impossible basically)
                    if (ts->constrained_temp > sd->AbsoluteMax) 
                    {
                        // Update RAM
                        sd->AbsoluteMax = ts->constrained_temp;                 
                        CopyDateTime(CurrentDateTime, &sd->AbsoluteMaxTimeStamp);
                        // Update EEPROM
                        switch (ts->sensorName) // offsetof doesn't like pointers
                        {   case TS_INTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break;
                            case TS_EXTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break;
                            case TS_AUX:        
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMax), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMaxTimeStamp), CurrentDateTime);
                                break; 
                        }
                        SendDisplay(CMD_TEMP_ALLTIME_MAX_NEG, -ts->constrained_temp, ts->sensorName);   // Re-minus
                        SendDisplayDateTime(CurrentDateTime);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New All-Time Max: ")); DebugSerial->println(ts->constrained_temp); }
                    }
                    
                    // Check for new all time negative min temp (quite possible)
                    if (ts->constrained_temp < sd->AbsoluteMin) 
                    {
                        // Update RAM
                        sd->AbsoluteMin = ts->constrained_temp;
                        CopyDateTime(CurrentDateTime, &sd->AbsoluteMinTimeStamp);
                        // Update EEPROM
                        switch (ts->sensorName) // offsetof doesn't like pointers
                        {   case TS_INTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMin), ts->constrained_temp);
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedInternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);  
                                break;
                            case TS_EXTERNAL:   
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMin), ts->constrained_temp); 
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedExternalTemp.AbsoluteMinTimeStamp), CurrentDateTime);   
                                break;
                            case TS_AUX:        
                                EEPROM.updateInt(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMin), ts->constrained_temp);  
                                EEPROM.updateBlock(offsetof(_eeprom_data, SavedAuxTemp.AbsoluteMinTimeStamp), CurrentDateTime);  
                                break; 
                        }
                        SendDisplay(CMD_TEMP_ALLTIME_MIN_NEG, -ts->constrained_temp, ts->sensorName);   // Re-minus
                        SendDisplayDateTime(CurrentDateTime);
                        if (DEBUG) { PrintTempSensorName(ts->sensorName); DebugSerial->print(F(" Temp, New All-Time Min: ")); DebugSerial->println(ts->constrained_temp); }
                    }                    
                }
                
                if (DEBUG)
                {
                    // Only show if a change occured
                    if (ts->constrained_temp != ts->constrained_temp_prior)
                    {
                        PrintTempSensorName(ts->sensorName);
                        DebugSerial->print(F(" Temp: ")); 
                        DebugSerial->print(ts->constrained_temp); 
                        DebugSerial->println();
                        ts->constrained_temp_prior = ts->constrained_temp;
                    }
                }
            }
            else
            {   // We have not had a recent reading - should this sensor be set to lost? 
                if (ts->sensorLost == false)
                {
                    ts->sensorLost = true;                                  // Set lost
                    ts->firstReadingAfterFound = true;                      // Reset this flag
                    SendDisplay(CMD_TEMP_LOST, ts->sensorName);             // command, value = sensorname
                    if (DEBUG)
                    {
                        PrintTempSensorName(ts->sensorName);
                        DebugSerial->println(F(" temp sensor lost!")); 
                    }
                }
            }
        }
    }
}

void SendDisplayAllTimeTemps()
{
uint8_t cmd;  
// We let the display know what the all time min/max temps are for each sensor, once every time the display is turned on

    // Internal: all-time min
    eeprom.ramcopy.SavedInternalTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedInternalTemp.AbsoluteMin, InternalTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedInternalTemp.AbsoluteMinTimeStamp);
    // Internal: all-time max
    eeprom.ramcopy.SavedInternalTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedInternalTemp.AbsoluteMax, InternalTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedInternalTemp.AbsoluteMaxTimeStamp);
    // External: all-time min
    eeprom.ramcopy.SavedExternalTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedExternalTemp.AbsoluteMin, ExternalTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedExternalTemp.AbsoluteMinTimeStamp);
    // External: all-time max
    eeprom.ramcopy.SavedExternalTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedExternalTemp.AbsoluteMax, ExternalTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedExternalTemp.AbsoluteMaxTimeStamp);
    // Aux: all-time min
    eeprom.ramcopy.SavedAuxTemp.AbsoluteMin < 0 ? cmd = CMD_TEMP_ALLTIME_MIN_NEG : cmd = CMD_TEMP_ALLTIME_MIN_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedAuxTemp.AbsoluteMin, AuxTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedAuxTemp.AbsoluteMinTimeStamp);
    // Aux: all-time max
    eeprom.ramcopy.SavedAuxTemp.AbsoluteMax < 0 ? cmd = CMD_TEMP_ALLTIME_MAX_NEG : cmd = CMD_TEMP_ALLTIME_MAX_POS;
    SendDisplay(cmd, eeprom.ramcopy.SavedAuxTemp.AbsoluteMax, AuxTemp.sensorName);
    SendDisplayDateTime(eeprom.ramcopy.SavedAuxTemp.AbsoluteMaxTimeStamp);
}


void PrintTempSensorName(char n)
{
    switch (n)
    {
        case TS_INTERNAL:   DebugSerial->print(F("Internal"));    break;
        case TS_EXTERNAL:   DebugSerial->print(F("External"));    break;
        case TS_AUX:        DebugSerial->print(F("Aux"));         break;
    }
}

void PrintTempSensorType(byte s)
{
    switch (s)
    {
        case 1: DebugSerial->print(F("DS18S20"));   break;
        case 2: DebugSerial->print(F("DS18B20"));   break;
        case 3: DebugSerial->print(F("DS1822"));    break;
        default: DebugSerial->print(F("Unknown"));  break;
    }
}

