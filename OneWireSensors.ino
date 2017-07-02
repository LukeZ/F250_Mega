
void StartTempReadings(void)
{
    // This will attempt to detect and identify any temperature sensors on the bus
    SetupOneWireSensors();                                          
    
    // If any sensors are detected, start a temperature reading process that will repeat itself automatically
    if (InternalTemp.present || ExternalTemp.present || AuxTemp.present) 
    { 
        TempHandlerEnabled = true;
        TempHandler();                      // The handler cycles through each sensor on the bus and continuously takes readings and averages them, updates the sensor structs, etc... 
                                            // It only needs to be called once to start (with TempHandlerEnabled = true) and from there it will keep going forever until TempHandlerEnabled = false.         
        
        // This function sends temperature data to the Teensy at the set interval
        if (TimerID_TempSender == 0) TimerID_TempSender = timer.setInterval(TEMP_SEND_FREQ, SendTempInfo);            
        else if (timer.isEnabled(TimerID_TempSender) == false) timer.enable(TimerID_TempSender);

        if (TimerID_TempSensorCheck == 0) TimerID_TempSensorCheck = timer.setInterval(TEMP_SENSOR_CHECK_FREQ, SetupOneWireSensors);
        else if (timer.isEnabled(TimerID_TempSensorCheck) == false) timer.enable(TimerID_TempSensorCheck); 
    }
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
uint8_t sensorCount = 0;
boolean correctType = false; 

boolean Found[NUM_TEMP_SENSORS] = {false, false, false};
_tempsensor *ts; 
    
    // See if anything is attached
    while(oneWire.search(addr) == true && sensorCount < 3)
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
            sensorCount += 1;

            // See if we have this one saved already
            // Internal? 
            if (memcmp(addr, eeprom.ramcopy.TempAddress_Int, 8) == 0)
            {   // Yes, this is the internal sensor
                for (uint8_t i=0; i<8; i++) { InternalTemp.address[i] = addr[i]; }
                InternalTemp.type_s = type_s;
                Found[TS_INTERNAL] = true; 
            }
            // External? 
            else if (memcmp(addr, eeprom.ramcopy.TempAddress_Ext, 8) == 0)
            {   // Yes, this is the external sensor
                for (uint8_t i=0; i<8; i++) { ExternalTemp.address[i] = addr[i]; }
                ExternalTemp.type_s = type_s;
                Found[TS_EXTERNAL] = true;
            }
            // Aux? 
            else if (memcmp(addr, eeprom.ramcopy.TempAddress_Aux, 8) == 0)
            {   // Yes, this is the aux sensor
                for (uint8_t i=0; i<8; i++) { AuxTemp.address[i] = addr[i]; }
                AuxTemp.type_s = type_s;
                Found[TS_AUX] = true;
            }
            else
            {   // No, we haven't seen this sensor before. We save the first previously-unseen sensor as internal, the second as external, third as aux
                // This means if you want to save the internal sensor, plug it in but not the external, boot up and let it save. Now disconnect power, 
                // plug in the external sensor, and reboot. Now disconnect power again, connect the aux, and reboot. 
                if (sensorCount == 1)   
                {
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Int[i] = addr[i]; InternalTemp.address[i] = addr[i]; }
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Int), InternalTemp.address);
                    InternalTemp.type_s = type_s;
                    Found[TS_INTERNAL] = true;
                }
                else if (sensorCount == 2)
                {
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Ext[i] = addr[i]; ExternalTemp.address[i] = addr[i]; }
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Ext), ExternalTemp.address);
                    ExternalTemp.type_s = type_s;
                    Found[TS_EXTERNAL] = true;
                }
                else if (sensorCount == 3)
                {
                    for (uint8_t i = 0; i<8; i++)   { eeprom.ramcopy.TempAddress_Aux[i] = addr[i]; AuxTemp.address[i] = addr[i]; }
                    EEPROM.updateBlock(offsetof(_eeprom_data, TempAddress_Aux), AuxTemp.address);
                    AuxTemp.type_s = type_s;
                    Found[TS_AUX] = true; 
                }                
            }
        }
    }

    for (uint8_t i=0; i<NUM_TEMP_SENSORS; i++)
    {
        if (i == 0) ts = &InternalTemp;
        if (i == 1) ts = &ExternalTemp;
        if (i == 2) ts = &AuxTemp; 
                
        if (Found[i] && !ts->present)       // This is a new find
        {
            ts->present = true; 
            ts->sensorLost = false; 
            if (DEBUG) 
            {
                PrintTempSensorName(ts->sensorName);
                DebugSerial->print(F(" temp sensor found (Type: "));
                DebugSerial->print(ts->type_s); 
                DebugSerial->println(F(")")); 
            }
        }   

        if (!Found[i] && ts->present)       // This is a new loss - at one time the sensor was found, but is no longer
        {
            ts->present = false; 
            ts->sensorLost = true; 
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
    InternalTemp.type_s = 0;
    InternalTemp.temperature = 999;
    InternalTemp.readingStarted = false;
    InternalTemp.newData = false; 
    InternalTemp.lastMeasure = 0;
    InternalTemp.sensorName = TS_INTERNAL;
    InternalTemp.constrained_temp = 0;
    InternalTemp.sensorLost = false;
    clearFIR(TEMP_NTAPS, InternalTemp.fir);
    
    ExternalTemp.present = false;
    ExternalTemp.type_s = 0;
    ExternalTemp.temperature = 999;
    ExternalTemp.readingStarted = false;
    ExternalTemp.newData = false; 
    ExternalTemp.lastMeasure = 0;
    ExternalTemp.sensorName = TS_EXTERNAL;
    ExternalTemp.constrained_temp = 0;
    ExternalTemp.sensorLost = false;
    clearFIR(TEMP_NTAPS, ExternalTemp.fir);

    AuxTemp.present = false;
    AuxTemp.type_s = 0;
    AuxTemp.temperature = 999;
    AuxTemp.readingStarted = false;
    AuxTemp.newData = false; 
    AuxTemp.lastMeasure = 0;
    AuxTemp.sensorName = TS_AUX;
    AuxTemp.constrained_temp = 0;
    AuxTemp.sensorLost = false;
    clearFIR(TEMP_NTAPS, AuxTemp.fir);
    
    for( uint8_t i = 0; i < 8; i++) 
    {        
        InternalTemp.address[i] = 0;
        ExternalTemp.address[i] = 0;
        AuxTemp.address[i] = 0;
    }
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

    ts->temperature = fir_basic(fahrenheit, TEMP_NTAPS, ts->fir);   // Update average reading
    ts->constrained_temp = constrain((int16_t)(fahrenheit + 0.5), -255, 255);
    ts->readingStarted = false;
    ts->newData = true;
    ts->lastMeasure = millis();
    if (ts->sensorLost == true)
    {
        ts->sensorLost = false;         // We have a reading, sensor is not lost
        if (DEBUG)
        {
            PrintTempSensorName(ts->sensorName);
            DebugSerial->println(F(" temp sensor restored"));
        }
    }
    
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

    for (uint8_t i=0; i<NUM_TEMP_SENSORS; i++)
    {
        if (i == 0) ts = &InternalTemp;
        if (i == 1) ts = &ExternalTemp;
        if (i == 2) ts = &AuxTemp; 

        // If sensor is present
        if (ts->present)
        {   // If we have a recent reading
            if (millis() - ts->lastMeasure < INVALID_TIME)
            {   
                if (ts->constrained_temp >= 0) SendDisplay(CMD_TEMP_POSITIVE, ts->constrained_temp, ts->sensorName);       // command, value = temp, modifier = sensorname
                else                          SendDisplay(CMD_TEMP_NEGATIVE, ts->constrained_temp, ts->sensorName);
                if (DEBUG)
                {
                    // DebugSerial->print(F("Send "));
                    PrintTempSensorName(ts->sensorName);
                    DebugSerial->print(F(" Temp: ")); 
                    DebugSerial->print(ts->constrained_temp); 
                    // DebugSerial->print(F("* F at Time: ")); 
                    // DebugSerial->print(ts->lastMeasure);
                    DebugSerial->println();
                }
            }
            else
            {   // We have not had a recent reading - should this sensor be set to lost? 
                if (ts->sensorLost == false)
                {
                    ts->sensorLost = true;                                  // Set lost
                    if (DEBUG)
                    {
                        SendDisplay(CMD_TEMP_LOST, ts->sensorName);             // command, value = sensorname
                        PrintTempSensorName(ts->sensorName);
                        DebugSerial->println(F(" temp sensor lost!")); 
                    }
                }
            }
        }
    }
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


