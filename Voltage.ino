
void SetupVoltageSensor()
{
    setFIR(VOLTAGE_NTAPS, BattVoltageFIR, MeasureVoltage());                            // This pre-fills the FIR line with the first reading
}

void StartVoltageReadings()
{   
    // This function sends the current averaged voltage reading to the display on a routine schedule
    if (TimerID_VoltageSender == 0) TimerID_VoltageSender = timer.setInterval(VOLTAGE_SEND_FREQ, SendVoltageInfo);
    else if (timer.isEnabled(TimerID_VoltageSender) == false) timer.enable(TimerID_VoltageSender);

    // This function updates the voltage reading (and averages it) at some interval
    if (TimerID_VoltageSensorCheck == 0) TimerID_VoltageSensorCheck = timer.setInterval(VOLTAGE_CHECK_FREQ, UpdateVoltage);
    else if (timer.isEnabled(TimerID_VoltageSensorCheck) == false) timer.enable(TimerID_VoltageSensorCheck);     

    if (DEBUG) DebugSerial->println(F("Voltage Readings Started"));
}

void PauseVoltageReadings()
{
    if (TimerID_VoltageSender > 0)      timer.disable(TimerID_VoltageSender);           // Pause the routine submission of voltage data to the display
    if (TimerID_VoltageSensorCheck > 0) timer.disable(TimerID_VoltageSensorCheck);      // Pause the routine update of averaged voltage readings
    
    if (DEBUG) DebugSerial->println(F("Voltage Readings Paused"));
}

float MeasureVoltage(void)
{
    int VRaw; 
    VRaw = analogRead(AttoVoltage);
    return (VRaw/49.44);                                                                // AttoPilot 45 Amp board https://www.sparkfun.com/products/10643
}

void UpdateVoltage()
{
    BattVoltage = fir_basic(MeasureVoltage(), VOLTAGE_NTAPS, BattVoltageFIR);           // Update averaged reading    
}

void SendVoltageInfo()
{
    // Multiply by 10, constrain to 255 (ie, we can't read over 25.5 volts), convert to an integer, and send. 
    // On the opposite end we will divide by 10 to get a single decimal point precision

    float   fSendVoltage;   // Float voltage
    uint8_t iSendVoltage;   // Integer voltage
    
    fSendVoltage = BattVoltage * 10.0;                                                  // Multiply by 10 to move the decimal point over
    iSendVoltage = constrain((int16_t)fSendVoltage, 0, 255);                            // Convert to integer (lose anything beyond one decimal point), the constrain to 255 = 25.5 volts, we should never reach that,
                                                                                        // and anyway the sensor will saturate at ~20 volts. 
    
    SendDisplay(CMD_VOLTAGE, iSendVoltage);                                             // Chips ahoy! 

    if (DEBUG && (int32_t)BattVoltage != (int32_t)BattVoltage_Prior)
    {
        DebugSerial->print(F("Battery voltage: ")); 
        DebugSerial->print(BattVoltage, 1);
        DebugSerial->print(F(" ("));
        DebugSerial->print(iSendVoltage);
        DebugSerial->println(F(")"));     
        BattVoltage_Prior = BattVoltage; 
    }        
}



