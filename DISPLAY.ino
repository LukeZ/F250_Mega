
void TurnOnDisplay(void) 
{
    SendDisplay(CMD_DISPLAY_TURN_ON);
}

void TurnOffDisplay(void) 
{
    SendDisplay(CMD_DISPLAY_TURN_OFF);
}

void UpdateDisplay(void)
{   // As opposed to PollInputs on the main tab, which sends display information whenever something changes, this function gets called at routine intervals 
    // and sends data to the display whether it needs it or not. Basically it is a redundancy in case the PollInputs communication failed. 
    // Note also, this doesn't send _all_ data, but only that which isn't self updating already (things like the temp sensors & GPS etc take care of sending their own updates)
    
    Ham_On ? SendDisplay(CMD_HAM_ON, true) : SendDisplay(CMD_HAM_ON, false);                        // Ham/CB microphone selector
    FuelPump_On ? SendDisplay(CMD_FUEL_PUMP, true) : SendDisplay(CMD_FUEL_PUMP, false);             // Fuel pump
    SendDisplay(CMD_TQC_LOCK_STATUS, TQCLockState);                                                 // Torque converter lockup state
    OverdriveEnabled ? SendDisplay(CMD_OVERDRIVE, true) : SendDisplay(CMD_OVERDRIVE, false);        // Overdrive enabled
    AlternateTransSetting ? SendDisplay(CMD_TRANS_TABLE, 2) : SendDisplay(CMD_TRANS_TABLE, 1);      // Baumann Table 2 (alternate transmission configuration settings - negative signal means Table 2 active)
    Low_Air_Warning ? SendDisplay(CMD_LOW_AIR_WARN, true) : SendDisplay(CMD_LOW_AIR_WARN, false);   // Low air tank warning

    // Yeah ok, temp sensors are good at sending current temp regularly, but not min/maxes
    _tempsensor *ts; 
    for (uint8_t i=0; i<NUM_TEMP_SENSORS; i++)
    {
        if (i == 0) { ts = &InternalTemp; }
        if (i == 1) { ts = &ExternalTemp; }
        if (i == 2) { ts = &AuxTemp;      }
        if (ts->maxSessionTemp > 0) SendDisplay(CMD_TEMP_MAX_POS, ts->maxSessionTemp, ts->sensorName);
        if (ts->maxSessionTemp < 0) SendDisplay(CMD_TEMP_MAX_NEG, -ts->maxSessionTemp, ts->sensorName); // Re-minus
        if (ts->minSessionTemp > 0) SendDisplay(CMD_TEMP_MIN_POS, ts->minSessionTemp, ts->sensorName);
        if (ts->minSessionTemp < 0) SendDisplay(CMD_TEMP_MIN_NEG, -ts->minSessionTemp, ts->sensorName); // Re-minus
        if (ts->sensorLost)         SendDisplay(CMD_TEMP_LOST, ts->sensorName);                         // Every now and then, remind it if a sensor is not present
    }
    
}

