
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
    
    Ham_On ? SendDisplay(CMD_HAM_ON) : SendDisplay(CMD_CB_ON);                              // Ham/CB microphone selector
    FuelPump_On ? SendDisplay(CMD_FUEL_PUMP, true) : SendDisplay(CMD_FUEL_PUMP, false);     // Fuel pump
    switch (TorqueConverterState)                                                           // Torque converter lockup state
    {   
        case TQC_AUTO:          SendDisplay(CMD_TQC_AUTO);          break;                  // Auto
        case TQC_FORCE_LOCK:    SendDisplay(CMD_TQC_FORCE_LOCK);    break;                  // Forced lock in 3rd and 4th gear
        case TQC_FORCE_UNLOCK:  SendDisplay(CMD_TQC_FORCE_UNLOCK);  break;                  // Forced unlock
    }
    OverdriveEnabled ? SendDisplay(CMD_OVERDRIVE_ON) : SendDisplay(CMD_OVERDRIVE_OFF);      // Overdrive enabled
    AlternateTransSetting ? SendDisplay(CMD_TRANS_TABLE2) : SendDisplay(CMD_TRANS_TABLE1);  // Baumann Table 2 (alternate transmission configuration settings - negative signal means Table 2 active)
    Low_Air_Warning ? SendDisplay(CMD_LOW_AIR_WARN) : SendDisplay(CMD_AIR_RESTORED);        // Low air tank warning (negative signal is the warning)
}

