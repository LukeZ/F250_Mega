
// This routine gets called whenever the input pin from the Viper Channel 2 signal changes
void ToggleRearLights(void)
{
    // When the Aux button on the keyfob is held down for more than 1.5 seconds, the Viper will hold this line to ground for as long as it is held.
    // Typically this is used for a trunk release. In our case, we want a latched on/off button to toggle the rear aux lights. You can already control
    // the dome/bed lights by holding both lock & unlock buttons at the same time. That trigger is latching, but this one is not, so we implement it here. 

    // If the ISR gets triggered, it means the pin FELL, meaning, it was brought to ground, meaning, the user pushed the Aux button on the keyfob, meaning, do something (toggle state).
    // But all we do is set a flag and let the loop take care of it through Polls();
    UpdateLightsFlag = true;
}

void HandleUpdateLightsFlag(void)
{
    static boolean RearLightsOn = false; 

    UpdateLightsFlag = false;               // We're handling this, clear the handle flag

    // First of all, ignore all activity during the debounce time
    if (millis() - TimeAuxRearLightsChanged < AuxRearLightsDebounceTime) return;
    
    // Ok, enough time has passed since the last command, toggle the lights
    if (RearLightsOn)
    {
        // Turn them off
        TurnOffAuxRearLights();
        RearLightsOn = false; 
        if (DEBUG) DebugSerial->println("Rear Lights Off");
    }
    else
    {
        // Turn them on
        TurnOnAuxRearLights();
        RearLightsOn = true;
        // But also - set a timer that will automatically turn them off after some length of time, in case the user forgets
        timer.setTimeout(300000L, TurnOffAuxRearLights);    // We wait 5 minutes for auto-off. Of course this will only work so long as timer is being polled

        // FOR TESTING - your ears will destroy!
//        SoundHorn(500);
        if (DEBUG) DebugSerial->println("Rear Lights On");    
    }
}

void TurnOnAuxRearLights()
{
    digitalWrite(K2_AuxRearLights, LOW);                            // Lights on
    TimeAuxRearLightsChanged = millis();                            // Save the time
}

void TurnOffAuxRearLights()
{
    digitalWrite(K2_AuxRearLights, HIGH);                           // Lights off
    TimeAuxRearLightsChanged = millis();                            // Save the time
}

void SoundHorn(int howLong)
{
    HornOn();                               // Turn the horn on
    timer.setTimeout(howLong, HornOff);     // Set a task to turn it off after howLong. Better hope we are polling the timer object!
}

void HornOn()
{
    digitalWrite(K1_Whelen, LOW);
}

void HornOff()
{
    digitalWrite(K1_Whelen, HIGH);
}

void TurnOnRelays(void) 
{   // Pin held to ground closes the relays
    digitalWrite(K1_Whelen, LOW);
    digitalWrite(K2_AuxRearLights, LOW);
    digitalWrite(K3_, LOW);
    digitalWrite(K4_, LOW);
}

void TurnOffRelays(void) 
{   // Pin held high opens the relays
    digitalWrite(K1_Whelen, HIGH);
    digitalWrite(K2_AuxRearLights, HIGH);
    digitalWrite(K3_, HIGH);
    digitalWrite(K4_, HIGH);
}

