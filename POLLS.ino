
void Polls(void)
{
    // Stuff that needs polling
    
    if (UpdateLightsFlag) HandleUpdateLightsFlag();         // If this flag is set, the pin-change ISR tripped. Handle it. 
}

