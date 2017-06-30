
void Polls(void)
{
    // Stuff that needs polling
    
    if (UpdateLightsFlag) HandleUpdateLightsFlag();         // If this flag is set, the ISR tripped. Handle it. 
    
}

