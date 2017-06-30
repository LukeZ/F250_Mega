
boolean IsCarOn(void)
{
    // We have two inputs, "Run & Accy" or "Run & Start". We don't have a means (directly) to actually determine if the car
    // is in Run or Accessory, only if the car is "on" generally. 
    // Perhaps with other sensors (speed for one, either form GPS or Baumann speed sensor) we could tell if the engine is running, but for now, we only
    // have a generic "on" or off"
    if (digitalRead(Run_Accy) == DI_High || digitalRead(Run_Start) == DI_High) { return true; } else { return false; }
}    

