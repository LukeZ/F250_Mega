

boolean CarOn(void)
{
    if (digitalRead(Run_Accy) == DI_High || digitalRead(Run_Start) == DI_High) { return true; } else { return false; }
}    

