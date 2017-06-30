
void TurnOffCellPhone(void) 
{
    gsm.shutdown();
}

void TurnOnCellPhone(void) 
{
    digitalWrite(GPS_EN, HIGH);
}
