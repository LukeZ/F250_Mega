
void TurnOffGPS(void) 
{
    digitalWrite(GPS_EN, LOW);
}

void TurnOnGPS(void) 
{
    digitalWrite(GPS_EN, HIGH);
}
