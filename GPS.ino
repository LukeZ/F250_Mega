
void TurnOnGPS(void) 
{
    // GPS initialize
    digitalWrite(GPS_EN, HIGH);         // Power on
    GPS.begin(9600);                   // 9600 NMEA is the default baud rate for MTK - some use 4800
}

void TurnOffGPS(void) 
{
    digitalWrite(GPS_EN, LOW);
}

