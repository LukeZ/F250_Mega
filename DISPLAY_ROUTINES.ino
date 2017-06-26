

void TurnOffDisplay(void) 
{
    analogWrite(TFT_PWM, 0);
//    digitalWrite(TFT_PWM, LOW);
}

void TurnOnDisplay(void)
{
    analogWrite(TFT_PWM, Backlight);
//    digitalWrite(TFT_PWM, HIGH);

//    tft.fillScreen(ILI9340_BLACK);
}


