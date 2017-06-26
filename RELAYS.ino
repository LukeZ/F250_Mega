

void TurnOffRelays(void) 
{
    digitalWrite(K1_Whelen, LOW);
    digitalWrite(K2_AuxRearLights, LOW);
    digitalWrite(K3_, LOW);
    digitalWrite(K4_, LOW);
}

void TurnOffRelays_Alt(void) 
{
    digitalWrite(K1_Whelen, HIGH);
    digitalWrite(K2_AuxRearLights, HIGH);
    digitalWrite(K3_, HIGH);
    digitalWrite(K4_, HIGH);
}

