
_TQC_STATE GetTorqueConverterState(void)
{
    // Torque converted locked status    
    if      (digitalRead(TC_Locked) == DI_High)     return TQC_FORCE_LOCK;
    else if (digitalRead(TC_Unlocked) == DI_High)   return TQC_FORCE_UNLOCK;
    else                                            return TQC_AUTO;
}

