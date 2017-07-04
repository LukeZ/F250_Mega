#ifndef COMMS_H
#define COMMS_H

// Teensy Display Address
#define DISPLAY_ADDRESS                      0xDA    // 218  Address of the Teensy display controller
#define MASTER_ADDRESS                       0x56    // 86   Address of the ATmega2560 master device

// Commands                                         
#define CMD_DISPLAY_TURN_OFF            0x0E    // 14
#define CMD_DISPLAY_TURN_ON             0x0F    // 15

#define RCV_CMD_SET_HOME_COORD          0x10    // 16   If GPS fix obtained, set current coordinates to home
#define RCV_CMD_SET_HOME_ALT            0x11    // 17   Set home altitude to value * 100 + modifier (tens)
#define RCV_CMD_SET_CURRENT_ALT         0x12    // 18   Set current altitude adjustment using this altitude (value * 100) + modifier (tens)
#define RCV_CMD_SET_TIMEZONE            0x13    // 19   AKST, PST, MST, CST, EST passed in value
/*
0x14    // 20
0x15    // 21
0x16    // 22
0x17    // 23
0x18    // 24
0x19    // 25
0x1A    // 26
0x1B    // 27
0x1C    // 28
0x1D    // 29
0x1E    // 30
0x1F    // 31
*/

#define CMD_TEMP_POSITIVE               0x2C    // 44   Positive temperature value (saves us from needing to send the sign)
#define CMD_TEMP_NEGATIVE               0x2D    // 45   Negative temperature value (saves us from needing to send the sign)
#define CMD_TEMP_LOST                   0x2E    // 46   Command gets sent if a sensor is lost, this lets the display not to rely on the last reading forever. Next time a temp is sent means it has been found again. 
#define CMD_HAM_ON                      0x2F    // 47   When the Ham is on, the CB is off. The Teensy knows this, so sending one or the other command accomplishes the same thing. 
#define CMD_CB_ON                       0x30    // 48   When the CB is on, the Ham is off. The Teensy knows this, so sending one or the other command accomplishes the same thing. 
#define CMD_FUEL_PUMP_ON                0x31    // 49
#define CMD_FUEL_PUMP_OFF               0x32    // 50
#define CMD_TQC_AUTO                    0x33    // 51   Torque converter lockup status controlled by Baumann controller
#define CMD_TQC_FORCE_LOCK              0x34    // 52   Torque converter forced lock in gears 3 and 4. Good for highway or long descents to assist brakes. 
#define CMD_TQC_FORCE_UNLOCK            0x35    // 53   Torque converted forced unlock all gears. Good for city driving. 
#define CMD_OVERDRIVE_ON                0x36    // 54   Overdrive is enabled
#define CMD_OVERDRIVE_OFF               0x37    // 55   Overdrive is disabled
#define CMD_LOW_AIR_WARN                0x38    // 56   Low air warning (suspension air tank)
#define CMD_AIR_RESTORED                0x39    // 57   Air pressure restored (suspension air tank)
#define CMD_TRANS_TABLE2                0x3A    // 58   Baumann using Table 2 transmission settings
#define CMD_TRANS_TABLE1                0x3B    // 59   Baumann using Table 1 (default) transmission settings
#define CMD_VOLTAGE                     0x3C    // 60   Send voltage multiplied by 10 (divide by 10 on opposite end)
#define CMD_GPS_ALTITUDE_POS            0x3D    // 61   Altitude in feet, positive numbers. Value = alt divided by 100, modifier = remainder of feet less than 100 (add to val*100)
#define CMD_GPS_ALTITUDE_NEG            0x3E   // 62    Altitude in feet, negative numbers, in case I go to Death Valley. 
#define CMD_SPEED_MPH                   0x3F    // 63   Speed in MPH. If modifier = 1, this is the maximum speed this session. If modifier = 0, just regular instantaneous speed. 
#define CMD_GPS_FIX                     0x40    // 64   GPS Fix status update. Value is boolean for fix, Modifier is 0 = no fix, 1 = GPS, 2 = DGPS (very accurate)
#define CMD_GPS_SATELLITES              0x41    // 65   Number of satellites
#define CMD_GPS_YEAR                    0x42    // 66   Value holds year after 2000
#define CMD_MONTH_DAY                   0x43    // 67   Value holds month (1-12), Modifier holds day (0-31)
#define CMD_HOUR_MINUTE                 0x44    // 68   Value holds hour (0-23), Modifier holds minute (0-59)
#define CMD_BEEP_ONCE                   0x45    // 69
#define CMD_BEEP_X                      0x46    // 70
#define CMD_SET_VOLUME                  0x47    // 71
#define CMD_BRAKE_SOUND                 0x48    // 72

// Modifiers
#define MAX_NUM_SQUEAKS                  6     // How many squeaks can this device implement
#define MAX_NUM_CLACKS                   6     // How many clacks can this device implement
#define MAX_NUM_USER_SOUNDS              4     // How many user sounds does this device implement


#endif // COMMS_H



