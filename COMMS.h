#ifndef COMMS_H
#define COMMS_H

// Teensy Display Address
#define DISPLAY_ADDRESS                      0xDA    // 218  Address of the Teensy display controller
#define MASTER_ADDRESS                       0x56    // 86   Address of the ATmega2560 master device

// Commands                                         
#define CMD_DISPLAY_TURN_OFF            0x0E    // 14
#define CMD_DISPLAY_TURN_ON             0x0F    // 15

// "Receive" is from the perspective of the Mega, receiving data from the display, which is the a-typical case
#define RCV_CMD_SET_HOME_COORD          0x10    // 16   If GPS fix obtained, set current coordinates to home
#define RCV_CMD_SET_HOME_ALT            0x11    // 17   Set home altitude to value * 100 + modifier (tens)
#define RCV_CMD_SET_CURRENT_ALT         0x12    // 18   Set current altitude adjustment using this altitude (value * 100) + modifier (tens)
#define RCV_CMD_SET_GPS_ALT             0x13    // 19   Set current altitude adjustment based on current GPS altitude - ie, we assume (or know) the GPS is correct
#define RCV_CMD_SET_TIMEZONE            0x14    // 20   AKST, PST, MST, CST, EST passed in value
#define RCV_CMD_RESET_ABS_TEMP          0x15    // 21   Will reset the absolute (all-time) min max temperature settings for sensor passed in modifier (basically sets them to temp right now)
/*
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
#define CMD_TEMP_POSITIVE               0x24    // 36   Positive temperature value (saves us from needing to send the sign). Value = temp, Modifier = sensor num.
#define CMD_TEMP_NEGATIVE               0x25    // 37   Negative temperature value (saves us from needing to send the sign). Value = temp, Modifier = sensor num.
#define CMD_TEMP_MIN_POS                0x26    // 38   Session min temp (positive). Value = temp, Modifier = sensor num.
#define CMD_TEMP_MIN_NEG                0x27    // 39   Session min temp (negative). Value = temp, Modifier = sensor num.
#define CMD_TEMP_MAX_POS                0x28    // 40   Session max temp (positive). Value = temp, Modifier = sensor num.
#define CMD_TEMP_MAX_NEG                0x29    // 41   Session max temp (negative). Value = temp, Modifier = sensor num.
#define CMD_TEMP_ALLTIME_MIN_POS        0x2A    // 42   All time minimum temp (negative). Value = temp, Modifier = sensor num.
#define CMD_TEMP_ALLTIME_MIN_NEG        0x2B    // 43   All time minum temp (positive). Value = temp, Modifier = sensor num.
#define CMD_TEMP_ALLTIME_MAX_POS        0x2C    // 44   All time maximum temp (positive). Value = temp, Modifier = sensor num.
#define CMD_TEMP_ALLTIME_MAX_NEG        0x2D    // 45   All time maximum temp (negative - highly unlikely). Value = temp, Modifier = sensor num.
#define CMD_TEMP_LOST                   0x2E    // 46   Command gets sent if a sensor is lost or not present, this lets the display not to rely on the last reading forever. Next time a temp is sent means it has been found again. Sensor sent in value.
#define CMD_HAM_ON                      0x2F    // 47   When the Ham is on (value = 1), the CB is off, and vice versa (value = 0)
#define CMD_XXX                         0x30    // 48   UNUSED
#define CMD_FUEL_PUMP                   0x31    // 49   Value is on/off
#define CMD_YYY                         0x32    // 50   UNUSED
#define CMD_TQC_LOCK_STATUS             0x33    // 51   Torque converter lockup status controlled by Baumann controller. Value 0 = Auto, 1 = Force Lock, 2 = Force Unlock
#define CMD_AAA                         0x34    // 52   UNUSED
#define CMD_BBB                         0x35    // 53   UNUSED
#define CMD_OVERDRIVE                   0x36    // 54   Overdrive. Value = true if enabled, false if disabled
#define CMD_HOME_ALT                    0x37    // 55   Pass the home altitude to the display in feet, positive numbers ONLY. Value = alt divided by 100, modifier = remainder of feet less than 100 (add to val*100)
#define CMD_LOW_AIR_WARN                0x38    // 56   Low air warning (suspension air tank). Value = 1 if Low, 0 = if OK
#define CMD_ZZZ                         0x39    // 57   UNUSED
#define CMD_TRANS_TABLE                 0x3A    // 58   Baumann settings - value = 1 default, 2 = Table 2
#define CMD_USE_PRESSURE_ALT            0x3B    // 59   If Value = true, display altitude from barometer, if False use GPS
#define CMD_VOLTAGE                     0x3C    // 60   Send voltage multiplied by 10 (divide by 10 on opposite end)
#define CMD_GPS_ALTITUDE_POS            0x3D    // 61   Altitude in feet, positive numbers. Value = alt divided by 100, modifier = remainder of feet less than 100 (add to val*100)
#define CMD_GPS_ALTITUDE_NEG            0x3E    // 62   Altitude in feet, negative numbers, in case I go to Death Valley. 
#define CMD_SPEED_MPH                   0x3F    // 63   Speed in MPH. If modifier = 1, this is the maximum speed this session. If modifier = 0, just regular instantaneous speed. 
#define CMD_GPS_ANGLE                   0x40    // 64   If Modifier = 0 Value holds heading up to 180, if Modifier = 1 Value holds heading - 180 (so add 180)
#define CMD_GPS_HEADING                 0x41    // 65   Value contains a number from 0-15 representing one of the 16 cardinal directions
#define CMD_GPS_FIX                     0x42    // 66   GPS Fix status update. Value is boolean for fix, Modifier is 0 = no fix, 1 = GPS, 2 = DGPS (very accurate)
#define CMD_GPS_SATELLITES              0x43    // 67   Number of satellites in value
#define CMD_YEAR                        0x44    // 68   Value holds year after 2000, Modifier holds timezone
#define CMD_MONTH_DAY                   0x45    // 69   Value holds month (1-12), Modifier holds day (0-31)
#define CMD_HOUR_MINUTE                 0x46    // 70   Value holds hour (0-23), Modifier holds minute (0-59)
#define CMD_ACTION_TAKEN                0x47    // 71   Return letter to the display when it asks us to do something, letting it know we actually did. 
#define CMD_PRESSURE_ALTITUDE_POS       0x48    // 72   Altitude in feet, positive numbers. Value = alt divided by 100, modifier = remainder of feet less than 100 (add to val*100)
#define CMD_PRESSURE_ALTITUDE_NEG       0x49    // 73   Altitude in feet, negative numbers, in case I go to Death Valley.
#define CMD_PRESSURE_MERCURY            0x4A    // 74   Barometric pressure in inches of mercury (in hg). Value = integer, Modifier = 2 digit. Convert to kPa on the other end if you need to


#endif // COMMS_H



