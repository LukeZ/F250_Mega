
#include "COMMS.h"
#include "src/F250_Settings/F250_Settings.h"
#include "src/F250_EEPROM/F250_EEPROM.h"
#include "src/EEPROMex/EEPROMex.h"  
#include "src/F250_SimpleTimer/F250_SimpleTimer.h"
#include "src/Sleepy/Sleepy.h"
#include "src/Adafruit-BMP085/Adafruit_BMP085.h"
#include <OneWire.h>
#include "src/Adafruit_GPS_Mod/Adafruit_GPS_Mega_Hardware.h"        // Modified version of Adafruit GPS library to only use hardware serial, avoids conflicts with GSM library. Created from Adafruit version downloaded 6/30/2017
#include <GSM.h>

// GLOBAL VARIABLES
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------>>
    // USEFUL NAMES
        const byte On                           = 1;
        const byte Off                          = 0;

    // SERIAL   (GPS Serial defined below in GPS section)
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        HardwareSerial                          *DebugSerial;       // Which serial port to print debug messages to (HardwareSerial is equal to Serial0/Serial Port 0/USART0)
        boolean DEBUG                           = true;             // Print debugging messages to the PC
        
        #define DisplaySerial                   Serial3             // The hardware serial port assigned to the Teensy display computer
        #define SENTENCE_BYTES                  5                   // How many bytes in a valid sentence. 
        struct DataSentence {                                       // Serial commands should have four bytes (plus termination). 
            uint8_t    Address                  = 0;                // We use a struct for convenience
            uint8_t    Command                  = 0;
            uint8_t    Value                    = 0;
            uint8_t    Modifier                 = 0;
            uint8_t    Checksum                 = 0;
        };
        
    // PROJECT SPECIFIC EEPROM
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        F250_EEPROM eeprom;                                         // Wrapper class for dealing with eeprom. Note that EEPROM is also a valid object, it is the name of the EEPROMex class instance. Use the correct one!
                                                                    // F250_EEPROM basically provides some further functionality beyond EEPROMex. 
    // SIMPLE TIMER 
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        F250_SimpleTimer timer;                                     // SimpleTimer named "timer"
        boolean TimeUp                          = true;

    // OPTICALLY BUFFERED DIGITAL INPUTS - BY THE WAY, THESE ALL COME IN INVERTED FROM REALITY 
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>    
        // Use this for humans to work around the inverted nature of these signals
        #define DI_High                         0
        #define DI_Low                          1
    
        // Input board 1 - positive input signals (+12V)
        #define BaumannSpeedo                   A8                  // Although these pins belong to an analog port, we are setting them to digital input    
        #define TC_Locked                       A9                  // Torque converter is "forced" into lock, which will now occur automatically in 3rd and 4th gear. Good for highway or long descents where you don't want to heat your brakes
        #define TC_Unlocked                     A10                 // Torque converter is "forced" unlock, good for driving around town. Why do we have two of these? Because if neither is active, then the torque converter is in "Auto" mode.
        #define Overdrive_Off                   A11
        #define Run_Accy                        A12                 // Run & Accessory - active if engine is either running, or key is in accessory position
        #define Run_Start                       A13                 // Run & Start - active if engine is being _started_, or key is in accessory position
        #define CB_Select                       A14
        #define Viper_Siren                     A15
        // Status 1 tracking variables
        boolean Ham_On                          = false;        
        boolean OverdriveEnabled                = false;
        boolean AlarmTriggered                  = false;
        typedef char _TQC_STATE;                                     // Torque converted locked status
        #define TQC_AUTO                        0
        #define TQC_FORCE_LOCK                  1
        #define TQC_FORCE_UNLOCK                2
        #define TQC_UNKNOWN                     3                    // Not really used other than to force a message on startup
        _TQC_STATE TQCLockState                 = TQC_UNKNOWN;
        
        // Input board 2 - positive input signals
        #define FuelPump                        22
        // the other 7 are for now unusued inputs
        #define DI_22                           23
        #define DI_23                           24
        #define DI_24                           25
        #define DI_25                           26
        #define DI_26                           27
        #define DI_27                           28
        #define DI_28                           29        
        // Status 2 tracking variables
        boolean FuelPump_On                     = false;
    
        // Input board 3 - negative input signals
        #define SS1                             38                  // Baumann speed sensor           
        #define BaumannTable2                   39                  // Alternate transmission configuration
        #define Viper_Armed                     40                  // Is alarm system active (not triggered, but waiting)
        #define Viper_Ch2                        2                  // Aux button from alarm keyfob, we will massage the signal. Moved from D41 to D2 so I can take advantage of interrupt on change pin. 
        #define Low_Air                         42                  // Low air alarm
        #define DI_36                           43
        #define DI_37                           44
        #define DI_38                           45
        // Status 3 tracking variables
        boolean AlternateTransSetting           = false;
        boolean AlarmArmed                      = false;
        boolean Low_Air_Warning                 = false;

    // ANALOG INPUTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define AttoVoltage                     A0    
        // The rest are presently unused
        #define AI_2                            A1
        #define AI_3                            A2
        #define AI_4                            A3
        #define AI_5                            A4
        #define AI_6                            A5
        #define AI_7                            A6
        #define AI_8                            A7       

    // DIGITAL OUTPUTS (5V RELAY BOARD - SIGNAL HELD LOW/GROUND WILL TURN ON THE RELAY)
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define K1_Whelen                       13                  // Horn
        #define K2_AuxRearLights                11                  // Rear lights
        #define K3_                             9                   // N/A for now
        #define K4_                             8                   // N/A for now

    // AUX REAR LIGHTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>        
        boolean UpdateLightsFlag                = false;            // This gets set by an ISR, then the loop handles it using Polls();
        uint32_t TimeAuxRearLightsChanged       = 0;                // Save the time when we change the state of the aux rear lights. We will use it to debounce the input
        #define AuxRearLightsDebounceTime       500                 // After a change, ignore any other signals for this length of time
     
    // GPS BOARD SPECIFIC
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // Most GPS code taken from the "parsing" example in the Adafruit Ultimate GPS code base. 
        // "This code shows how to listen to the GPS module in an interrupt which allows the program to have more 'freedom' - 
        // just parse when a new NMEA sentence is available! Then access data when desired."
        #define GPS_EN                          7
        #define GPS_FIX                         6
        #define GPS_PPS                         5

        // My GPS is connected to Hardware Serial 1
        #define GPS_Serial                      Serial1             // For whatever reason, if you set this to an actual variable HardwareSerial, it will not work, so keep it at a DEFINE
        Adafruit_GPS GPS(&GPS_Serial);                              

        boolean GPS_FAST_UPDATE                 = true;             // If set to true will try to have the GPS update at 5Hz instead of 1Hz. Technically it works but when everything else is active I may find it interferes too much. 
        #define GPS_CHECK_uS                    300                 // The Timer 1 Compare A ISR is used to check for incoming GPS serial data. Adafruit had an ISR set to trip every 1mS, this was too slow for 5Hz updates. 
                                                                    // This define sets how frequently your ISR should trip, in uS (1 mS = 1000 uS). Even 500 uS (1/2 mS, twice as fast as Arduino) seems to work, but to be safe we
                                                                    // go a bit more frequent. Thankfully we were also able to up the baud rate to 115200 so it doesn't take as long to read incoming data, and if nothing is there, 
                                                                    // then the interrupt doesn't really take any time. 
        #define TIMER1_TICKS_PER_uS             2                   // Timer 1 is set to tick twice per uS, so we have excessively fine control. 
        
        boolean GPSECHO                         = false;            // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console. Set to 'true' if you want to debug and listen to the raw GPS sentences. 
                                                                    // We want it off except for debugging if necessary. 
        boolean GPS_Interrupt_Active            = false;            // This keeps track of whether the every 1mS interrupt to check for received GPS data is active or not (I don't think we actually need this variable anywhere)
        void Enable_GPS_Interrupt(boolean);                         // Func prototype keeps Arduino 0023 happy
        boolean GPS_FirstFix                    = false;            // We can check a few things when a fix is first obtained, like antenna status, that we won't need to check later. 
        boolean GPS_Fixed = false;                                  // An internal tracking variable, separate from the GPS library variable, will also let us track changes in the fix status
        uint8_t GPS_Fix_Quality                 = 0;                // So we can track changes
        uint8_t GPS_Antenna_Status              = ANTENNA_STAT_UNKNOWN; // Antenna status
        boolean GPS_Antenna_Status_Known        = false;            // Did we figure out the antenna status yet

        // Home stuff
        #define MAX_DISTANCE_COUNTS_AS_HOME     16000               // Distance from stored home position within which we will use the stored home altitude as a barometric adjustment, in other words, how far away from 
                                                                    // home do we still count "as home." In meters. 16,000 meters ~ 10 miles
                                                                    // Beyond this distance you can set a manual adjustment, or else we may decide to use GPS to adjust
        boolean startAtHome                     = false;            // Are we starting at home? 

        // Time Stuff
        // PST, MST, CST, etc... are defined in Settings.h. The current timezone among those is stored in EEPROM
        int UTC_Offset[5] = {-9, -8, -7, -6, -5};                   // CST adjustment from UTC, during standard time. In DST (summertime), the difference is one less (so the CST offset would become -5)
        int DSTbegin[] = { 310, 309, 308, 313, 312, 311, 310, 308, 314, 313, 312, 310, 309 };               // DST 2013 - 2025 in Canada and US
        int DSTend[] = { 1103, 1102, 1101, 1106, 1105, 1104, 1103, 1101, 1107, 1106, 1105, 1103, 1102 };    // DST 2013 - 2025 in Canada and US
        int DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };                             // Number of days in month, non-leap-year years
        _datetime CurrentDateTime;                                                                          // What is the current time

        // GPS Speed
        uint8_t MPH;                                                // Miles per hour, integer, can't exceed 255. Shouldn't be a problem.
        #define Minimum_MPH                     4                   // What minimum speed below which we ignore as noise from the GPS
        uint8_t Max_MPH;                                            // Maximum speed obtained since the car has been turned on
        const int GPS_SPEED_NTAPS               = 5;                // How many readings to average over
        float GPSSpeedFIR[GPS_SPEED_NTAPS];                         // Filter line for GPS speed readings (in knots)
        float GPS_Avg_Speed_Knots               = 0;                // Current average GPS speed (in knots - float)

        // GPS Bearing
        const int GPS_BEARING_NTAPS             = 10;               // How many readings to average over
        float GPSBearingFIR[GPS_BEARING_NTAPS];                     // Filter line for GPS bearing readings 
        float GPS_Avg_Bearing                   = 0;                // Current average bearing

        // GPS Coordinates
        uint8_t HeadingCode;                                        // 16 possible values (0-15) representing the points in a clockwise direction from North (N, NNE, NE, ENE, E, ESE, SE, SSE, S, etc... 
        float Current_Latitude;                                     // Present (or last known) location in degrees
        float Current_Longitude;                                    // Present (or last known) location in degrees

        // GPS Altitude
        float GPS_Altitude_Meters;                                  // Current altitude in meters
        int16_t GPS_Altitude_Feet;                                  // Current GPS altitude rounded to nearest foot

        // GPS -> Display
        #define GPS_SEND_FREQ                   500                 // How often to send GPS data to the screen in mS. GPS updates at 5hz, we send to the display at 2hz (every 1/2 second)
        int TimerID_GPSSender                   = 0;                // Timer that sends the readings to the display on some schedule
        

    // DALLAS ONE-WIRE (TEMPERATURE SENSORS)
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define ONE_WIRE_BUS                    30                  // One-Wire signal pin
        OneWire oneWire(ONE_WIRE_BUS);                              // We have two libraries going on here. One for OneWire generally:
        // DallasTemperature tempSensors(&oneWire);                 // The other is the Dallas Temperature Control library. Pass our oneWire reference to Dallas Temperature.
                                                                    // EDIT: We decided we could handle temp readings ourselves without their library
                                                    
        const int TEMP_NTAPS                    = 10;               // Number of elements in the FIR filter for temp readings. Temp is read approximately every (1 sec * number of sensors) seconds (2-3 sec)

        typedef char _TEMP_SENSOR;                                  // Convenient names for our sensors
        #define TS_INTERNAL                     0
        #define TS_EXTERNAL                     1
        #define TS_AUX                          2
        #define NUM_TEMP_SENSORS                3

        // Temperature sensor data for each individual sensor
        struct _tempsensor{                             
            byte address[8];                                        // One-Wire address
            boolean present;                                        // Sensor found
            byte type_s;                                            // Sensor type 1 = DS18S20 or old DS1820, 0 = DS18B20 or DS1822
            float temperature;                                      // Current temperature in Fahrenheit
            int16_t constrained_temp;                               // Integer value between -255 and 255
            int16_t constrained_temp_prior;                         // Last reading since change
            int16_t minSessionTemp;                                 // Min temp of this session
            int16_t maxSessionTemp;                                 // Max temp of this session
            boolean readingStarted;
            boolean newData;
            boolean sensorLost;                                     // Have we lost the sensor
            float fir[TEMP_NTAPS];                                  // Filter line for temp readings
            uint32_t lastMeasure;                                   // When did we last record a valid measurement
            _TEMP_SENSOR sensorName;                                // These will get assigned in InitTempStructs()
        };

        _tempsensor InternalTemp;                                   // Create three global sensor structs
        _tempsensor ExternalTemp;
        _tempsensor AuxTemp; 

        #define TEMP_SEND_FREQ                  4000                // How often to send temperature data to the screen in mS
        #define TEMP_SENSOR_CHECK_FREQ          10000L              // How often to check for presence or absence of sensors

        int TimerID_TempSensorCheck             = 0;                // Timer that polls the bus to detect attached temperature sensors
        int TimerID_TempSender                  = 0;                // Timer that sends the readings to the display on some schedule
        boolean TempHandlerEnabled = false;                         // The handler cycles through the various detected sensors and reads them over and over (when enabled)
 

    // VOLTAGE SENSOR
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // Take a measurement once per second, average over the last 10 measurements (10 seconds). Send update to screen every 3 seconds. 
        #define VOLTAGE_CHECK_FREQ              1000                // How often to take voltage measurement
        #define VOLTAGE_SEND_FREQ               3000                // How often to send voltage measurement to the screen in mS        
        int TimerID_VoltageSensorCheck          = 0;                // Timer that updates the voltage reading and averages it
        int TimerID_VoltageSender               = 0;                // Timer that sends the readings to the display on some schedule
        const int VOLTAGE_NTAPS                 = 10;               // Number of elements in the FIR filter for voltage readings.
        float BattVoltageFIR[VOLTAGE_NTAPS];                        // Filter line for voltage readings
        float BattVoltage                       = 0;                // Averaged battery voltage
        float BattVoltage_Prior                 = 0;    
        
    
    // GSM BOARD
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define GSM_TxOut                       10                  // Meaning tx-out from GSM, rx-in to Arduino
        #define GSM_RxIn                        3                   // Meaning rx-in  to GSM, tx-out from Arduino
        #define GSM_RST                         4

        #define PINNUMBER                       ""                  // This is not the physical pin, this is the PIN account number for your SIM card

        GSM gsm;                                                    // include a 'true' parameter for debug enabled        


    // BMP180 (BMP085 compatible) BAROMETRIC PRESSURE SENSOR & BAROMETRIC ALTITUDE 
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        Adafruit_BMP085 bmp;                                        // BMP180 sensor

        #define Troposphere                     0                   // We use this to specify which altitude formula to use
        #define Tropopause                      1                   // 
        unsigned char Region = Troposphere;                         // Atmospheric region we're currently in - can also be Tropopause (defined below)
        float p1_Tsphere = 101.325;                                 // Pressure at sea level in kPa, adjusted with CorrectP1 routine. Initialized here to standard-day value
        float p1_Tpause = 22.631;                                   // Pressure at beginning of tropopause, adjusted with CorrectP1 routine. Initialized here to standard-day value
        const float T1 = 288.15;                                    // Temperature at base of troposphere (Kelvin)
        const float a = -0.0065;                                    // Slope of temperature gradient in 0-11 km region (Troposphere) - Degrees Kelvin / meter
        const float g = 9.80665;                                    // Acceleration due to gravity meters / second squared
        const float R = 287.05;                                     // Ideal gas constant Joules / kilogram-kelvins

        #define PRESSURE_ALT_CHECK_FREQ         500                 // How often to take pressure measurement. We do it rather often since we have a long filter line
        #define PRESSURE_ALT_SEND_FREQ          2000                // How often to send pressure altitude to the screen in mS        
        int TimerID_PressureAltitudeCheck       = 0;                // Timer that updates the average pressure sensor reading and converts to altitude
        int TimerID_PressureAltitudeSender      = 0;                // Timer that sends the readings to the display on some schedule
        const int PA_NTAPS                      = 20;               // How many readings to average pressure over
        float PA_line[PA_NTAPS];                                    // Filter line for Altitude
        float Pressure                          = 101.325;          // Current static preasure measurement, in kPa. Initializedto standard-day sea level value. 

        float Pressure_Altitude_Meters;                             // Current pressure altitude in meters
        int16_t Pressure_Altitude_Feet;                             // Current pressure altitude in feet


    // APPLICATION STATE MACHINE
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        typedef char APP_STATE;                                
        #define APPLICATION_BOOT                0
        #define VEHICLE_OFF                     1
        #define VEHICLE_TRANSITION_ON           2
        #define VEHICLE_ON                      3
        #define VEHICLE_TRANSITION_OFF          4

        #define TransitionOnTime                3000                // How many milliseconds to wait from the time the car is turned on  until we actually go full on
        #define TransitionOffTime               5000                // How many milliseconds to wait from the time the car is turned off until we actually go full off


    // COMMUNICATION SETTINGS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define DISPLAY_UPDATE_FREQ             10000               // Display will be updated every this many milliseconds whether anything changed or not
        int TimerID_DisplayUpdate               = 0;

void setup() 
{
  
    // Set the input/output states of our pins
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        SetMyPins();

    // Interrupts
    // -------------------------------------------------------------------------------------------------------------------------------------------------->    
        attachInterrupt(digitalPinToInterrupt(2), ToggleRearLights, FALLING);    // Set an interrupt to occur whenever Viper_Ch2 signal goes low (Aux button pressed on the keyfob)
    
    // Turn off all relays. We do this as soon as possible at the beginning of the sketch
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        TurnOffRelays();

    // Init Serial & Comms
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        Serial.begin(USB_BAUD_RATE); 
        DebugSerial = &Serial;                                      // Use Serial 0 for debugging messages
        DisplaySerial.begin(DISPLAY_BAUD_RATE);
        delay(100);
        TurnOffDisplay();                                           // Start with display off

    // Load EEPROM
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        boolean did_we_init = eeprom.begin();                       // begin() will initialize EEPROM if it never has been before, and load all EEPROM settings into our ramcopy struct
        if (did_we_init && DEBUG) { DebugSerial->println(F("EEPROM Initalized")); }    
        CurrentDateTime.timezone = eeprom.ramcopy.Timezone;         // It's convenient to have Timezone in EEPROM but from here on out we'd like to refer to it using our date struct

    // Sensors
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
        InitTempStructs();                                          // Start by initializing our RAM structs
        SetupVoltageSensor();                                       // Initialize the voltage sensor
        TurnOffGPS();                                               // Keep the GPS off an startup until we know if the car is on or not. 
        InitAltimeter();
}

void loop(void) 
{
    static uint32_t TransitionOnStartTime = 0;
    static uint32_t TransitionOffStartTime = 0;
    static APP_STATE ApplicationState = APPLICATION_BOOT;

    // Regardless of state, we always update the timer and other timed routines that need polling
    timer.run();    // Simple Timer stuff
    Polls();        // Non-timer stuff that needs checking

    switch (ApplicationState)
    {
        case APPLICATION_BOOT:
            // This state occurs when the program first loads. We don't yet know the status of the vehicle.
            if (IsCarOn()) 
            {
                // Transition directly to on
                TransitionOnStartTime = millis() + TransitionOnTime + 1;    // This will eliminate the on transition delay
                ApplicationState = VEHICLE_TRANSITION_ON;
            }
            else
            {
                TurnOffDisplay();                                           // Already sent this in Setup(), but do it again just in case
                ApplicationState = VEHICLE_OFF;                             // Don't transition, go straight to Off
            }
            break; 

            
        case VEHICLE_OFF:
            // In Car Off mode we want to do the barest minimum of stuff. 
            
            // Sleep for some time
            Sleepy::loseSomeTime(1000); 
            // Alternatively, powerdown (will still wake again with interrupt)
                /// flushes pending data in Serial and then enter low-power mode, wake up
                /// with watchdog, INT0/1, or pin-change
            // flushAndPowerDown();

            // Has the car turned on yet? 
            if (IsCarOn()) 
            {
                TransitionOnStartTime = millis();
                ApplicationState = VEHICLE_TRANSITION_ON;               // If the car has been turned on go to transition on state
            }
            break;


        case VEHICLE_TRANSITION_ON:
            // Before we do anything, we want to see if we are going to stay on or if this was just a glitch (ie during starting the engine)
            // So we wait, and if the car is still on after some length of time, we'll rouse ourselves to the job at hand. 

            // If the car was turned back off while we're waiting, revert back to the off state
            if (IsCarOn() == false)
            {
                ApplicationState = VEHICLE_OFF;
            }
            else if (millis() - TransitionOnStartTime > TransitionOnTime)
            {   
                if (DEBUG) DebugSerial->println(F("Vehicle turned on"));// Ok, we've waited long enough, go ahead and turn everything on
                ApplicationState = VEHICLE_ON;                          // Transition to full time ON state
                
                // Get things rolling
                TurnOnDisplay();                                        // Turn on the display 
                InitSessionMinMaxes();                                  // Clear session min/maxes
                TurnOnGPS();                                            // Get the GPS going, this one takes a while, do it before the others
                StartTempReadings();                                    // Start reading temperature data and sending it to the display
                StartVoltageReadings();                                 // Start reading battery voltage
                StartPressureReadings();                                // Start reading the barometric pressure sensor
                // This will start a repeating timer that will send status updates to the display whether anything has changed or not
                // Below in the ON state we will also continuously check a Poll routine that will additionally send pertinent updates
                // the moment any changes are detected. 
                if (TimerID_DisplayUpdate == 0) timer.setInterval(DISPLAY_UPDATE_FREQ, UpdateDisplay); 
                else if (timer.isEnabled(TimerID_DisplayUpdate) == false) timer.enable(TimerID_DisplayUpdate);
            }
            break;

        case VEHICLE_ON:
            DoVehicleOnStuff();                                         // Everything that needs to be done while vehicle is on, that isn't already scheduled
          
            // Has the car been turned off? 
            if (IsCarOn() == false) 
            {
                TransitionOffStartTime = millis();
                ApplicationState = VEHICLE_TRANSITION_OFF;  // If the car has been turned off go to transition off state
            }
            break;


        case VEHICLE_TRANSITION_OFF:
            // Before we do anything, we want to see if we are going to stay off or if this was just a glitch (glitch maybe, or pehaps engine turned off but then user wento accessory)
            // So we wait, and if the car is still off after some length of time then we'll consider it official

            // While we wait, keep up appearances
            DoVehicleOnStuff();                                         // Everything that needs to be done while vehicle is on, that isn't already scheduled

            // If the car was turned back on while we're waiting, revert back to the on state
            if (IsCarOn())
            {
                ApplicationState = VEHICLE_ON;
            }            
            else if (millis() - TransitionOffStartTime > TransitionOffTime)
            {   
                if (DEBUG) DebugSerial->println(F("Vehicle turned off"));// Ok, we've waited long enough, go ahead and turn everything off
                ApplicationState = VEHICLE_OFF;                         // Go to full time Off state

                // Shut the shit down
                TurnOffDisplay();                                       // Tell the display to turn off
                PauseTempReadings();                                    // Stop taking temperature readings or sending them to the display
                PauseVoltageReadings();                                 // Stop taking battery voltage readings
                PausePressureReadings();                                // Stop taking barometric pressure readings
                TurnOffGPS();                                           // Shutdown the GPS
                TurnOffCellPhone();                                     // And the cell phone
                if (TimerID_DisplayUpdate > 0) timer.disable(TimerID_DisplayUpdate);    // Pause sending display updates
            }
            break;
    }
}

// We make all this stuff its own routine so it is easy to call it from both VEHICLE_ON and temporarily also from VEHICLE_TRANSITION_OFF
void DoVehicleOnStuff(void)
{   
    CheckSerial();                                              // See if the display is communicating with us
    PollInputs();                                               // Handle input polling of things we want the display to know about that changed. This is not important when the car is off. 
    // If we set the GPS to fast updates the 1 mS interrupt isn't fast enough to keep up, so we have to read here in the main loop
    if (!GPS_Interrupt_Active) { while (GPS_Serial.available()) {GPS.read();} }
    CheckGPS_ForData();                                         // This checks for a complete and valid sentence; if one is found it updates local variables. Data gets sent to the display independently. 
}

void InitSessionMinMaxes(void)
{
    Max_MPH = 0;                            // What is the fastest we've gone since the car was turned on
    InternalTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    InternalTemp.maxSessionTemp = -50;            
    ExternalTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    ExternalTemp.maxSessionTemp = -50;            
    AuxTemp.minSessionTemp = 200;      // Initialize min/max to values that will be overwritten quickly
    AuxTemp.maxSessionTemp = -50;                    
}

void PollInputs()
{
    // Here we see if any inputs have changed, and if so, send a message to the display. 
    
    // Ham/CB microphone selector
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    if (digitalRead(CB_Select) == DI_High && Ham_On == true)
    {
        Ham_On = false;
        SendDisplay(CMD_HAM_ON, false);
        if (DEBUG) DebugSerial->println(F("CB microphone selected"));
    }
    if (digitalRead(CB_Select) == DI_Low && Ham_On == false)
    {
        Ham_On = true;
        SendDisplay(CMD_HAM_ON, true);
        if (DEBUG) DebugSerial->println(F("Ham microphone selected"));
    }

    // Fuel pump
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    if (digitalRead(FuelPump) == DI_High && FuelPump_On == false)  
    {
        FuelPump_On = true;
        SendDisplay(CMD_FUEL_PUMP, true);
        if (DEBUG) DebugSerial->println(F("Fuel Pump turned on!!")); 
    }
    if (digitalRead(FuelPump) == DI_Low && FuelPump_On == true)
    {
        FuelPump_On = false;
        SendDisplay(CMD_FUEL_PUMP, false);
        if (DEBUG) DebugSerial->println(F("Fuel Pump turned off")); 
    }

    // Torque converter lockup state
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    _TQC_STATE tqc = GetTorqueConverterState();
    if (tqc != TQCLockState)
    {
        TQCLockState = tqc;
        if (DEBUG) DebugSerial->print(F("Torque converter locked mode: "));
        switch (TQCLockState)
        {
            case TQC_AUTO:          SendDisplay(CMD_TQC_LOCK_STATUS, TQCLockState); if (DEBUG) { DebugSerial->println(F("Auto"));          } break;
            case TQC_FORCE_LOCK:    SendDisplay(CMD_TQC_LOCK_STATUS, TQCLockState); if (DEBUG) { DebugSerial->println(F("Force Locked"));  } break;
            case TQC_FORCE_UNLOCK:  SendDisplay(CMD_TQC_LOCK_STATUS, TQCLockState); if (DEBUG) { DebugSerial->println(F("Force Unlocked"));} break;
        }
    }

    // Overdrive enabled
    // -------------------------------------------------------------------------------------------------------------------------------------------------->                
    if (digitalRead(Overdrive_Off) == DI_High && OverdriveEnabled == true)  
    {
        OverdriveEnabled = false;
        SendDisplay(CMD_OVERDRIVE, false);
        if (DEBUG) DebugSerial->println(F("Overdrive disabled")); 
    }
    if (digitalRead(Overdrive_Off) == DI_Low && OverdriveEnabled == false)
    {
        OverdriveEnabled = true;
        SendDisplay(CMD_OVERDRIVE, true);
        if (DEBUG) DebugSerial->println(F("Overdrive enabled")); 
    }    

    // Baumann Table 2 (alternate transmission configuration settings - negative signal means Table 2 active)
    // -------------------------------------------------------------------------------------------------------------------------------------------------->     
    if (digitalRead(BaumannTable2) == DI_Low && AlternateTransSetting == false)  
    {
        AlternateTransSetting = true;
        SendDisplay(CMD_TRANS_TABLE, 2);
        if (DEBUG) DebugSerial->println(F("Baumann Table 2 selected")); 
    }
    if (digitalRead(BaumannTable2) == DI_High && AlternateTransSetting == true)
    {
        AlternateTransSetting = false;
        SendDisplay(CMD_TRANS_TABLE, 1);
        if (DEBUG) DebugSerial->println(F("Baumann Default (Table 1) settings selected")); 
    }   
    
    // Low air tank warning (negative signal is the warning)
    // -------------------------------------------------------------------------------------------------------------------------------------------------->     
    if (digitalRead(Low_Air) == DI_Low && Low_Air_Warning == false)  
    {
        Low_Air_Warning = true;
        SendDisplay(CMD_LOW_AIR_WARN, true);
        if (DEBUG) DebugSerial->println(F("Low air warning!")); 
    }
    if (digitalRead(Low_Air) == DI_High && Low_Air_Warning == true)
    {
        Low_Air_Warning = false;
        SendDisplay(CMD_LOW_AIR_WARN, false);
        if (DEBUG) DebugSerial->println(F("Air pressure restored")); 
    }   

    // Alarm status - this probably doesn't need to be sent to the display, but rather handled internally
    // -------------------------------------------------------------------------------------------------------------------------------------------------->     
    // boolean AlarmArmed = false;          // Is alarm armed
    // boolean AlarmTriggered = false;      // Has the alarm been triggered; literally, the siren is going off


}

void SetMyPins() 
{

    // OPTICALLY BUFFERED DIGITAL INPUTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // Input board 1 - positive input signals (+12V)
        // Although these pins belong to an analog port, we are setting them to digital input    
        pinMode(BaumannSpeedo, INPUT_PULLUP);
        pinMode(TC_Locked, INPUT_PULLUP);
        pinMode(TC_Unlocked, INPUT_PULLUP);
        pinMode(Overdrive_Off, INPUT_PULLUP);
        pinMode(Run_Accy, INPUT_PULLUP);
        pinMode(Run_Start, INPUT_PULLUP);
        pinMode(CB_Select, INPUT_PULLUP);
        pinMode(Viper_Siren, INPUT_PULLUP);
        
        // Input board 2 - positive input signals
        pinMode(FuelPump, INPUT_PULLUP);
        // the other 7 are for now unusued inputs
        pinMode(DI_22, INPUT_PULLUP);
        pinMode(DI_23, INPUT_PULLUP);
        pinMode(DI_24, INPUT_PULLUP);
        pinMode(DI_25, INPUT_PULLUP);
        pinMode(DI_26, INPUT_PULLUP);
        pinMode(DI_27, INPUT_PULLUP);
        pinMode(DI_28, INPUT_PULLUP);

        // Input board 3 - negative input signals
        pinMode(SS1, INPUT_PULLUP);
        pinMode(BaumannTable2, INPUT_PULLUP);
        pinMode(Viper_Armed, INPUT_PULLUP);
        pinMode(Viper_Ch2, INPUT_PULLUP);
        pinMode(Low_Air, INPUT_PULLUP);
        pinMode(DI_36, INPUT_PULLUP);
        pinMode(DI_37, INPUT_PULLUP);
        pinMode(DI_38, INPUT_PULLUP);


    // ANALOG INPUTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        pinMode(AttoVoltage, INPUT);
        // The rest are presently unused
        pinMode(AI_2, INPUT);
        pinMode(AI_3, INPUT);
        pinMode(AI_4, INPUT);
        pinMode(AI_5, INPUT);
        pinMode(AI_6, INPUT);
        pinMode(AI_7, INPUT);
        pinMode(AI_8, INPUT);


    // I2C - Used for GPS and maybe something else I don't remember
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>    
        // pinMode(I2C_SDA, ?);
        // pinMode(I2C_SCL, ?)


    // GPS BOARD SPECIFIC
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        //http://learn.adafruit.com/adafruit-ultimate-gps/advanced-wiring
        pinMode(GPS_EN, OUTPUT); 
        pinMode(GPS_FIX, INPUT);     
        pinMode(GPS_PPS, INPUT);
        

    // DALLAS ONE-WIRE
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // pinMode(ONE_WIRE_BUS, ?);    // Set by OneWire library


    // 5V RELAY BOARD (DIGITAL OUTPUTS) - signal low closes the relay
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        pinMode(K1_Whelen, OUTPUT);
        pinMode(K2_AuxRearLights, OUTPUT);
        pinMode(K3_, OUTPUT);
        pinMode(K4_, OUTPUT);
        

    // GSM BOARD
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        pinMode(GSM_TxOut, INPUT);    // Meaning tx-out from GSM, rx-in to Arduino
        pinMode(GSM_RxIn, OUTPUT);    // Meaning rx-in  to GSM, tx-out from Arduino
        pinMode(GSM_RST, OUTPUT);
  
}



