
#include "COMMS.h"
#include "src/F250_Settings/F250_Settings.h"
#include "src/F250_EEPROM/F250_EEPROM.h"
#include "src/EEPROMex/EEPROMex.h"  
#include "src/F250_SimpleTimer/F250_SimpleTimer.h"
#include "src/Sleepy/Sleepy.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include <OneWire.h>
#include <Adafruit_GPS_Mega_Hardware.h>
#include <GSM.h>


// GLOBAL VARIABLES
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------>>
    // USEFUL NAMES
        const byte On                           = 1;
        const byte Off                          = 0;

    // SERIAL
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define DisplaySerial                   Serial3             // The hardware serial port assigned to the Teensy display computer
        #define GPS_Serial                      Serial1             // The hardware serial port assigned to communicate with the Adafruit GPS
        #define SENTENCE_BYTES                  5                   // How many bytes in a valid sentence. Note we use 5 bytes, not the 4 you are used to seeing with the Scout or Sabertooth! 
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
        _TQC_STATE TorqueConverterState         = TQC_UNKNOWN;
        
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
     
    // I2C - Used for Pressure Sensor
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define I2C_SDA                         20
        #define I2C_SCL                         21


    // GPS BOARD SPECIFIC
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define GPS_EN                          7
        #define GPS_FIX                         6
        #define GPS_PPS                         5
        // Connect the GPS Power pin to 5V
        // Connect the GPS Ground pin to ground
        // If using hardware serial (e.g. Arduino Mega):
        //   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
        //   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
        // LUKE EDIT: My GPS is connected to Hardware Serial 1
        
        // If using hardware serial (e.g. Arduino Mega), comment
        // out the above six lines and enable this line instead:
        Adafruit_GPS GPS(&GPS_Serial);

        // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
        // Set to 'true' if you want to debug and listen to the raw GPS sentences
        #define GPSECHO                         false

        // this keeps track of whether we're using the interrupt
        // off by default!
        boolean usingInterrupt                  = false;
        void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy        
        

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
        const int VOLTAGE_NTAPS                 = 10;               // Number of elements in the FIR filter for voltage readings. Temp is read approximately every (1 sec * number of sensors) seconds (2-3 sec)
        float BattVoltageFIR[VOLTAGE_NTAPS];                        // Filter line for voltage readings
        float BattVoltage                       = 0;                // Averaged battery voltage
        #define VOLTAGE_CHECK_FREQ              1000                // How often to take voltage measurement
        #define VOLTAGE_SEND_FREQ               4000                // How often to send voltage measurement to the screen in mS        
        int TimerID_VoltageSensorCheck          = 0;                // Timer that polls the bus to detect attached temperature sensors
        int TimerID_VoltageSender               = 0;                // Timer that sends the readings to the display on some schedule
        
    
    // GSM BOARD
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define GSM_TxOut                       10                  // Meaning tx-out from GSM, rx-in to Arduino
        #define GSM_RxIn                        3                   // Meaning rx-in  to GSM, tx-out from Arduino
        #define GSM_RST                         4

        #define PINNUMBER                       ""                  // This is not the physical pin, this is the PIN account number for your SIM card

        GSM gsm;                                                    // include a 'true' parameter for debug enabled        


    // APPLICATION STATE MACHINE
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        typedef char APP_STATE;                                
        #define VEHICLE_OFF                     0
        #define VEHICLE_TRANSITION_ON           1
        #define VEHICLE_ON                      2
        #define VEHICLE_TRANSITION_OFF          3

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

    // Load EEPROM
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        boolean did_we_init = eeprom.begin();                           // begin() will initialize EEPROM if it never has been before, and load all EEPROM settings into our ramcopy struct
        if (did_we_init) { Serial.println(F("EEPROM Initalized")); }    // We can use this for testing, but in general it's not needed, it doesn't happen often, and rarely would the user catch it. 
        
    // Init Serial & Comms
    // -------------------------------------------------------------------------------------------------------------------------------------------------->
        Serial.begin(USB_BAUD_RATE); 
        DisplaySerial.begin(DISPLAY_BAUD_RATE);

    // Sensors
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
        InitTempStructs();                                              // Start by initializing our RAM structs
        SetupVoltageSensor();                                           // Initialize the voltage sensor
        
}

void loop(void) 
{
    static uint32_t TransitionOnStartTime = 0;
    static uint32_t TransitionOffStartTime = 0;
    static APP_STATE ApplicationState = VEHICLE_OFF;

    // Regardless of state, we always update the timer and other timed routines that need polling
    timer.run();    // Simple Timer stuff
    Polls();        // Non-timer stuff that needs checking

    switch (ApplicationState)
    {
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
                Serial.println(F("Vehicle turned on"));                 // Ok, we've waited long enough, go ahead and turn everything on
                ApplicationState = VEHICLE_ON;                          // Transition to full time ON state
                
                // Get things rolling
                TurnOnDisplay();                                        // Turn on the display 
                StartTempReadings();                                    // Start reading temperature data and sending it to the display
                StartVoltageReadings();                                 // Start reading battery voltage
                TurnOnGPS();                                            // Get the GPS going
                // This will start a repeating timer that will send status updates to the display whether anything has changed or not
                // Below in the ON state we will also continuously check a Poll routine that will additionally send pertinent updates
                // the moment any changes are detected. 
                if (TimerID_DisplayUpdate == 0) timer.setInterval(DISPLAY_UPDATE_FREQ, UpdateDisplay); 
                else if (timer.isEnabled(TimerID_DisplayUpdate) == false) timer.enable(TimerID_DisplayUpdate);
            }
            break;

        case VEHICLE_ON:
            PollInputs();                                               // Handle input polling of things we want the display to know about that changed. This is not important when the car is off. 
            
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

            // If the car was turned back on while we're waiting, revert back to the on state
            if (IsCarOn())
            {
                ApplicationState = VEHICLE_ON;
            }            
            else if (millis() - TransitionOffStartTime > TransitionOffTime)
            {   
                Serial.println(F("Vehicle turned off"));                // Ok, we've waited long enough, go ahead and turn everything off
                ApplicationState = VEHICLE_OFF;                         // Go to full time Off state

                // Shut the shit down
                TurnOffDisplay();                                       // Tell the display to turn off
                PauseTempReadings();                                    // Stop taking temperature readings or sending them to the display
                PauseVoltageReadings();                                 // Stop taking battery voltage readings
                TurnOffGPS();                                           // Shutdown the GPS
                TurnOffCellPhone();                                     // And the cell phone
                if (TimerID_DisplayUpdate > 0) timer.disable(TimerID_DisplayUpdate);    // Pause sending display updates
            }
            break;
    }
}

void PollInputs()
{
    // Here we see if any inputs have changed, and if so, send a message to the display. 
    
    // Ham/CB microphone selector
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    if (digitalRead(CB_Select) == DI_High && Ham_On == true)
    {
        Ham_On = false;
        SendDisplay(CMD_CB_ON);
        Serial.println(F("CB microphone selected"));
    }
    if (digitalRead(CB_Select) == DI_Low && Ham_On == false)
    {
        Ham_On = true;
        SendDisplay(CMD_HAM_ON);
        Serial.println(F("Ham microphone selected"));
    }

    // Fuel pump
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    if (digitalRead(FuelPump) == DI_High && FuelPump_On == false)  
    {
        FuelPump_On = true;
        SendDisplay(CMD_FUEL_PUMP_ON);
        Serial.println(F("Fuel Pump turned on!!")); 
    }
    if (digitalRead(FuelPump) == DI_Low && FuelPump_On == true)
    {
        FuelPump_On = false;
        SendDisplay(CMD_FUEL_PUMP_OFF);
        Serial.println(F("Fuel Pump turned off")); 
    }

    // Torque converter lockup state
    // -------------------------------------------------------------------------------------------------------------------------------------------------->        
    _TQC_STATE tqc = GetTorqueConverterState();
    if (tqc != TorqueConverterState)
    {
        TorqueConverterState = tqc;
        Serial.print(F("Torque converter locked mode: "));
        switch (tqc)
        {
            case TQC_AUTO:          SendDisplay(CMD_TQC_AUTO);          Serial.println(F("Auto"));           break;
            case TQC_FORCE_LOCK:    SendDisplay(CMD_TQC_FORCE_LOCK);    Serial.println(F("Force Locked"));   break;
            case TQC_FORCE_UNLOCK:  SendDisplay(CMD_TQC_FORCE_UNLOCK);  Serial.println(F("Force Unlocked")); break;
        }
    }

    // Overdrive enabled
    // -------------------------------------------------------------------------------------------------------------------------------------------------->                
    if (digitalRead(Overdrive_Off) == DI_High && OverdriveEnabled == true)  
    {
        OverdriveEnabled = false;
        SendDisplay(CMD_OVERDRIVE_OFF);
        Serial.println(F("Overdrive disabled")); 
    }
    if (digitalRead(Overdrive_Off) == DI_Low && OverdriveEnabled == false)
    {
        OverdriveEnabled = true;
        SendDisplay(CMD_OVERDRIVE_ON);
        Serial.println(F("Overdrive enabled")); 
    }    

    // Baumann Table 2 (alternate transmission configuration settings - negative signal means Table 2 active)
    // -------------------------------------------------------------------------------------------------------------------------------------------------->     
    if (digitalRead(BaumannTable2) == DI_Low && AlternateTransSetting == false)  
    {
        AlternateTransSetting = true;
        SendDisplay(CMD_TRANS_TABLE2);
        Serial.println(F("Baumann Table 2 selected")); 
    }
    if (digitalRead(BaumannTable2) == DI_High && AlternateTransSetting == true)
    {
        AlternateTransSetting = false;
        SendDisplay(CMD_TRANS_TABLE1);
        Serial.println(F("Baumann Default (Table 1) settings selected")); 
    }   
    
    // Low air tank warning (negative signal is the warning)
    // -------------------------------------------------------------------------------------------------------------------------------------------------->     
    if (digitalRead(Low_Air) == DI_Low && Low_Air_Warning == false)  
    {
        Low_Air_Warning = true;
        SendDisplay(CMD_LOW_AIR_WARN);
        Serial.println(F("Low air warning!")); 
    }
    if (digitalRead(Low_Air) == DI_High && Low_Air_Warning == true)
    {
        Low_Air_Warning = false;
        SendDisplay(CMD_AIR_RESTORED);
        Serial.println(F("Air pressure restored")); 
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

/*

// This is a further TFT test    
//  for(uint8_t rotation=0; rotation<4; rotation++) {
//    tft.setRotation(rotation);
//    testText();
//    delay(2000);
//  }
*/

