
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GPS_Mega_Hardware.h>
#include <GSM.h>
#include <JeeLib.h>


#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif


ISR(WDT_vect) { Sleepy::watchdogEvent(); }


//=======================================================================================================================================================>>
// DEFINES 
//=======================================================================================================================================================>>
    // Useful names 
        const byte YES = 1;
        const byte NO = 0;
        const byte SET = 1;
        const byte CLEARED = 0;
        const byte On = 1;
        const byte Off = 0;

    // SPI - TFT DISPLAY
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>    
        // These are the hardware SPI pins for the Mega
//        #define _sclk                52
//        #define _miso                50
//        #define _mosi                51
        #define _cs                  53        // I think 49 is CS for the SD card, 53 (SS) select for the TFT 
        #define _dc                  47
        #define _rst                 48
        #define TFT_PWM              12
        int Backlight              = 127;       // Initialize display to 50% brightness (0-255 range)

// What it was incorrectly wired as:
//        #define _sclk                49
//        #define _miso                51
//        #define _mosi                50
//        #define _cs                  48        
//        #define _dc                  12
//        #define _rst                 53
//        #define TFT_PWM              47

    
        // Using software SPI is really not suggested, its incredibly slow
//        Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _mosi, _sclk, _rst, _miso);
        // Use hardware SPI
        Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);


    // OPTICALLY BUFFERED DIGITAL INPUTS - BY THE WAY, THESE ALL COME IN INVERTED FROM REALITY 
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // Use this for humans to work around the inverted nature of these signals
        #define DI_High              0
        #define DI_Low               1

        // Input board 1 - positive input signals (+12V)
        #define BaumannSpeedo        A8            // Although these pins belong to an analog port, we are setting them to digital input    
        #define TC_Locked            A9    
        #define TC_Unlocked          A10
        #define Overdrive_Off        A11
        #define Run_Accy             A12
        #define Run_Start            A13
        #define CB_Select            A14
        #define Viper_Siren          A15
        
        // Input board 2 - positive input signals
        #define FuelPump             22
        // the other 7 are for now unusued inputs
        #define DI_22                23
        #define DI_23                24
        #define DI_24                25
        #define DI_25                26
        #define DI_26                27
        #define DI_27                28
        #define DI_28                29        

        // Input board 3 - negative input signals
        #define SS1                  38           
        #define BaumannTable2        39
        #define Viper_Armed          40
        #define Viper_Ch2            41
        #define Low_Air              42
        #define DI_36                43
        #define DI_37                44
        #define DI_38                45


    // ANALOG INPUTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define AttoVoltage          A0    
        // The rest are presently unused
        #define AI_2                 A1
        #define AI_3                 A2
        #define AI_4                 A3
        #define AI_5                 A4
        #define AI_6                 A5
        #define AI_7                 A6
        #define AI_8                 A7       


    // I2C - Used for Pressure Sensor
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>    
        #define I2C_SDA              20
        #define I2C_SCL              21


    // GPS BOARD SPECIFIC
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define GPS_EN               7
        #define GPS_FIX              6
        #define GPS_PPS              5
        // Connect the GPS Power pin to 5V
        // Connect the GPS Ground pin to ground
        // If using hardware serial (e.g. Arduino Mega):
        //   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
        //   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
        // LUKE EDIT: My GPS is connected to Hardware Serial 1
        
        // If using hardware serial (e.g. Arduino Mega), comment
        // out the above six lines and enable this line instead:
        Adafruit_GPS GPS(&Serial1);

        // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
        // Set to 'true' if you want to debug and listen to the raw GPS sentences
        #define GPSECHO  false

        // this keeps track of whether we're using the interrupt
        // off by default!
        boolean usingInterrupt = false;
        void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy        
        

    // DALLAS ONE-WIRE
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define ONE_WIRE_BUS         30
        
        // We have two libraries going on here. One for OneWire generally:
        OneWire oneWire(ONE_WIRE_BUS);
        // The other is the Dallas Temperature Control library. Pass our oneWire reference to Dallas Temperature.
        DallasTemperature tempSensors(&oneWire);


    // 5V RELAY BOARD (DIGITAL OUTPUTS)
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define K1_Whelen            13
        #define K2_AuxRearLights     11
        #define K3_                  9
        #define K4_                  8
        

    // GSM BOARD
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        #define GSM_TxOut            10    // Meaning tx-out from GSM, rx-in to Arduino
        #define GSM_RxIn             3     // Meaning rx-in  to GSM, tx-out from Arduino
        #define GSM_RST              4

        #define PINNUMBER ""               // This is not the physical pin, this is the PIN account number for your SIM card

        GSM gsm;                           // include a 'true' parameter for debug enabled        





void setup() {
  
    // Set the input/output states of our pins
    SetMyPins();
    
    
    // Turn off all relays. We do this as soon as possible at the beginning of the sketch
    TurnOffRelays_Alt();

    // Dallas Temperature Control sensors initialize
    tempSensors.begin();
    
    // GPS initialize
    // 9600 NMEA is the default baud rate for MTK - some use 4800
//    GPS.begin(9600);    

    // Display initialize
    tft.begin();
    tft.fillScreen(ILI9340_BLACK);

    digitalWrite(TFT_PWM, HIGH);



  Serial.begin(9600);
  while (!Serial);
  
  Serial.println("Adafruit 2.2\" SPI TFT Test!"); 
 
  Serial.println(F("Benchmark                Time (microseconds)"));
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

 
  Serial.println(F("Done!"));
}

void loop(void) 
{

    static int CurrentState;        // Current car status


// This is a further TFT test    
//  for(uint8_t rotation=0; rotation<4; rotation++) {
//    tft.setRotation(rotation);
//    testText();
//    delay(2000);
//  }

    // Check states
    
    // Is the car on or off? 

    if (CarOn() == true)
    {
        if (CurrentState == Off)
        {
            // Then we have a change of state
            CurrentState = On;

            //Yes, the car is on. We turn on the display (to "Backlight" brightness)
            TurnOnDisplay();
            TurnOnGPS();

            Serial.println("Car On!"); 
        }
        else
        {
            //Yes, the car is on, but it already was on. Let's wait to see if they turn it off
            Sleepy::loseSomeTime(1000);    // Wait one second
        }

        ShowStatus();

    }
    else
    {
        if (CurrentState == On)
        {
            //The car is off, but previously it was on. Let's shut some things down
            CurrentState = Off;

            //No, the car is not on. Turn off the display
            TurnOffDisplay();    
            TurnOffGPS();
        
            //Let's turn off the cell shield. We'll leave it off for now. 
            gsm.shutdown();

            Serial.println("Car Off");        
        }


        // Sleep for 1 second
        Sleepy::loseSomeTime(1000);

    }


}


void ShowStatus(void)
{
  tft.fillScreen(ILI9340_RED);

  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_WHITE);  
  tft.setTextSize(1);
  
  if (digitalRead(Run_Accy) == DI_High && digitalRead(Run_Start) == DI_Low) 
  { tft.println("Accessory"); } 
  else if (digitalRead(Run_Accy) == DI_Low && digitalRead(Run_Start) == DI_High) 
  { tft.println("Car On"); }

  if (digitalRead(CB_Select) == DI_High) 
  { tft.println("CB ON"); }
  else
  { tft.println("HAM ON"); }

  tft.println();

  tft.setTextColor(ILI9340_YELLOW); 
  tft.setTextSize(2);
  if (digitalRead(FuelPump) == DI_High)
  { tft.println("Fuel Pump: ON!"); }
  else
  { tft.println("Fuel Pump: Off"); }

}



unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9340_BLACK);
  tft.fillScreen(ILI9340_RED);
  tft.fillScreen(ILI9340_GREEN);
  tft.fillScreen(ILI9340_BLUE);
  tft.fillScreen(ILI9340_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9340_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9340_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9340_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9340_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}



void SetMyPins() {


    // SPI - TFT DISPLAY
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>    
        // These are the hardware SPI pins for the Mega. We will let most of them be set by the Adafruit library
        // pinMode(_sclk, ?);
        // pinMode(_miso, ?);
        // pinMode(_mosi, ?);
        // pinMode(_cs, ?);
        // pinMode(_dc, ?);
        // pinMode(_rst, ?);
        pinMode(_cs, OUTPUT);
        pinMode(TFT_PWM, OUTPUT);

    // OPTICALLY BUFFERED DIGITAL INPUTS
    //--------------------------------------------------------------------------------------------------------------------------------------------------->>
        // Input board 1 - positive input signals (+12V)
        // Although these pins belong to an analog port, we are setting them to digital input    
        pinMode(BaumannSpeedo, INPUT);
        pinMode(TC_Locked, INPUT);
        pinMode(TC_Unlocked, INPUT);
        pinMode(Overdrive_Off, INPUT);
        pinMode(Run_Accy, INPUT);
        pinMode(Run_Start, INPUT);
        pinMode(CB_Select, INPUT);
        pinMode(Viper_Siren, INPUT);
        
        // Input board 2 - positive input signals
        pinMode(FuelPump, INPUT);
        // the other 7 are for now unusued inputs
        pinMode(DI_22, INPUT);
        pinMode(DI_23, INPUT);
        pinMode(DI_24, INPUT);
        pinMode(DI_25, INPUT);
        pinMode(DI_26, INPUT);
        pinMode(DI_27, INPUT);
        pinMode(DI_28, INPUT);

        // Input board 3 - negative input signals
        pinMode(SS1, INPUT);
        pinMode(BaumannTable2, INPUT);
        pinMode(Viper_Armed, INPUT);
        pinMode(Viper_Ch2, INPUT);
        pinMode(Low_Air, INPUT);
        pinMode(DI_36, INPUT);
        pinMode(DI_37, INPUT);
        pinMode(DI_38, INPUT);


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


    // 5V RELAY BOARD (DIGITAL OUTPUTS)
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
