// Voltage Sensor System
// Rev 1.0 (26/06/2021)
// - Maxtrax

#include <Adafruit_MCP3008.h>
#include <DTIOI2CtoParallelConverter.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Scheduler.h>

const char * app_ver = "v1.0";

const byte ROTARY_CLK = 3;  //Output A
const byte ROTARY_DT = 4;   //Output B
const byte ROTARY_SW = 2;   //Switch

const byte ADC3_CS = 5;
const byte ADC2_CS = 6;
const byte ADC1_CS = 7;
const byte MOSI_PIN = 8;
const byte SCK_PIN = 9;
const byte MISO_PIN = 10;
const byte BUZZER_PIN = PIN0_7;

const int DEBOUNCE_MS = 50;
const int CHECK_MS = 10;

const byte MAX_CHANNELS = 18;
const byte MAX_LED = MAX_CHANNELS*2;

const byte INVALID_VOLT = 0xFF;

enum _Volt_config
{
  CFG_7V,
  CFG_9V,
  CFG_12V,
  CFG_15V,
  CFG_25V,
  CFG_TEST,
  CFG_MAX
};

const char *VoltLimitMsg[CFG_MAX] =
{
  "7V ",
  "9V ",
  "12V",
  "15V",
  "25V",
  "TST"
};

int VoltLimit[CFG_MAX] =
{
  25,
  30,
  45,
  50,
  80,
  0
};

//analog volt calculation
//val=(adc_data/MAX)*ref_volt
//volt=(val/div_ratio)

typedef struct _reg_map_io_expandr_t
{
    DTIOI2CtoParallelConverter *expandr;
    byte expandr_bus;
    byte expandr_pin;
    byte expandr_pin_state;
}reg_map_io_expandr_t;

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ROTARY_CLK, ROTARY_DT, RotaryEncoder::LatchMode::FOUR3);

Adafruit_MCP3008 adc1_U2;
Adafruit_MCP3008 adc2_U3;
Adafruit_MCP3008 adc3_U4;

DTIOI2CtoParallelConverter ioExp1_U300(0x74);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter ioExp2_U301(0x75);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter ioExp3_U302(0x76);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 0)

// The LCD constructor - I2C address 0x38
LiquidCrystal_I2C lcd(0x38, 4, 5, 6, 0, 1, 2, 3, 7, POSITIVE);

static byte g_debouncedBtnState = 1;
static byte g_selectVolt = INVALID_VOLT;
static byte g_setLimit = 0;
static reg_map_io_expandr_t g_LEDMappingArr[MAX_LED] = {};
static int g_LEDcount = 0;
static int g_chanCount = 0;
static int g_adcReadings[MAX_CHANNELS] = {};

void toggleLEDs(int num_LED, bool on)
{
    if( (num_LED >= 0) && (num_LED < MAX_LED) )
    {
        if(g_LEDMappingArr[num_LED].expandr_bus == 0)
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, on); 
        }
        else // g_LEDMappingArr[num_LED].expandr_bus == 1
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, on);
        }
    }
}

void initLEDsOnExpandr(DTIOI2CtoParallelConverter *io_expandr)
{
    //init LEDs on P0x bus
    for(int P0_count = PIN1_7; ((P0_count >= PIN1_0) && (g_LEDcount < MAX_LED)); P0_count--)
    {
        io_expandr->pinMode0(P0_count, LOW);
        io_expandr->digitalWrite0(P0_count, HIGH);        
        
        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 0;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P0_count;
        g_LEDcount++;
    }

    //init LEDs on P1x bus
    for(int P1_count = PIN0_7; ((P1_count >= PIN0_0) && (g_LEDcount < MAX_LED)); P1_count--)
    {
        io_expandr->pinMode1(P1_count, LOW);
        io_expandr->digitalWrite1(P1_count, HIGH);

        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 1;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P1_count;
        g_LEDcount++;
    }
}

void displayStartMsg()
{
    lcd.setCursor(0,0);
    lcd.print("Volt Sense ");
    lcd.setCursor(11,0);
    lcd.print(app_ver);
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Select: NA");
    lcd.setCursor(0,1);
    lcd.print("Set Limit: 7V");
}

void readADC(Adafruit_MCP3008 *adc)
{
    for(byte chan = 0; ((chan < 8) && (g_chanCount < MAX_CHANNELS)); chan++)
    {
        g_adcReadings[g_chanCount++] = adc->readADC(chan);
    }
}

void pollADC() 
{
    g_chanCount = 0; //reset channel count before reading
    
    readADC(&adc1_U2);
    readADC(&adc2_U3);
    readADC(&adc3_U4);
    
    yield(); //yield to pass control to other tasks
}

void pollingRotary() 
{
    static int pos = 0;
    encoder.tick();
    
    int newPos = encoder.getPosition();
    if (pos != newPos)
    {
        if(0 <= newPos)
        {
            g_selectVolt = (newPos % CFG_MAX);
        }
        else
        {
            g_selectVolt = (CFG_MAX - (abs(newPos) % CFG_MAX)) % CFG_MAX;
        }
        
        lcd.setCursor(8,0);
        lcd.print(VoltLimitMsg[g_selectVolt]);
        
        pos = newPos;
    }
    
    yield(); //yield to pass control to other tasks
}

//returns true if state changed
bool debounceBtnSw(byte *state)
{
    static uint8_t count = DEBOUNCE_MS/CHECK_MS;
    bool state_changed = false;

    //read the door switch from the HW
    byte raw_state = digitalRead(ROTARY_SW);
    *state = g_debouncedBtnState;

    if (raw_state == g_debouncedBtnState)
    {
        //set the timer which allows a change from current state.
        count = DEBOUNCE_MS/CHECK_MS;
    }
    else
    {
        //state has changed - wait for new state to become stable.
        if (--count == 0)
        {
            // Timer expired - accept the change.
            g_debouncedBtnState = raw_state;
            state_changed = true;
            *state = g_debouncedBtnState;
            
            // And reset the timer.
            count = DEBOUNCE_MS/CHECK_MS;
        }
    }

    return state_changed;
}

void debounceBtnSWRoutine()
{
    byte switch_state = 0;
    
    //if door state changed, update the state
    if(debounceBtnSw(&switch_state))
    {
        if(!switch_state)
        {
            g_setLimit = g_selectVolt;
            lcd.setCursor(11,1);
            lcd.print(VoltLimitMsg[g_setLimit]);
        }
    }
    
    delay(CHECK_MS);
}

void setup() 
{
    Serial.begin(9600);
    //while (!Serial);
    
    Serial.print("Voltage Sensor ");
    Serial.println(app_ver);
    
    Wire.begin(); //need to start the Wire for I2C devices to function
    
    // Software SPI (specify all, use any available digital)
    // (sck, mosi, miso, cs);   
    adc1_U2.begin(SCK_PIN, MOSI_PIN, MISO_PIN, ADC1_CS);
    adc2_U3.begin(SCK_PIN, MOSI_PIN, MISO_PIN, ADC2_CS);
    adc3_U4.begin(SCK_PIN, MOSI_PIN, MISO_PIN, ADC3_CS);
    
    lcd.begin(16,2); // sixteen characters across - 2 lines
    lcd.backlight();
    
    //display app title and version
    displayStartMsg();
    
    //initialize the LED pins
    initLEDsOnExpandr(&ioExp1_U300);
    initLEDsOnExpandr(&ioExp2_U301);
    initLEDsOnExpandr(&ioExp3_U302);
    
    //initialize the Buzzer
    ioExp3_U302.pinMode0(BUZZER_PIN, LOW);
    ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
    
    Scheduler.startLoop(pollADC); //read ADC value thread
    
    Scheduler.startLoop(pollingRotary); //polling rotary knob thread
    
    Scheduler.startLoop(debounceBtnSWRoutine); //debouncing button switch thread
}

void loop() 
{
    const int FACTOR = 2;
    for(byte chan = 0; chan < MAX_CHANNELS; chan++)
    {
        //get LED mapping
        int green_led = (chan * FACTOR);
        int red_led = (chan * FACTOR) + 1;
            
        if(0 == g_setLimit)
        {
            Serial.print("ADC");
            Serial.print(chan);
            Serial.print(":");
            Serial.println(g_adcReadings[chan]);
            delay(300);
            toggleLEDs(green_led, LOW);
            toggleLEDs(red_led, LOW);
            ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
            delay(300);
            toggleLEDs(green_led, HIGH);
            toggleLEDs(red_led, HIGH);
            ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
            delay(300);
        }
        else if(INVALID_VOLT != g_setLimit)
        {
            //over voltage detected
            if(g_adcReadings[chan] > VoltLimit[g_setLimit])
            {
                ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
                
                toggleLEDs(green_led, LOW);
                toggleLEDs(red_led, HIGH);
            }
            else //normal voltage 
            {
                ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
                
                toggleLEDs(red_led, LOW);
                toggleLEDs(green_led, HIGH);
            }
        }
    }
}
