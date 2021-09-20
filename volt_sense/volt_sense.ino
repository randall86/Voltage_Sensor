// Voltage Sensor System
// Rev 2.0 (19/09/2021)
// - Maxtrax

#include <Adafruit_MCP3008.h>
#include <DTIOI2CtoParallelConverter.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Scheduler.h>

const char * app_ver = "v2.0";

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
const int DELAY_MS = 100;
const int POLLING_RATE_MS = DELAY_MS*3;
 
const byte MAX_CHANNELS = 18;
const byte MAX_LED = MAX_CHANNELS*2;
const byte MAX_GROUPING = MAX_CHANNELS/2;
const byte MAX_PROFILE = 20;
const byte INVALID_VOLT = 0xFF;

enum _Volt_config
{
    CFG_10V,
    CFG_25V,
    CFG_50V,
    CFG_80V,
    CFG_100V,
    CFG_120V,
    CFG_150V,
    CFG_180V,
    CFG_200V,
    CFG_220V,
    CFG_250V,
    CFG_280V,
    CFG_300V,
    MAX_CFG
};

enum _Volt_states
{
    INIT,
    START,
    STOP,
    ACK,
    CFG,
    TEST,
    MAX_STATE
};

enum _Stop_states
{
    RESUME,
    RESET,
    RECONFIG,
    MAX_STOP_STATE
};

const char *StateMsg[MAX_STATE] =
{
  "Ready          ",
  "Sensing...     ",
  "Stopped        ",
  "Acknowledged   ",
  "Configuration  ",
  "Testing...     "
};

const char *StoppedStateMsg[MAX_STOP_STATE] =
{
  "Resume?        ",
  "Reset?         ",
  "Reconfigure?   "
};

typedef struct _chan_val_limit
{
    int chan_val;
    int volt_lower_limit;
    int volt_upper_limit;
}chan_val_limit_t;

typedef struct _volt_cfg
{
    int min_volt;
    int max_volt;
}volt_cfg_t;

typedef struct _volt_profile
{
    int vb_profile;
    int vc_profile;
}volt_profile_t;

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
DTIOI2CtoParallelConverter ioExp2_U301(0x76);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter ioExp3_U302(0x75);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 0)

// The LCD constructor - I2C address 0x38
LiquidCrystal_I2C lcd(0x38, 4, 5, 6, 0, 1, 2, 3, 7, POSITIVE);

static byte g_debouncedBtnState = 1;
static bool g_switchPressed = false;
static reg_map_io_expandr_t g_LEDMappingArr[MAX_LED] = {};
static int g_LEDcount = 0;
static int g_chanCount = 0;
static chan_val_limit_t g_adcReadings[MAX_CHANNELS] = {};
static byte g_currentState = INIT;
static bool g_faultDetected = false;
static int g_newPos = 0;
static int g_selectGroup = -1;
static int g_groupProfile[MAX_GROUPING] = {};

//ADC calculated values with 5% tolerance
static const volt_cfg_t g_config[MAX_CFG] =
{
    {32, 36},       //10V
    {81, 90},       //25V
    {162, 179},     //50V
    {259, 286},     //80V
    {324, 358},     //100V
    {389, 430},     //120V
    {486, 537},     //150V
    {583, 644},     //180V
    {648, 716},     //200V
    {713, 788},     //220V
    {810, 895},     //250V
    {907, 1003},    //280V
    {972, 1023}     //300V
};

static const volt_profile_t g_profile[MAX_PROFILE] =
{
    {CFG_10V, CFG_50V},     //Profile01
    {CFG_10V, CFG_100V},    //Profile02
    {CFG_25V, CFG_80V},     //Profile03
    {CFG_25V, CFG_120V},    //Profile04
    {CFG_50V, CFG_100V},    //Profile05
    {CFG_50V, CFG_150V},    //Profile06
    {CFG_80V, CFG_200V},    //Profile07
    {CFG_80V, CFG_220V},    //Profile08
    {CFG_100V, CFG_120V},   //Profile09
    {CFG_100V, CFG_200V},   //Profile10
    {CFG_120V, CFG_150V},   //Profile11
    {CFG_120V, CFG_180V},   //Profile12
    {CFG_150V, CFG_180V},   //Profile13
    {CFG_150V, CFG_200V},   //Profile14
    {CFG_180V, CFG_200V},   //Profile15
    {CFG_200V, CFG_220V},   //Profile16
    {CFG_220V, CFG_250V},   //Profile17
    {CFG_250V, CFG_280V},   //Profile18
    {CFG_250V, CFG_300V},   //Profile19
    {CFG_280V, CFG_300V}    //Profile20
};

#define FACTOR 2
#define GRN_LED(chan) (chan * FACTOR)
#define RED_LED(chan) ((chan * FACTOR) + 1)

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
    for(int P1_count = PIN1_7; ((P1_count >= PIN1_0) && (g_LEDcount < MAX_LED)); P1_count--)
    {
        io_expandr->pinMode1(P1_count, LOW);
        io_expandr->digitalWrite1(P1_count, LOW);        
        
        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 1;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P1_count;
        g_LEDcount++;
    }

    //init LEDs on P1x bus
    for(int P0_count = PIN0_7; ((P0_count >= PIN0_0) && (g_LEDcount < MAX_LED)); P0_count--)
    {
        io_expandr->pinMode0(P0_count, LOW);
        io_expandr->digitalWrite0(P0_count, LOW);

        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 0;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P0_count;
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
}

void readADC(Adafruit_MCP3008 *adc)
{
    for(byte chan = 0; ((chan < 8) && (g_chanCount < MAX_CHANNELS)); chan++)
    {
        g_adcReadings[g_chanCount++].chan_val = adc->readADC(chan);
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
    encoder.tick();
    
    int newPos = encoder.getPosition();
    if (g_newPos != newPos)
    {
        g_newPos = newPos;
    }
    
    yield(); //yield to pass control to other tasks
}

int getRotarySelection(int pos, int max)
{
    int ret = 0;
    
    if(0 <= pos)
    {
        ret = (pos % max);
    }
    else
    {
        ret = (max - (abs(pos) % max)) % max;
    }
    
    return ret;
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
    
    if(debounceBtnSw(&switch_state))
    {
        if(!switch_state)
        {
            g_switchPressed = true;
        }
    }
    
    delay(CHECK_MS);
}

void doInitialization()
{
    //clear the fault detect flag
    g_faultDetected = false;
    
    //initialize all LED state to good
    for(byte chan = 0; chan < MAX_CHANNELS; chan++)
    {
        toggleLEDs(GRN_LED(chan), LOW);
        toggleLEDs(RED_LED(chan), HIGH);
    }
    
    //turn off the fault indication buzzer
    ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
}

void printGroupProfileSelection()
{
    char text[16] = {};
    //GroupX ProfXX
    snprintf(text, 16, "Group%d Prof%02d  ", g_selectGroup+1, g_groupProfile[g_selectGroup]+1);
    
    lcd.setCursor(0,1);
    lcd.print(text);
}

void updateConfiguration()
{
    //initialize all channel volt limits according to the selected group profile
    for(byte group = 0; group < MAX_GROUPING; group++)
    {
        //get selected profile for group
        if(g_groupProfile[group] < MAX_PROFILE)
        {
            int vb = g_profile[g_groupProfile[group]].vb_profile;
            int vc = g_profile[g_groupProfile[group]].vc_profile;
            
            //assign limit for vb channel in group
            int vb_offset = (group * 2);
            if(vb_offset < MAX_CHANNELS)
            {
                g_adcReadings[vb_offset].volt_lower_limit = g_config[vb].min_volt;
                g_adcReadings[vb_offset].volt_upper_limit = g_config[vb].max_volt;
            }
            
            //assign limit for vc channel in group
            int vc_offset = vb_offset + 1;
            if(vc_offset < MAX_CHANNELS)
            {
                g_adcReadings[vc_offset].volt_lower_limit = g_config[vc].min_volt;
                g_adcReadings[vc_offset].volt_upper_limit = g_config[vc].max_volt;
            }
        }       
    }
}

void handleStopState()
{
    const byte new_state[MAX_STOP_STATE] = {START, START, CFG};
    int curr_selection = -1;
    bool print_confirmation = false;
    encoder.setPosition(0);
    
    while(STOP == g_currentState)
    {
        int selected_state = getRotarySelection(g_newPos, MAX_STOP_STATE);
        if(selected_state < MAX_STOP_STATE)
        {
            if(curr_selection != selected_state)
            {
                curr_selection = selected_state;
                lcd.setCursor(0,1);
                lcd.print(StoppedStateMsg[curr_selection]);
            }
            
            if(g_switchPressed)
            {
                if(!print_confirmation)
                {
                    g_switchPressed = false;
                    
                    lcd.setCursor(0,1);
                    lcd.print("Confirm?       ");
                    print_confirmation = true;
                }
                
                if(g_switchPressed)
                {
                    g_switchPressed = false;
                    
                    if(RESUME != curr_selection) //reset LEDS and Buzzer if not resuming
                    {
                        doInitialization();
                    }
                    
                    g_currentState = new_state[curr_selection];
                    lcd.clear();
                }
            }
        }
        
        delay(POLLING_RATE_MS);
    }
}

void handleConfigState()
{
    bool print_confirmation = false;
    int group_pos = 0;
    encoder.setPosition(0);
    
    while(CFG == g_currentState)
    {
        int selected_group = getRotarySelection(g_newPos, MAX_GROUPING+1); //additional 1 for exit selection
        if(selected_group < MAX_GROUPING)
        {
            if(g_selectGroup != selected_group)
            {
                g_selectGroup = selected_group;
                printGroupProfileSelection();
                group_pos = g_newPos;
            }
            
            if(g_switchPressed)
            {
                g_switchPressed = false;
                encoder.setPosition(0);
                
                bool profile_selected = false;
                while(false == profile_selected)
                {
                    int selected_profile = getRotarySelection(g_newPos, MAX_PROFILE);
                    if(g_groupProfile[g_selectGroup] != selected_profile)
                    {
                        g_groupProfile[g_selectGroup] = selected_profile;
                        printGroupProfileSelection();
                    }
                    
                    if(g_switchPressed)
                    {
                        g_switchPressed = false;
                        profile_selected = true;
                        encoder.setPosition(group_pos);
                    }

                    delay(DELAY_MS);
                }
            }
            print_confirmation = false;
        }
        else
        {
            if(!print_confirmation)
            {
                lcd.setCursor(0,1);
                lcd.print("Confirm?       ");
                print_confirmation = true;
            }
            
            if(g_switchPressed)
            {
                g_switchPressed = false;
                
                updateConfiguration();
                g_currentState = START;
                lcd.clear();
            }
        }
        
        delay(POLLING_RATE_MS);
    }
}

void handleUIRoutine()
{
    static byte state = MAX_STATE;
    
    if(state != g_currentState)
    {
        state = g_currentState;
        switch(state)
        {       
            case INIT:
            {
                //display app title and version
                displayStartMsg();
                if(g_switchPressed)
                {
                    g_currentState = TEST;
                }
                else
                {
                    doInitialization();
                    g_currentState = CFG;
                }
                g_switchPressed = false;
                break;
            }
            
            case START:
            {
                lcd.setCursor(0,0);
                lcd.print(StateMsg[state]);
                break;
            }
            
            case STOP:
            {
                lcd.setCursor(0,0);
                lcd.print(StateMsg[state]);
                delay(1000);
                handleStopState();
                break;
            }
            
            case ACK:
            {
                lcd.setCursor(0,0);
                lcd.print(StateMsg[state]);
                delay(1000);
                g_currentState = STOP;
                break;
            }
            
            case CFG:
            {
                lcd.setCursor(0,0);
                lcd.print(StateMsg[state]);
                handleConfigState();
                break;
            }
            
            case TEST:
            {
                lcd.setCursor(0,0);
                lcd.print(StateMsg[state]);
                break;
            }
        }
    }
    
    yield(); //yield to pass control to other tasks
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
    
    //initialize the LED pins
    initLEDsOnExpandr(&ioExp1_U300);
    initLEDsOnExpandr(&ioExp2_U301);
    initLEDsOnExpandr(&ioExp3_U302);
    
    //initialize the Buzzer
    ioExp3_U302.pinMode0(BUZZER_PIN, LOW);
    ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
    
    Scheduler.startLoop(pollADC); //read ADC value thread
    Scheduler.startLoop(pollingRotary); //polling rotary knob thread
    Scheduler.startLoop(debounceBtnSWRoutine); //debouncing button switch thread
    Scheduler.startLoop(handleUIRoutine); //display thread
}

void loop() 
{
    switch(g_currentState)
    {       
        case INIT:
        {
            break;
        }
        
        case START:
        {
            for(byte chan = 0; chan < MAX_CHANNELS; chan++)
            {
                //incorrect voltage detected
                if( (0 != g_adcReadings[chan].chan_val) &&
                    ((g_adcReadings[chan].chan_val < g_adcReadings[chan].volt_lower_limit) ||
                    (g_adcReadings[chan].chan_val > g_adcReadings[chan].volt_upper_limit)) )
                {
                    ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
                    
                    toggleLEDs(GRN_LED(chan), HIGH);
                    toggleLEDs(RED_LED(chan), LOW);
                    g_faultDetected = true;
                }
            }
            
            if(g_faultDetected && g_switchPressed) //if fault and switch pressed goto ACK
            {
                g_switchPressed = false;
                g_currentState = ACK;
            }
            else if(g_switchPressed) //if only switch pressed goto STOP
            {
                g_switchPressed = false;
                g_currentState = STOP;
            }
            
            break;
        }
        
        case STOP:
        {
            break;
        }
        
        case ACK:
        {
            if(g_faultDetected)
            {
                ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
                g_faultDetected = false;
            }
            break;
        }
        
        case CFG:
        {
            break;
        }
        
        case TEST:
        {
            for(byte chan = 0; chan < MAX_CHANNELS; chan++)
            {
                Serial.print("ADC");
                Serial.print(chan);
                Serial.print(":");
                Serial.println(g_adcReadings[chan].chan_val);
                delay(300);
                toggleLEDs(GRN_LED(chan), LOW);
                toggleLEDs(RED_LED(chan), LOW);
                ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
                delay(300);
                toggleLEDs(GRN_LED(chan), HIGH);
                toggleLEDs(RED_LED(chan), HIGH);
                ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
                delay(300);
            }
            break;
        }
    }
    delay(POLLING_RATE_MS);
}
