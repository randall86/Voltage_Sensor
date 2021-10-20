// Voltage Sensor System
// Rev 2.4 (21/10/2021)
// - Maxtrax

#include <Adafruit_MCP3008.h>
#include <DTIOI2CtoParallelConverter.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Scheduler.h>
#include <CSV_Parser.h>
#include <SPI.h>
#include <SD.h>

const char * app_ver = "v2.4";
const char * cfg_file = "config.csv";

const byte ROTARY_CLK = 3;  //Output A
const byte ROTARY_DT = 4;   //Output B
const byte ROTARY_SW = 2;   //Switch

const byte ADC3_CS = 5;
const byte ADC2_CS = 6;
const byte ADC1_CS = 7;
const byte ADC_MOSI_PIN = 8;
const byte ADC_SCK_PIN = 9;
const byte ADC_MISO_PIN = 10;

const byte BUZZER_PIN = PIN0_7;

const int chipSelect = SDCARD_SS_PIN;

const int DEBOUNCE_MS = 50;
const int CHECK_MS = 10;
const int DELAY_MS = 100;
const int POLLING_RATE_MS = DELAY_MS*3;
 
const byte MAX_CHANNELS = 18;
const byte MAX_LED = MAX_CHANNELS*2;
const byte MAX_GROUPING = MAX_CHANNELS/2;
const byte MAX_PROFILE = 26;
const byte INVALID_VOLT = 0xFF;

enum _Volt_config
{
    CFG_DIS,
    CFG_0V,
    CFG_1V,
    CFG_3V,
    CFG_4_32V,
    CFG_5V,
    CFG_5_5V,
    CFG_6_5V,
    CFG_7V,
    CFG_14V,
    CFG_16V,
    CFG_20V,
    CFG_22V,
    CFG_24V,
    CFG_32V,
    CFG_39V,
    CFG_40V,
    CFG_45V,
    CFG_49V,
    CFG_50V,
    CFG_60V,
    CFG_75V,
    CFG_80V,
    CFG_200V,
    CFG_320V,
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

typedef struct _fault_ack
{
    bool fault_detected;
    bool is_acknowledged;
}fault_ack_t;

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
static fault_ack_t g_chanFaultAck[MAX_CHANNELS] = {}; 
static bool g_storAvai = false;

//ADC calculated values with 5% tolerance
static const volt_cfg_t g_config[MAX_CFG] =
{
    {0, 0},         //Disabled
    {0, 7},         //0V
    {1, 20},         //1V
    {5, 20},        //3V
    {10, 20},       //4.32V
    {14, 20},       //5V
    {16, 21},       //5.5V and 5.6V
    {19, 24},       //6.5V
    {21, 27},       //7V
    {43, 53},       //14V
    {49, 60},       //16V
    {61, 75},       //20V
    {68, 83},       //22V
    {74, 90},       //24V
    {98, 120},      //32V
    {120, 146},     //39V
    {123, 150},     //40V
    {138, 169},     //45V
    {150, 184},     //49V
    {153, 188},     //50V
    {184, 225},     //60V
    {230, 281},     //75V
    {246, 300},     //80V
    {614, 750},     //200V
    {982, 1023}     //320V
};

static const volt_profile_t g_profile[MAX_PROFILE] =
{
    {CFG_DIS,   CFG_DIS},   //Profile00 - Disable
    {CFG_0V,    CFG_4_32V}, //Profile01
    {CFG_0V,    CFG_6_5V},  //Profile02
    {CFG_0V,    CFG_200V},  //Profile03
    {CFG_0V,    CFG_60V},   //Profile04
    {CFG_1V,    CFG_49V},   //Profile05
    {CFG_1V,    CFG_45V},   //Profile06
    {CFG_20V,   CFG_0V},    //Profile07
    {CFG_0V,    CFG_5V},    //Profile08
    {CFG_0V,    CFG_20V},   //Profile09
    {CFG_0V,    CFG_3V},    //Profile10
    {CFG_0V,    CFG_75V},   //Profile11
    {CFG_0V,    CFG_5_5V},  //Profile12
    {CFG_1V,    CFG_39V},   //Profile13
    {CFG_1V,    CFG_40V},   //Profile14
    {CFG_1V,    CFG_50V},   //Profile15
    {CFG_1V,    CFG_14V},   //Profile16
    {CFG_0V,    CFG_24V},   //Profile17
    {CFG_1V,    CFG_60V},   //Profile18
    {CFG_0V,    CFG_320V},  //Profile19
    {CFG_0V,    CFG_80V},   //Profile20
    {CFG_16V,   CFG_0V},    //Profile21
    {CFG_0V,    CFG_24V},   //Profile22
    {CFG_0V,    CFG_32V},   //Profile23
    {CFG_0V,    CFG_22V},   //Profile24
    {CFG_0V,    CFG_5_5V}   //Profile25
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
    
    int newPos = ((-1)*encoder.getPosition()); //invert +ve to -ve and vice versa
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
        g_chanFaultAck[chan].is_acknowledged = false;
        g_chanFaultAck[chan].fault_detected = false;
    }
    
    //turn off the fault indication buzzer
    ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
}

void printGroupProfileSelection()
{
    char text[16] = {};
    
    if(0 == g_groupProfile[g_selectGroup])
    {
        //GroupX Disable
        snprintf(text, 16, "Group%d Disable ", g_selectGroup+1);
    }
    else
    {
        //GroupX ProfXX
        snprintf(text, 16, "Group%d Prof%02d  ", g_selectGroup+1, g_groupProfile[g_selectGroup]);
    }
    
    lcd.setCursor(0,1);
    lcd.print(text);
}

void removeConfiguration()
{
    //remove config file from SD card if available
    if(true == g_storAvai)
    {
        if(SD.exists(cfg_file))
        {
            SD.remove(cfg_file);
        }
    }
}

void readConfiguration()
{
    // see if the card is present and can be initialized:
    if (SD.begin(chipSelect)) 
    {
        g_storAvai = true;
        
        String format = ""; //config data string
        for(byte column = 0; column < MAX_GROUPING; column++)
        {
            format += "d";
        }
        
        // (format, has_header, delimiter); 
        CSV_Parser cp(format.c_str(), false, ',');
        
        // The line below (readSDfile) wouldn't work if SD.begin wasn't called before.
        // readSDfile can be used as conditional, it returns 'false' if the file does not exist.
        if (cp.readSDfile(cfg_file))
        {
            for(int index = 0; index < cp.getColumnsCount(); index++)
            {
                int16_t *column_data = (int16_t*)cp[index];
                g_groupProfile[index] = column_data[0]; //update the saved profile configuration
            }
        }
        else
        {
            Serial.println("Config file does not exist...");
        }
    }
    else
    {
        Serial.println("Card failed, or not present");
    }
}

void updateConfiguration()
{
    String dataString = ""; //config data string
    
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
        
        //form the configured profile for each group separated by comma
        dataString += String(g_groupProfile[group]);
        if((group + 1) < MAX_GROUPING)
        {
            dataString += ",";
        }
    }
    
    //save config to SD card if available
    if(true == g_storAvai)
    {
        File dataFile = SD.open(cfg_file, O_WRITE | O_CREAT | O_TRUNC);
        if(dataFile)
        {
            dataFile.println(dataString);
            dataFile.close();
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
                removeConfiguration();
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
    adc1_U2.begin(ADC_SCK_PIN, ADC_MOSI_PIN, ADC_MISO_PIN, ADC1_CS);
    adc2_U3.begin(ADC_SCK_PIN, ADC_MOSI_PIN, ADC_MISO_PIN, ADC2_CS);
    adc3_U4.begin(ADC_SCK_PIN, ADC_MOSI_PIN, ADC_MISO_PIN, ADC3_CS);
    
    lcd.begin(16,2); // sixteen characters across - 2 lines
    lcd.backlight();
    
    //initialize the LED pins
    initLEDsOnExpandr(&ioExp1_U300);
    initLEDsOnExpandr(&ioExp2_U301);
    initLEDsOnExpandr(&ioExp3_U302);
    
    //initialize the Buzzer
    ioExp3_U302.pinMode0(BUZZER_PIN, LOW);
    ioExp3_U302.digitalWrite0(BUZZER_PIN, LOW);
    
    //read for available config
    readConfiguration();
    
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
                //disabled
                if((0 == g_adcReadings[chan].volt_lower_limit) &&
                        (0 == g_adcReadings[chan].volt_upper_limit))
                {
                    toggleLEDs(GRN_LED(chan), HIGH);
                    toggleLEDs(RED_LED(chan), HIGH);
                    continue;
                }
                //already acknowledged
                else if(g_chanFaultAck[chan].is_acknowledged)
                {
                    continue;
                }
                //incorrect voltage detected
                else if((g_adcReadings[chan].chan_val < g_adcReadings[chan].volt_lower_limit) ||
                        (g_adcReadings[chan].chan_val > g_adcReadings[chan].volt_upper_limit))
                {
                    ioExp3_U302.digitalWrite0(BUZZER_PIN, HIGH);
                    
                    toggleLEDs(GRN_LED(chan), HIGH);
                    toggleLEDs(RED_LED(chan), LOW);
                    g_chanFaultAck[chan].fault_detected = true;
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
                for(byte chan = 0; chan < MAX_CHANNELS; chan++)
                {
                    g_chanFaultAck[chan].is_acknowledged = g_chanFaultAck[chan].fault_detected;
                }
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
