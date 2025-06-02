/*
Fuses für TCXO setzen:
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude -C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf -v -patmega328p -cstk500v2 -Pusb -U lfuse:w:0xE0:m -U hfuse:w:0xDA:m -U efuse:w:0xFD:m

.hex flashen:
/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude -C/Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf -v -patmega328p -cstk500v2 -Pusb -U flash:w:/Users/peaceman/code/Clockwork/build/Clockwork.ino.hex:i

*/

/* Todo

√ Test ESS-in signal amp
- Test it with actual audio levels
- Erkenne ich einen T510?
√ Summierverstärker hinzufügen
√ Test writing to the DAC_ENABLE_PIN
√ Test Relay Board Detection
√ Detect Projector Type — A0, 6 values from 5x 10k Resistors:
    No Projector connected
    Bauer Studioklasse (T610/T525/T510/T502) (12:1, 1.63 V)
    Bauer P8 (Custom:1, 1.63V)
    Bauer P8 Selecton (3:1, 1,63V
    Braun Visacustic (3:1, 1.89V)
    Reserve
√ Parametrize the per-projector settings in a struct, and use them, instead of static values
√ Add DAC init values for both 18 and 24 switchpos to the struct (and use them accordingly)
√ Make sure the DAC is disabled on first cold start (FreqMeasure), we often get a false 18 here
- Do we need to move the speed/dither configs to the projector struct, since they timer 
  osciallates and impulse freq (e.g. *12), or can we just tweak the timer factor?
- Learn IR Codes
- Send IR Code when projector starts (or Follow Audio / Start beep?)
- Implement Ext. Imp Mode
    Attach INT1 ISR, counting up
    Follow INT1 count, not the timer_pulses
    Detect Follow/Guide Switch position
    If Follow:
        Enable STOP_EN_PIN
        On first Impulse, Disable STOP_EN_PIN
    If Guide:
        On first Impulse, send learned IR code
    FreqMeasure (like in Auto Mode)
    Update Timecode Display
    Stop Detection
- Test & Design Lamp Relay Board
- Allow disabling Lamp Relay Board / Restore native "Programming Mode" (Din-Connector-Connector Tap?)
- Test "Pause Button" SSR (is it shorting?)
- Save "good" DAC values for a given FPS in EEPROM after a while
- Use smaller font for less resource usage
√ Read Guide/Follow Switch (D12)
    - Re-Read START_MODE_SWITCH_PIN when idling
√ Detect Lamp Relais Board (D7/STOP_EN pulled to GND)
- Fine Tune Potentiometer on Projector Board
- Create ESS out on D11 (Square wave)



Bugs:
- Starting in NONE does not work yet, DAC stays connected

Todo:
- Do I still need force_redraw?
make the 5 call of updateDigit more efficient: stop calculating if a digit did not need to beupdated.

- Use the display
    - make fps_rn an int
    - create "fps", "Auto" and "Off" tiles to save some progmem (_r font is 4KB bigger than _n)
- Use FreqMeasure in FPS_NONE
- actually apply the timer_factor
- speed.name is probably not really needed, saves 42 Bytes PROGMEM and 6 Bytes RAM
- use shaft_pulses instead of shaft_frames to determine past speed, more precise and omits a multiplication
- flash the DAC just once on very first start
- Decrement Shaft Impulses after detected shaft noise!
- Reset Counters? (long press both buttons?)
- Implement non-sepmag mode — on speed change when running,
        - Forget any previous errors and start fresh
        + allows for quick speed changes
        + ideal mode for use as "Peaceman's Box" (ESS out mode)
        + good for telecine (which could also just restart though)
        - loses sync with any sepmag audio
    Timecode would restart at 0:00:00.00
- Rangieren (slow) (idea: could get enabled by pressing +/- buttons while turning the projector tho "thread" pos)
- "start the audio" IR
- ESS in

Hardware:
- add IR emitter to start a device: Send o A1, Rcv on A2
- (Strobe Output to check other projector's speed)
- (ESS Out)

Notes:
- Is the dithering really delivering the target frequency? Seems to be slightly below.
  This might be diluted due to run up and breaking. Cross-validate the frac32!


*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Adafruit_MCP4725.h> // Fancy DAC for voltage control
#include <Wire.h>             // i2c to talk to the DAC
#include <PID_v1.h>           // using 1.2.0 from https://github.com/br3ttb/Arduino-PID-Library
#include <FreqMeasure.h>
#include <Button2.h>
#include <U8x8lib.h> // Display Driver

// projector specific constants
// const uint16_t DAC_INITIAL_VALUE = 1500; // This should equal a voltage that leads to approx 16-20 fps (on 18 fps) or 22-26 fps (on 24).
const unsigned long STOP_THRESHOLD = 25000; // microseconds until a stop will be detected
// uint8_t shaft_pulse_count = 12;             // Size of the Median Window. We use 12 to capture one entire shaft revolution


// pin consts
const byte SHAFT_PULSE_PIN = 2;  // aka INT0
const byte LED_RED_PIN = 5; //  Out of Sync
const byte FPS_PULSE_OUT_PIN = 11; //  12 on breadboard. Pulse when Crystal is enabled. Todo: This still needs to be coded
const byte DAC_ENABLE_PIN = 9;
const byte AUDIOMODE_SWITCH_PIN = 12;
const byte STOP_EN_PIN = 7; // Flipping this HIGH stops the projector if DAC_ENABLE_PIN is HIGH too
const byte LAMP_RELAY_DETECT_PIN = 7; // The same pin (in input mode) reflects if a Lamp Relais Board is connected
const byte PROJ_ID_PIN = A0; // This is used to detect the projector type, if one is connected

const byte LEFT_BTTN_PIN = 10;
const byte RIGHT_BTTN_PIN = A3;     // 13 on breadboard
const byte DROP_BACK_BTTN_PIN = 4;  // 11 on breadboard
const byte CATCH_UP_BTTN_PIN = 6;   // 12 on breadboard

// Use this for PID tuning with Pots
// const byte P_PIN = A6;
// const byte I_PIN = A7;
// const byte D_PIN = A2;

#define BTTN_NONE 0
#define BTTN_LEFT 1
#define BTTN_RIGHT 2

// ---- Define useful time constants and macros ------------------------------------
#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)

// --- some custome gfx for the display 
/*
const uint8_t unlockedLockTop[24] = {
    0b00000000, 
    0b00000000, 
    0b10000000, 
    0b10000000, 
    0b10000000, 
    0b10000000, 
    0b10000000, 
    0b10000000,
    0b10000000, 
    0b10000000, 
    0b10000000, 
    0b11100000, 
    0b11111000, 
    0b10001100, 
    0b00000110, 
    0b00000110,
    0b00000110, 
    0b00000110, 
    0b00001100, 
    0b11111000, 
    0b11100000, 
    0b00000000, 
    0b00000000, 
    0b00000000};
const uint8_t lockedLockTop[16] = {
    0b00000000, 0b00000000, 0b10000000, 0b11100000, 0b11111000, 0b10001100, 0b10000110, 0b10000110,
    0b10000110, 0b10000110, 0b10001100, 0b11111000, 0b11100000, 0b10000000, 0b00000000, 0b00000000};
const uint8_t lockBottom[16] = {
    0b00000000, 0b00000000, 0b01111111, 0b01111111, 0b01111111, 0b01111111, 0b01011001, 0b01000000,
    0b01000000, 0b01011001, 0b01111111, 0b01111111, 0b01111111, 0b01111111, 0b00000000, 0b00000000};
const uint8_t twoThirdsTop[8] = {
    0b01000010, 0b01100001, 0b01010001, 0b01001110, 0b10000000, 0b01000000, 0b00100000, 0b00010000};
const uint8_t twoThirdsBottom[8] = {
    0b00001000, 0b00000100, 0b00000010, 0b00000001, 0b00100010, 0b01001001, 0b01001001, 0b00110110};
*/
const uint8_t emptyTile[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Timer Variables
volatile uint32_t timer_pulses = 0;
volatile uint8_t timer_modulus = 0; // For Modulo in the ISR, to compensate the timer_factor
volatile uint32_t last_pulse_timestamp; // Timestamp of the last pulse, used to detect a stop
volatile uint32_t dither_accumulator_32 = 0; // Accumulator for Timer Dithering


// Shaft Encoder Variables
volatile uint32_t shaft_pulses = 0;
volatile uint32_t shaft_frames = 0;   // This is the actually advanced frames (pulses / shaft_segment_disc_divider)
volatile uint8_t shaft_modulus = 0;            // For Modulo in the ISR, to compensate the multiple pulses per revolution
volatile bool new_shaft_impulse_available = false;
uint32_t projector_start_millis = 0; // To track the start time of the projector running


uint8_t projector_speed_switch = 0;  // To track the detected position of the projector's speed switch (18 or 24)

// flags to assure reading only once both ISRs have done their duty
volatile bool shaft_pulse_count_updated;

// Instantiate the PID
double pid_setpoint, pid_input, pid_output;
PID myPID(&pid_input, &pid_output, &pid_setpoint, 0, 0, 0, REVERSE);

// Instantiate the DAC
Adafruit_MCP4725 dac;

// Instantiate the Buttons
Button2 leftButton, rightButton, dropBackButton, catchUpButton;

// Instantiate the Display
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

byte currently_selected_mode;

enum ProjectorStates
{
    PROJ_IDLE,
    PROJ_RUNNING
};
byte projector_state = PROJ_IDLE;

enum BlendModes
{
    BLEND_SEPMAG, // Option A: Convert timer_pulses to new speed: Projector will catch up until sync is reached again.
    BLEND_RESET,  // Option B: Forget any previous pulses and start fresh
    BLEND_EQUAL   // Option C: keep old errors around and correct them too
};
byte blend_mode = BLEND_SEPMAG;


// Name the indices of the speed config struct
// Make sure these align with the SpeedConfig struct array below.
#define FPS_NONE    0
#define FPS_EXTERN  1
#define FPS_AUTO    2
#define FPS_16_2_3  3   // => 200 Hz
#define FPS_18      4   // => 216 Hz
#define FPS_23_976  5   // => ~287.712 Hz
#define FPS_24      6   // => 288 Hz
#define FPS_25      7   // => 300 Hz
#define MODES_COUNT 8   // to know where to roll over in the menu

// Struct with all the information we need to configure the controller for a speed:
struct SpeedConfig
{
    /*
    The dither pair is used to approximate an impossible speed that the Timer1 could not precisely generate at 16 MHz.
    They are all calculated for prescaler 8 (2 MHz) to achieve high precision at manageable CPU load.
    The values are tailored for “fps * 12” = final frequency, since the Bauer's shaft has 12 segments per revolution.
    Explanation:
        - base = floor( (2000000 / (target freq)) ) or floor( (2000000 / (a multiple)) )
        - frac32 = (decimal point * 2^32) (UQ0.32 fixed decimal point)
        - timerFactor => Software divider in the ISR, some multiples of the target freq give higher precision.

    Examples:
        - FPS_9 => 108 Hz = 864 Hz ISR / 8
            => idealDiv = 2314.8148148..., base=2314, fraction=~0.8148148
            => frac32 = round(0.8148148 * 2^32) = 0xD0BE9C00
            => Endfreq ~ 108.000000 => 0 ppm
            {name, 2314, 0xD0BE9C00, 8, 9.000000, dac, dac-init}
        - FPS_16_2_3 => 200 Hz => idealDiv=10000 => fraction=0 => no dithering => 0 ppm
        - FPS_18 => 216 Hz = 864 Hz ISR / 4
        - FPS_23_976 => fraction = 288000/1001 ~ 287.712287712288 Hz
            => best base   = 6951
            => best frac32 = 0x638E38E4  (decimal 1670265060)
            => freqActual  = 2147483648000000/7463996984889 ~ 287.712287712283 Hz
            => error Hz    = -0.0000000000004
        - FPS_24 => 288 Hz = 864 / 3
        - FPS_25 => 300 Hz => idealDiv=6666.666..., fraction=0.666..., frac32=0xAAAAAAAB => 0 ppm

    To get the fraction in a readable form, do frac/2^32.
    So for NTSC: 0x638E38E4/2^32 = 0.388888888992369174957275390625, then add the base. To derive the achieved freq:

    Clock        Frac                 Base     Blades
    (2000000 / ((0x638E38E4 / 2^32) + 6951)) / 12       = 23.976023976023 61911029165406179609278975693918589246146597964225857607403057768128024783617845662574...
                              Actually needed 24/1001:  = 23.976023976023 97602397602397602397602397602397602397602397602397602397602397602397602397602397602397...
    ... close enough!

    Check dither-conf.py to generate more tuples.

    */
    char name[8];           // short name for debugging
    volatile uint16_t dither_base;   // Grundwert für OCR1A
    volatile uint32_t dither_frac32; // fractional part (UQ0.32), 0..(2^32-1)
    uint8_t timer_factor;   // a frequency multiplier for each timer config (some multiples give better accuracy)
    float end_freq;         // the actual frequency we get with this config as an approx. float
    bool dac_enable;        // to allow freewheeling (no speed controller impact)
};
SpeedConfig speed;

static const SpeedConfig PROGMEM s_speed_table[] = {
    /*
    This is a table of values needed to generate a certain speed. To save memory, we copy one just struct of values
    to memory when needed.
    */
   {"Off", 2313, 0xD0BE9C00, 3, 0, 0}, /* Need dummy values here to not f up the timer. */
   {"Ext Imp", 2313, 0xD0BE9C00, 3, 0, 1}, // External Pulse Input Mode — Todo: Not sure about the timer init values here.
   {"Auto", 0, 0, 0, 0, 0},
   {"16  fps", 9999, 0x00000000, 1, 16.666666, 1},
   {"18 fps", 2313, 0xD0BE9C00, 4, 18.000000, 1},
   {"23.976", 6950, 0x638E38E4, 1, 23.976024, 1},
   {"24 fps", 2313, 0xD0BE9C00, 3, 24.000000, 1},
   {"25 fps", 6665, 0xAAAAAAAB, 1, 25.000000, 1}};
   
   // !!!!!  Reduced all base values by 1 for exact acuracy. Not sure yet why... off by 1 it is ¯\_(ツ)_/¯

enum ProjectorType
{
    // These are the projector types we can detect via the voltage divider on A0
    NONE,
    BAUER_T,
    P8,
    SELECTON,
    VISACUSTIC,
    RESERVE
};

struct ProjectorConfig
{
    char name[8];                       // short name for debugging
    uint8_t pulses_per_rotation;        // The number of pulses per rotation of the projector's shaft
    uint16_t framecount_until_stable;   // The number of frames until the speed is considered stable
 
    uint16_t dac_init18_16;       // The initial DAC value for 16 2/3 fps at switchpos 18
    uint16_t dac_init18_18;       // The initial DAC value for 18 fps at switchpos 18
    uint16_t dac_init18_23;       // The initial DAC value for 23.976 fps at switchpos 18
    uint16_t dac_init18_24;       // The initial DAC value for 24 fps at switchpos 18
    uint16_t dac_init18_25;       // The initial DAC value for 25 fps at switchpos 18

    uint16_t dac_init24_16;      // The initial DAC value for 16 2/3 fps at switchpos 24
    uint16_t dac_init24_18;      // The initial DAC value for 18 fps at switchpos 24
    uint16_t dac_init24_23;      // The initial DAC value for 23.976 fps at switchpos 24
    uint16_t dac_init24_24;      // The initial DAC value for 24 fps at switchpos 24
    uint16_t dac_init24_25;      // The initial DAC value for 25 fps at switchpos 24

    double pid_p;   // PID values for the projector type
    double pid_i;
    double pid_d; 
};
ProjectorConfig projector_config;

static const ProjectorConfig PROGMEM s_projector_configs[] = {
    /*
    This is a table of projector-specific values. To save memory, we memcopy just struct of values
    to memory when needed.
    */
    {"None", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // No projector connected
    {"Bauer T", 12, 4, 
        1915, 2130, 3000, 3060, 3235, // DAC speed values for switch at 18
        1295, 1450, 2180, 2190, 2320, // DAC speed values for switch at 24
        30, 50, 0}, // Bauer T610/T525/T510/T502
    {"Bauer P8", 2, 8, 
        1200, 1500, 2000, 2100, 2200,
        1200, 1500, 2000, 2100, 2200, 
        30, 50, 0}, // Bauer P8
    {"Selecton", 3, 8, 
        1200, 1500, 2000, 2100, 2200,
        1200, 1500, 2000, 2100, 2200,
        30, 50, 0}, // Bauer P8 Selecton
    {"Visacstc", 3, 4, 
        1200, 1500, 2000, 2100, 2200,
        1200, 1500, 2000, 2100, 2200, 
        30, 50, 0}, // Braun Visacustic
    {"Reserved", 0, 0, 
        0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0,
        30, 50, 0},                // No projector connected
};

ProjectorType detectProjector(int adcValue)
{
 /* Reference Values:
   0: None
 200: Bauer T
 390: P8
 586: Selecton
 792: Visacustic
1000: Reserve
*/
    if (adcValue < 100)
        return NONE;
    else if (adcValue < 295)
        return BAUER_T;
    else if (adcValue < 488)
        return P8;
    else if (adcValue < 689)
        return SELECTON;
    else if (adcValue < 908)
        return VISACUSTIC;
    else
        return RESERVE;
}


void setup()
{
    // Initialize buttons using the helper function
    initializeButton(leftButton, LEFT_BTTN_PIN);
    initializeButton(rightButton, RIGHT_BTTN_PIN);
    initializeButton(dropBackButton, DROP_BACK_BTTN_PIN);
    initializeButton(catchUpButton, CATCH_UP_BTTN_PIN);

    Serial.begin(115200);

    pinMode(SHAFT_PULSE_PIN, INPUT);
    pinMode(FPS_PULSE_OUT_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LEFT_BTTN_PIN, INPUT_PULLUP);
    pinMode(RIGHT_BTTN_PIN, INPUT_PULLUP);
    pinMode(DROP_BACK_BTTN_PIN, INPUT_PULLUP);
    pinMode(CATCH_UP_BTTN_PIN, INPUT_PULLUP);
    pinMode(AUDIOMODE_SWITCH_PIN, INPUT); // This is used to read the Follow/Guide switch
    pinMode(DAC_ENABLE_PIN, OUTPUT);
    pinMode(PROJ_ID_PIN, INPUT);

    // Let's read the projector board's voltage divider to detect the projector type.
    int adcValue = analogRead(PROJ_ID_PIN);
    ProjectorType proj = detectProjector(adcValue);

    // Copy the projector config for the detected type
    memcpy_P(&projector_config, &s_projector_configs[proj], sizeof(ProjectorConfig));

    Serial.print(F("A0: "));
    Serial.print(adcValue);
    Serial.print(F(" → "));
    Serial.println(projector_config.name);

    // TODO: Read the DAC init values from EEPROM if available, otherwise use the defaults
    myPID.SetTunings(projector_config.pid_p, projector_config.pid_i, projector_config.pid_d); // Set the PID tunings based on the projector config


    // Briefly configure Pin 7 as Input to detect a Lamp Relais Board
    pinMode(LAMP_RELAY_DETECT_PIN, INPUT);
    // Check if the Lamp Relais Board is connected
    if (digitalRead(LAMP_RELAY_DETECT_PIN) == HIGH)
    {
        Serial.println(F("No Lamp-Relais found. Disabling Follow Mode."));
    }
    else
    {
        Serial.println(F("** Lamp-Relais found!"));
        if (digitalRead(AUDIOMODE_SWITCH_PIN) == LOW)
        {
            Serial.println(F("Guide-Mode enabled. We will start the Music."));
        }
        else
        {
            Serial.println(F("Follow-Mode enabled. The Music will start us."));
        }
    }
    // Configure Pin 7 as OUTPUT again, allowing us to stop the projector
    pinMode(STOP_EN_PIN, OUTPUT);
    digitalWrite(STOP_EN_PIN, LOW); 

    // Use this for PID tuning with Pots
    // pinMode(P_PIN, INPUT);
    // pinMode(I_PIN, INPUT);
    // pinMode(D_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(SHAFT_PULSE_PIN), onShaftImpulseISR, RISING); // We only want one edge of the signal to not be duty cycle dependent
    dac.begin(0x60);

    dac.setVoltage(projector_config.dac_init18_18, false); // 18 fps and 24 fps share the same DAC init value

    // Init the PID
    pid_input = 0;
    pid_setpoint = 0;
    myPID.SetOutputLimits(0, 4095);
    myPID.SetSampleTime(100);
    myPID.SetMode(MANUAL);
    pid_output = projector_config.dac_init18_18; // This avoids starting with a 0-Output signal
    myPID.SetMode(AUTOMATIC);

    currently_selected_mode = FPS_AUTO;
    activateSpeedConfig(currently_selected_mode);

    u8x8.begin();
    u8x8.setFont(u8x8_font_profont29_2x3_n); // https://github.com/olikraus/u8g2/wiki/fntlist8x8
    drawCurrentTime(0, 0, true); // Init the Display

    // u8x8.setFont(u8x8_font_5x8_r);
    // u8x8.setCursor(3, 7);
    // u8x8.print("|''''|'O''|");

    // open_iconic_arrow_4x // O or S for arrows,
    // profont for scale? |''''|''''|''''|''''|
    // u8x8_font_7x14_1x2_n (small): (r is 900B more)
}

void loop()
{
    // Atomic reads for volatile variables
    long local_timer_pulses = 0;
    long local_shaft_pulses = 0;
    long current_pulse_difference = 0;

    // Copy volatile variables
    noInterrupts();
    local_timer_pulses = timer_pulses;
    local_shaft_pulses = shaft_pulses;
    interrupts();

    // Compute current pulse difference
    current_pulse_difference = local_timer_pulses - local_shaft_pulses;

    // Speed control logic (refactored)
    if (projector_state == PROJ_RUNNING)
    {
        controlSpeed(current_pulse_difference);
        checkForStop();
    }
    else if (projector_state == PROJ_IDLE)
    {
        checkProjectorRunningYet();
    }

    // Handle button actions
    leftButton.loop();
    rightButton.loop();
    dropBackButton.loop();
    catchUpButton.loop();

    // Update display
    drawCurrentTime(shaft_frames, speed.end_freq, false);
    drawCurrentMode();


}

void controlSpeed(long current_pulse_difference)
{
    static long last_pulse_difference = 0;
    static long last_dac_value = 1500;
    uint16_t new_dac_value = 0;
    long local_timer_pulses = timer_pulses;
    long local_shaft_pulses = shaft_pulses;

    // Feed PID and compute DAC output
    pid_input = current_pulse_difference;
    myPID.Compute();
    new_dac_value = pid_output;
    dac.setVoltage(new_dac_value, false);

    // Detect if error is more than half a frame and light the red LED
    if (current_pulse_difference < -6 || current_pulse_difference > 6)
    {
        digitalWrite(LED_RED_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED_RED_PIN, LOW);
    }

    // Debug output
    if (current_pulse_difference != last_pulse_difference)
    {
        if (millis() % 100 == 1)
        {
            Serial.print(F("Mode: "));
            Serial.print(speed.name);
            Serial.print(F(", Error: "));
            Serial.print(current_pulse_difference);
            Serial.print(F(", DAC: "));
            Serial.print(new_dac_value);
            Serial.print(F(", timer: "));
            Serial.print(local_timer_pulses);
            Serial.print(F(", shaft: "));
            Serial.print(local_shaft_pulses);
            Serial.print(F(", Enable: "));
            Serial.print(digitalRead(DAC_ENABLE_PIN));
            Serial.print(F(", Running: "));
            Serial.println(projector_state);
        }
    }

    // Update last values
    last_pulse_difference = current_pulse_difference;
    last_dac_value = new_dac_value;
}

void checkForStop()
{
    unsigned long current_pulse_difference = 0; // Adjust type if necessary
    unsigned long local_timer_pulses = 0;       // Adjust type if necessary
    unsigned long local_shaft_pulses = 0;       // Adjust type if necessary

    if (shaft_pulse_count_updated)              // This is set in the shaft ISR
        last_pulse_timestamp = micros();

    if (hasStoppedSince(last_pulse_timestamp, STOP_THRESHOLD))
    {
        projector_state = PROJ_IDLE; // Projector is stopped
        Serial.println(F("[DEBUG] Projector stopped."));
        stopTimer1();
        shaft_pulses = 0;
        shaft_frames = 0;
        timer_pulses = 0;

        current_pulse_difference = 0;
        local_timer_pulses = 0;
        local_shaft_pulses = 0;

        // Reset DAC and PID
        dac.setVoltage(projector_config.dac_init18_18, false); // Reset the DAC to compensate for wound-up break corrections
        myPID.SetMode(MANUAL);
        pid_output = projector_config.dac_init18_18; //TODO: Do we know better than just using 18?
        pid_input = 0;
        myPID.Compute();
        Serial.print(F("PID Reset to initial DAC value: "));
        Serial.println(pid_output);
        myPID.SetMode(AUTOMATIC);
        digitalWrite(DAC_ENABLE_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        currently_selected_mode = FPS_AUTO;
        activateSpeedConfig(FPS_AUTO);

        FreqMeasure.end();
    }
    else
    {
        // Reset the flag and be ready for another incoming pulse
        shaft_pulse_count_updated = false;
    }
}

void initializeButton(Button2 &button, byte pin)
{
    button.begin(pin);
    button.setTapHandler(handleButtonTap);
    button.setDoubleClickTime(0); // disable double clicks
    button.setDebounceTime(10);
}

bool hasStoppedSince(unsigned long start, unsigned long duration)
{
    return (micros() - start) > duration;
}

void checkProjectorRunningYet()
{
    if (!new_shaft_impulse_available)
        return; // Skip processing if no new data

    new_shaft_impulse_available = false; // Reset the ISR's "new data available" flag

    double freq_sum = 0;
    int freq_count = 0;
    unsigned long last_available_freqMeasure_time = micros(); // Track the last time data was available

    FreqMeasure.begin();

    // collect e.g. 48 samples (4 frames à 12 imps)
    while (freq_count < (projector_config.pulses_per_rotation * projector_config.framecount_until_stable)) 
    {
        if (FreqMeasure.available())
        {
            // Average several readings together
            freq_sum += FreqMeasure.read();
            freq_count++;
            /* Debug Code
            Serial.print(freq_count);
            Serial.print(": ");
            Serial.println(freq_sum);
            */

            last_available_freqMeasure_time = micros(); // Update the last available time
        }

        // Check for timeout
        if (micros() - last_available_freqMeasure_time > STOP_THRESHOLD)
        {
            Serial.println(F("Shaft Noise ignored."));
            break; // Exit the loop
        }
    }

    FreqMeasure.end();

    // Only process if a full set of data was collected
    if (freq_count >= 48) // TODO: use framecount_until_stable!
    {
        float detected_frequency = FreqMeasure.countToFrequency(freq_sum / freq_count);
        Serial.print(F("Detected freq (FreqMeasure): "));
        Serial.println(detected_frequency / projector_config.pulses_per_rotation, 6);

        if (currently_selected_mode == FPS_AUTO)
        { // Determine the mode based on the detected frequency
            uint8_t target_mode = (detected_frequency <= 21 * projector_config.pulses_per_rotation ? FPS_18 : FPS_24);
            projector_speed_switch = (target_mode == FPS_18 ? 18 : 24);
            activateSpeedConfig(target_mode);
            currently_selected_mode = (target_mode);
        }
        // Init the pulse timer to not lose those first frames
        timer_pulses = freq_count;

        // Update projector state and init Timer1
        projector_start_millis = millis();  // we lose the time for 4 frames here
        projector_state = PROJ_RUNNING;
        Serial.print(F("Setting up Timer for "));
        Serial.println(speed.name);
        startTimer1();
        digitalWrite(DAC_ENABLE_PIN, HIGH);
    }

    // Reset variables
    freq_sum = 0;
    freq_count = 0;
    
}


void selectNextMode(Button2 &btn)
{
    int8_t change = (btn == leftButton) ? -1 : 1;

    // Determine if left or right button was pressed
    currently_selected_mode = (currently_selected_mode + change + MODES_COUNT) % MODES_COUNT;
    // Skip mode FPS_AUTO if projector is running, Auto Mode is only selctable when idling
    if (projector_state == PROJ_RUNNING && currently_selected_mode == FPS_AUTO)
    {
        // select next adjacent mode
        currently_selected_mode = (currently_selected_mode + change + MODES_COUNT) % MODES_COUNT;
    }
    Serial.print(F("New Mode: "));
    Serial.println(currently_selected_mode);

    activateSpeedConfig(currently_selected_mode); 

    // write the timerpulses (+timer_correction_factor)
    noInterrupts();
    // timer_pulses = timer_pulses * timer_correction_factor;
    interrupts();
}

void activateSpeedConfig(byte next_speed)
{
    float previous_avg_freq;
    
    // Copy next config from PROGMEM to struct
    memcpy_P(&speed, &s_speed_table[next_speed], sizeof(SpeedConfig));

    // To keep the sync Sepmag-style, we need to correct the timer to teh new speed
    float timer_correction_factor;
    if (projector_state == PROJ_RUNNING)
    {
        // work with ints as long as possible
        uint32_t now = millis();
        uint32_t shaft_frames_now = shaft_frames;
        unsigned long elapsed_time_ms = now - projector_start_millis;
        unsigned long frames_per_1000_ms = (unsigned long)shaft_frames_now * 1000; // inflate shaft_frames to improve acuracy
        float previous_avg_freq = (float)frames_per_1000_ms / elapsed_time_ms;

        Serial.print(F("Frames * 1000 so far: "));
        Serial.print(frames_per_1000_ms);
        Serial.print(F(", seconds running: "));
        Serial.println((float)elapsed_time_ms / 1000, 6);

        Serial.print(F("   -> Calcualted prev. Freq: "));
        Serial.println(previous_avg_freq, 6);

        timer_correction_factor = speed.end_freq / previous_avg_freq;

        Serial.print(F("Timer Factor: "));
        Serial.print(speed.end_freq, 6);
        Serial.print(F(" / "));
        Serial.print(F("before: "));
        Serial.print(previous_avg_freq, 6);
        Serial.print(F(" = "));
        Serial.println(timer_correction_factor, 6);
    }
    else
    {
        timer_correction_factor = 1;
    }

    Serial.print(F("Mode Name: "));
    Serial.println(speed.name);

    // Init the PID with a start value to not start at 0
    myPID.SetMode(MANUAL);
    // Use the correct DAC init value for the selected speed
    uint16_t dac_init = 0;
    Serial.print(F("Projector Speed Switch: "));
    Serial.println(projector_speed_switch);

    if (projector_speed_switch == 18)
    {
        // Use the DAC init values for switch position 18
        switch (next_speed) {
            case FPS_16_2_3:  dac_init = projector_config.dac_init18_16;    break;
            case FPS_18:      dac_init = projector_config.dac_init18_18;    break;
            case FPS_23_976:  dac_init = projector_config.dac_init18_23;    break;
            case FPS_24:      dac_init = projector_config.dac_init18_24;    break;
            case FPS_25:      dac_init = projector_config.dac_init18_25;    break;
            default:          dac_init = projector_config.dac_init18_18;    break;
        }
    }
    else
    {
        // Use the DAC init values for switch position 24
        switch (next_speed) {
            case FPS_16_2_3:  dac_init = projector_config.dac_init24_16;    break;
            case FPS_18:      dac_init = projector_config.dac_init24_18;    break;
            case FPS_23_976:  dac_init = projector_config.dac_init24_23;    break;
            case FPS_24:      dac_init = projector_config.dac_init24_24;    break;
            case FPS_25:      dac_init = projector_config.dac_init24_25;    break;
            default:          dac_init = projector_config.dac_init24_18;    break;
        }
    }
    pid_output = dac_init;
    myPID.Compute();
    myPID.SetMode(AUTOMATIC);

    // Configure DAC
    dac.setVoltage(dac_init, false);      // false: Do not make this the default value of the DAC
    digitalWrite(DAC_ENABLE_PIN, speed.dac_enable); // only 0 for NONE mode. Todo: Shave of 6 Bytes by checking for it's name
}

void handleButtonTap(Button2 &btn)
{
    if (btn == leftButton)
    {
        selectNextMode(btn);
    }
    else if (btn == rightButton)
    {
        selectNextMode(btn);
    }
    // Frame up/down is only allowed while the projector is running
    if (projector_state == PROJ_RUNNING)
    {
        if (btn == dropBackButton)
        {
            noInterrupts();
            shaft_pulses += projector_config.pulses_per_rotation; // Add one full rotation
            interrupts();
        }
        else if (btn == catchUpButton)
        {
            noInterrupts();
            shaft_pulses -= projector_config.pulses_per_rotation; // Subtract one full rotation
            interrupts();
        }
    }
}

void onShaftImpulseISR()
{
    // Expose the news
    shaft_pulses++;
    
    shaft_pulse_count_updated = true;
    new_shaft_impulse_available = true;

    if (shaft_modulus == 0)
    {
        shaft_frames++;
    }
    shaft_modulus++;
    shaft_modulus %= (projector_config.pulses_per_rotation);
}

ISR(TIMER1_COMPA_vect)
{
    // Increment the modulus counter
    timer_modulus++;

    // Check if the counter has reached the desired factor
    if (timer_modulus >= speed.timer_factor)
    {
        timer_pulses++;    // Increment timer_pulses
        timer_modulus = 0; // Reset the counter
        // PIND |= (1 << 7); // Toggle pin 7 (I guess this was used for debugging. It now conflicts with the STOP_EN_PIN)
    }
    // Dither logic (fixed-point accumulator)
    dither_accumulator_32 += speed.dither_frac32;

    // Check for accumulator overflow (dithering decision)
    OCR1A = (dither_accumulator_32 < speed.dither_frac32)
                ? (speed.dither_base + 1)
                : speed.dither_base;
}

void startTimer1()
{
    dither_accumulator_32 = 0;

    // Setup and Start Timer1
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    TCNT1 = 0;
    OCR1A = speed.dither_base;            // OCR1A start value
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC-Mode (WGM12=1), Prescaler=8 => CS11=1
    TIMSK1 |= (1 << OCIE1A);              // enable Compare-A-Interrupt
    interrupts();

    // Serial.print(F("*** Timer 1 prepared for "));
    // Serial.println(speed.end_freq);

}

void stopTimer1()
{ 
    // Stops Timer1, for when we are not in crystal mode
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}

void drawCurrentMode()
{
    static uint8_t prev_selected_mode = 99;
    if (currently_selected_mode != prev_selected_mode)
    {
        prev_selected_mode = currently_selected_mode;

        u8x8.setFont(u8x8_font_inr21_2x4_r/*u8x8_font_profont29_2x3_r*/);
        u8x8.setCursor(0, 0);
        printCentered(u8x8, speed.name, 8);
    }
}

void drawCurrentTime(int32_t frame_count, float fps_rn, bool force_redraw)
{
    static uint32_t prev_shaft_frames = -1; // need to differ from actual frames early to allow an initial call

    uint8_t hours = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    uint8_t current_sub_frame = 0;
    uint32_t current_film_second = 0;
    
    uint8_t right_sec_digit = 0;
    uint8_t left_sec_digit = 0;
    uint8_t right_min_digit = 0;
    uint8_t leftMinDigit = 0;
    uint8_t hour_digit = 0;
    bool sign = false;

    uint8_t prev_right_sec_digit = 99;
    uint8_t prev_left_sec_digit = 99;
    uint8_t prev_right_min_digit = 99;
    uint8_t prev_left_min_digit = 99;
    uint8_t prev_hour_digit = 99;
    bool prev_sign = false;

    if (shaft_frames != prev_shaft_frames) {
        prev_shaft_frames = shaft_frames;

        u8x8.setFont(u8x8_font_inr21_2x4_r /*u8x8_font_courR18_2x3_n*/);

        if (fps_rn != 50 / 3.0)
        {
            current_sub_frame = frame_count % int(fps_rn);
            current_film_second = abs(frame_count) / fps_rn;
        }
        else
        { // for 16 2/3 fps 
            current_sub_frame = frame_count % 50;
            current_film_second = (abs(frame_count) - current_sub_frame) * 3 / 50;
            if (frame_count % 50 >= 16)
            {
                current_sub_frame = (current_sub_frame - 16) % 17;
                current_film_second++;
                if (frame_count % 50 > 32)
                    current_film_second++;
            }
        }

        hours = numberOfHours(current_film_second);
        minutes = numberOfMinutes(current_film_second);
        seconds = numberOfSeconds(current_film_second);
        right_sec_digit = seconds % 10;

        // Handle negative frame counts and time nicely
        //
        if (right_sec_digit == 0 || force_redraw)
        {
            if (frame_count < 0)
                sign = true;
            else
                sign = false;
            if (sign != prev_sign || force_redraw)
            {
                force_redraw = true;
                prev_sign = sign;
                u8x8.setCursor(((sign) ? 4 : 2), 4);
                u8x8.print(F(":  :  -")); // when tc is negative, do not render sub frame count, but a leading minus sign
                if (!sign && fps_rn == 9)
                {
                    u8x8.drawTile(15, 3, 1, emptyTile);
                    u8x8.drawTile(15, 4, 1, emptyTile);
                }
            }
        }

        // Only paint the glyphs that have changed, this improves the display framerate a lot
        // Precompute offset for sign to avoid multiple ternary evaluations

        uint8_t sign_offset = sign ? 2 : 0;
        updateDigit(right_sec_digit, prev_right_sec_digit, 12, sign_offset, force_redraw);
        updateDigit(seconds / 10, prev_left_sec_digit, 10, sign_offset, force_redraw);
        updateDigit(minutes % 10, prev_right_min_digit, 6, sign_offset, force_redraw);
        updateDigit(minutes / 10, prev_left_min_digit, 4, sign_offset, force_redraw);
        updateDigit(hours % 10, prev_hour_digit, 0, sign_offset, force_redraw);

        // Print current Subframe, SMPTE style
        //
        if (frame_count >= 0)
        {
            u8x8.setFont(u8x8_font_7x14_1x2_n);    // just numbers
            // u8x8.setFont(u8x8_font_7x14_1x2_r); // full charset
            u8x8.setCursor(14, 4);
            if (current_sub_frame < 10 && fps_rn != 9)
                u8x8.print(0);
            u8x8.print(current_sub_frame);
        }
    }
}

void printCentered(U8X8 &u8x8, const char *text, uint8_t lineWidth)
{
    size_t textLength = strlen(text); // Get the length of the text
    if (textLength >= lineWidth)
    {
        u8x8.print(text); // If the text is longer than the line, print as is
        return;
    }

    uint8_t padding = (lineWidth - textLength) / 2; // Calculate the padding
    uint8_t extraSpace = (lineWidth - textLength) % 2; // Handle uneven padding
    
    for (uint8_t i = 0; i < padding; i++)
    {
        u8x8.print(' '); // Add spaces before the text
    }
    u8x8.print(text); // Print the text
    // Add spaces after the text
    for (uint8_t i = 0; i < padding + extraSpace; i++)
    {
        u8x8.print(' ');
    }
}

void updateDigit(uint8_t new_digit, uint8_t &prev_digit, uint8_t cursor_pos, uint8_t sign_offset, bool force_redraw)
{
    if (new_digit != prev_digit || force_redraw)
    {
        prev_digit = new_digit;
        u8x8.setCursor(cursor_pos + sign_offset, 4);
        u8x8.print(new_digit);
    }
}