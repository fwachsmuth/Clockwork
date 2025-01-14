/* Todo

Bugs:
- Starting in NONE does not work yet, DAC stays connected

Todo: 
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
- Clamp the PID output to avoid halting the motor
- ESS in

Hardware:
- add IR emitter to start a device
- 74LVC2G14 is smaller Schmitt Trigger
- Reconfigure OpAmp offset to allow tha DAC to halt the motor
- Strobe Output to check other projector's speed
- ESS Out

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
constexpr size_t SHAFT_SEGMENT_COUNT = 12;             // Size of the Median Window. We use 12 to capture one entire shaft revolution


// pins and consts
const byte SHAFT_PULSE_PIN = 2;
const byte LED_RED_PIN = 5; //  Out of Sync
const byte OSCILLOSCOPE_PIN = 7; //  Crystal enabled
const byte ENABLE_PIN = 9;

// Use this for PID tuning with Pots
// const byte P_PIN = A6;
// const byte I_PIN = A7;
// const byte D_PIN = A3;

const byte LEFT_BTTN_PIN = 10;
const byte RIGHT_BTTN_PIN = 13;
const byte DROP_BACK_BTTN_PIN = 11;
const byte CATCH_UP_BTTN_PIN = 12;

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

// --- some custome gfx fro the display 
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

// PID stuff
double pid_setpoint, pid_input, pid_output;
double pid_Kp = 30, pid_Ki = 50, pid_Kd = 0; // old PID values for frame based controlling

// PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);

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
#define FPS_AUTO    1
#define FPS_16_2_3  2   // => 200 Hz
#define FPS_18      3   // => 216 Hz
#define FPS_23_976  4   // => ~287.712 Hz
#define FPS_24      5   // => 288 Hz
#define FPS_25      6   // => 300 Hz
#define MODES_COUNT 7   // to know where to roll over in the menu

// Struct with all the information we need to configure the controller for a speed:
struct SpeedConfig
{
    /*

    The dither pair is used to approximate an impossible speed that the Timer1 could not precisely generate at 16 MHz.
    They are all claculated for prescaler 8 (2 MHz) to achieve high precision at manageable CPU load.
    The values are tailored for “fps * 12” = final frequency, since the Bauer's shaft has 12 segments per revolution.
    Explanation:
        - base = floor( (2000000 / (target freq)) ) or floor( (2000000 / (a multiple)) )
        - frac32 = (decimal point * 2^32) (UQ0.32 fixed decimal point)
        - timerFactor => Software divider in the ISR, some multiples of the target frq give higher precision.

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
    uint16_t dac_init;      // approx dac values to start with
};
SpeedConfig speed;

static const SpeedConfig PROGMEM s_speed_table[] = {
/*
This is a table of values needed to generate a certain speed. To save memory, copy one just struct of values
to memory when needed.
*/
    {"Off", 2314, 0xD0BE9C00, 3, 0, 0, 0}, /* Need dummy values here to not f up the timer. */
    {"Auto", 0, 0, 0, 0, 1, 1500},
    {"16  fps", 10000, 0x00000000, 1, 16.666666, 1, 1200},
    {"18 fps", 2314, 0xD0BE9C00, 4, 18.000000, 1, 1500},
    {"23.976", 6951, 0x638E38E4, 1, 23.976024, 1, 2000},
    {"24 fps", 2314, 0xD0BE9C00, 3, 24.000000, 1, 2100},
    {"25 fps", 6666, 0xAAAAAAAB, 1, 25.000000, 1, 2200}};

void setup()
{
    // Initialize buttons using the helper function
    initializeButton(leftButton, LEFT_BTTN_PIN);
    initializeButton(rightButton, RIGHT_BTTN_PIN);
    initializeButton(dropBackButton, DROP_BACK_BTTN_PIN);
    initializeButton(catchUpButton, CATCH_UP_BTTN_PIN);

    Serial.begin(115200);

    pinMode(SHAFT_PULSE_PIN, INPUT);
    pinMode(OSCILLOSCOPE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(LEFT_BTTN_PIN, INPUT_PULLUP);
    pinMode(RIGHT_BTTN_PIN, INPUT_PULLUP);
    pinMode(DROP_BACK_BTTN_PIN, INPUT_PULLUP);
    pinMode(CATCH_UP_BTTN_PIN, INPUT_PULLUP);

    // Use this for PID tuning with Pots
    // pinMode(P_PIN, INPUT);
    // pinMode(I_PIN, INPUT);
    // pinMode(D_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(SHAFT_PULSE_PIN), onShaftImpulseISR, RISING); // We only want one edge of the signal to not be duty cycle dependent
    dac.begin(0x60);

    dac.setVoltage(speed.dac_init, false); // 1537 is petty much 18 fps and 24 fps

    // Init the PID
    pid_input = 0;
    pid_setpoint = 0;
    myPID.SetOutputLimits(0, 4095);
    myPID.SetSampleTime(100);
    myPID.SetMode(MANUAL);
    pid_output = speed.dac_init; // This avoids starting with a 0-Output signal
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
            Serial.print(digitalRead(ENABLE_PIN));
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
        dac.setVoltage(speed.dac_init, false); // Reset the DAC to compensate for wound-up break corrections
        myPID.SetMode(MANUAL);
        pid_output = speed.dac_init;
        pid_input = 0;
        myPID.Compute();
        Serial.print(F("PID Reset to initial DAC value: "));
        Serial.println(pid_output);
        myPID.SetMode(AUTOMATIC);
        digitalWrite(ENABLE_PIN, LOW);
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

void unused_measureFrequency()
{
    static double sum = 0;
    static int count = 48;
    
    if (FreqMeasure.available())
    {
        // average several reading together
        sum += FreqMeasure.read();
        count++;
        if (count > 48) // 48 impulses = 4 frames
        {
            float frequency = FreqMeasure.countToFrequency(sum / count);

            // Buffer to hold the formatted string
            char buffer[20];
            // Convert float to string with 5 decimal places
            dtostrf(frequency / 12, 8, 5, buffer); // (value, width, precision, buffer)

            Serial.println(buffer);
            u8x8.drawString(0, 1, buffer);
            sum = 0;
            count = 0;
        }
    }
    // FreqMeasure.end();
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

    while (freq_count <= 47)    // collect 48 samples (4 frames)
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
    if (freq_count >= 48)
    {
        float detected_frequency = FreqMeasure.countToFrequency(freq_sum / freq_count);
        Serial.print(F("Detected freq (FreqMeasure): "));
        Serial.println(detected_frequency / SHAFT_SEGMENT_COUNT);

        if (currently_selected_mode == FPS_AUTO)
        { // Determine the mode based on the detected frequency
            uint8_t target_mode = (detected_frequency <= 21 * SHAFT_SEGMENT_COUNT ? FPS_18 : FPS_24);
            activateSpeedConfig(target_mode);
            projector_speed_switch = (target_mode == FPS_18 ? 18 : 24);
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
        digitalWrite(ENABLE_PIN, HIGH);
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
    pid_output = speed.dac_init;
    myPID.Compute();
    myPID.SetMode(AUTOMATIC);

    // Configure DAC
    dac.setVoltage(speed.dac_init, false);      // false: Do not make this the default value of the DAC
    digitalWrite(ENABLE_PIN, speed.dac_enable); // only 0 for NONE mode. Todo: Shave of 6 Bytes by checking for it's name
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
            shaft_pulses += SHAFT_SEGMENT_COUNT;
            interrupts();
        }
        else if (btn == catchUpButton)
        {
            noInterrupts();
            shaft_pulses -= SHAFT_SEGMENT_COUNT;
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
    shaft_modulus %= (SHAFT_SEGMENT_COUNT);
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
        PIND |= (1 << 7); // Toggle pin 7kica
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
                Serial.println(F("Force Redraw."));
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

/*
uint8_t sign_offset = sign ? 2 : 0;

void updateDigit(uint8_t new_digit, uint8_t &prev_digit, uint8_t cursor_pos) {
    if (new_digit != prev_digit || force_redraw) {
        prev_digit = new_digit;
        u8x8.setCursor(cursor_pos + sign_offset, 4);
        u8x8.print(new_digit);
    }
}

// Update seconds digits
updateDigit(right_sec_digit, prev_right_sec_digit, 12);
updateDigit(seconds / 10, prev_left_sec_digit, 10);

// Update minutes digits
updateDigit(minutes % 10, prev_right_min_digit, 6);
updateDigit(minutes / 10, prev_left_min_digit, 4);

// Update hour digit
updateDigit(hours % 10, prev_hour_digit, 0);
*/

        uint8_t sign_offset = sign ? 2 : 0;
        updateDigit(right_sec_digit, prev_right_sec_digit, 12, sign_offset, force_redraw);
        updateDigit(seconds / 10, prev_left_sec_digit, 10, sign_offset, force_redraw);
        updateDigit(minutes % 10, prev_right_min_digit, 6, sign_offset, force_redraw);
        updateDigit(minutes / 10, prev_left_min_digit, 4, sign_offset, force_redraw);
        updateDigit(hours % 10, prev_hour_digit, 0, sign_offset, force_redraw);


        /*
                uint8_t sign_offset = sign ? 2 : 0;

                // Check if we need to update right seconds digit
                if (right_sec_digit != prev_right_sec_digit || force_redraw)
                {
                    prev_right_sec_digit = right_sec_digit;
                    u8x8.setCursor(12 + sign_offset, 4);
                    u8x8.print(right_sec_digit);
                }

                // Compute left seconds digit
                uint8_t new_left_sec_digit = seconds / 10;
                if (new_left_sec_digit != prev_left_sec_digit || force_redraw)
                {
                    prev_left_sec_digit = new_left_sec_digit;
                    u8x8.setCursor(10 + sign_offset, 4);
                    u8x8.print(new_left_sec_digit);
                }

                // Check if we need to update right minutes digit
                uint8_t new_right_min_digit = minutes % 10;
                if (new_right_min_digit != prev_right_min_digit || force_redraw)
                {
                    prev_right_min_digit = new_right_min_digit;
                    u8x8.setCursor(6 + sign_offset, 4);
                    u8x8.print(new_right_min_digit);
                }

                // Compute left minutes digit
                uint8_t new_left_min_digit = minutes / 10;
                if (new_left_min_digit != prev_left_min_digit || force_redraw)
                {
                    prev_left_min_digit = new_left_min_digit;
                    u8x8.setCursor(4 + sign_offset, 4);
                    u8x8.print(new_left_min_digit);
                }

                // Check if we need to update hour digit
                uint8_t new_hour_digit = hours % 10;
                if (new_hour_digit != prev_hour_digit || force_redraw)
                {
                    prev_hour_digit = new_hour_digit;
                    u8x8.setCursor(0 + sign_offset, 4);
                    u8x8.print(new_hour_digit);
                }
                */

        // if (frame_count != 0)
        //     u8x8.setCursor(16 - (int(log10(abs(frame_count)) + (sign ? 3 : 2)) << 1), 0); 
        // else
        //     u8x8.setCursor(12, 0);
        // u8x8.print(" ");
        // u8x8.print(frame_count);

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