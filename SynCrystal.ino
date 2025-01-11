/* Todo

- No spped mode activate between 0->1 and 6->0, since the AUTO switch case has no code yet!

- refactor activateNextMode() (If instead of switch)
- remove unused params in struct: auto_mode, and potentially dac_mode

- use speed.timer_factor
- use speed.dac_enable
- use speed.auto_mode
- use speed.dac_init
- update comments in the new struct
- flash the DAC just once on very first start


- New Bugs:
- Remaining pid-Errors seem to add up on speed changes?
    Mode: AUTO, Error: 1, DAC: 1415
    [DEBUG] Projector stopped.
    PID Reset to initial DAC value: 1500.00
    Shaft Noise ignored.
    timer_pulses * next_freq / (millis() - proojector_start_millis) / 1000))
->  From 0 Pulses to 0 * 16.67 / (1/12) / (2016)/1000) = nan
->  My calculation: 0 <-
    XTAL_16_2_3
- updated_time_pulses not written back yet since they cause a "stop" deetcted !?
- Not sure if entering Auto/None works correctly... pretzel brain
- Changing form 25 fps back to Auto does not retain a previous 18fps-Auto
- Decrement Shaft Impulses after detected shaft noise!


Pending Bugs for Sepmag Sync:
- Changing form 25 fps back to Auto does not retain a previous 18fps-Auto
- rundetection does not kick in when starting after manual mode selection


Hardware
- add IR emitter
- 74LVC2G14
- Strobe Output
- Reconfigure OpAmp offset to allow tha DAC to halt the motor

Code
- Remove dead vars
- Move controller and stop detection from loop into functions
    - B:  Forget any previous errors and start fresh
        + allws for quick speed changes
        + ideal mode for use as "Peaceman's Box" (ESS out mode)
        + good for telecine (which could also just restartm though)
        - loses sync with any sepmag audio
    Timecode would restart at 0:00:00.00
- Use the display
- Review volatile vars (only for variables modified inside ISRs and used outside)
- Reset Counters? (long press both buttons?)
- Rangieren (langsam)
- "start the audio" IR
- Clamp the PID output to avoid halting the motor

Irgendwann
- Fernstart/stop support
- ESS support
- Add Realtime Timecode

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
const unsigned long SAVE_THRESHOLD = 10000; // ms until a stable DAC value will be considered as new EPROM default
constexpr size_t SHAFT_SEGMENT_COUNT = 12;             // Size of the Median Window. We use 12 to capture one entire shaft revolution


// pins and consts
const byte SHAFT_PULSE_PIN = 2;
const byte LED_RED_PIN = 5;
const byte LED_GREEN_PIN = 7;
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

const byte ledRed = 5;          //  Out of Sync
const byte ledGreen = 7;        //  Crystal enabled
const byte ledSlowerYellow = 6; //  -
const byte ledFasterYellow = 8; //  +

// Timer Variables
volatile uint32_t timer_pulses = 0;
volatile uint32_t timer_frames = 0; // This is the timer1 (frequency / timer_factor) — equalling actual desired fps (no multiples)
volatile uint8_t timer_modulus = 0; // For Modulo in the ISR, to compensate the timer_factor
// int timer_factor = 0;           // this is used for the Timer1 "postscaler", since multiples of 18 and 24 Hz give better accuracy
volatile uint32_t last_pulse_timestamp; // Timestamp of the last pulse, used to detect a stop
volatile uint32_t dither_accumulator_32 = 0; // Accumulator for Timer Dithering


// Shaft Encoder Variables
volatile uint32_t shaft_pulses = 0;
volatile uint32_t shaft_frames = 0;   // This is the actually advanced frames (pulses / shaft_segment_disc_divider)
volatile uint8_t shaft_modulus = 0;            // For Modulo in the ISR, to compensate the multiple pulses per revolution
volatile bool new_shaft_impulse_available = false;
uint32_t projector_start_millis = 0; // To track the start time of the projector running
volatile uint32_t last_frame_timestamp;      // Timestamp of the last pulse, used to detect a stop


uint8_t projector_speed_switch = 0;  // To track the detected position of the projector's speed switch (18 or 24)

// flags to assure reading only once both ISRs have done their duty
volatile bool shaft_frame_count_updated;
volatile bool shaft_pulse_count_updated;
volatile bool timer_frame_count_updated;
volatile bool timer_pulse_count_updated;

// PID stuff
double pid_setpoint, pid_input, pid_output;
double pid_Kp = 30, pid_Ki = 50, pid_Kd = 0; // old PID values for frame based controlling

// Adaptive PID — not worth it.
// double cons_Kp = 20, cons_Ki = 8, cons_Kd = 0; // old PID values for frame based controlling
// double agg_Kp = 50, agg_Ki = 10, agg_Kd = 0; // old PID values for frame based controlling

// PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);
PID myPID(&pid_input, &pid_output, &pid_setpoint, pid_Kp, pid_Ki, pid_Kd, REVERSE);

// Instantiate the DAC
Adafruit_MCP4725 dac;

// Instantiate the Buttons
Button2 leftButton, rightButton, dropBackButton, catchUpButton;

// Instantiate the Display
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

byte currently_selected_mode;
float previous_freq = 1.00; // to allow recalculation of timer_frames and timecode

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
    char name[6];           // short name for debugging
    volatile uint16_t dither_base;   // Grundwert für OCR1A
    volatile uint32_t dither_frac32; // fractional part (UQ0.32), 0..(2^32-1)
    uint8_t timer_factor;   // a frequency multiplier for each timer config (some multiples give better accuracy)
    float end_freq;         // the actual frequency we get with this config as an approx. float
    bool dac_enable;        // to allow freewheeling (no speed controller impact)
    uint16_t dac_init;      // approx dac values to start with
};
SpeedConfig speed;

static const SpeedConfig PROGMEM s_speed_table[] = {
    {"NONE", 0, 0, 0, 0, 0, 0},
    {"AUTO", 0, 0, 0, 0, 1, 1500},
    {"16.66", 10000, 0x00000000, 1, 16.666666, 1, 1200},
    {"18.00", 2314, 0xD0BE9C00, 4, 18.000000, 1, 1500},
    {"23.98", 6951, 0x638E38E4, 1, 23.976024, 1, 2000},
    {"24.00", 2314, 0xD0BE9C00, 3, 24.000000, 1, 2100},
    {"25.00", 6666, 0xAAAAAAAB, 1, 25.000000, 1, 2200}   
};
float previous_avg_freq;

/* old struct

 static const DitherConfig PROGMEM s_ditherTable[] = {

        
        ---------------------------------------------------------------------------
         Table with Base/Frac/Factor values for each target frequency
           Prescaler=8 => Timer frequency = 2 MHz
           The values are tailored for “fps * 12” = final frequency.

           Explanation:
             - base = floor( (2000000 / (target freq)) ) or floor( (2000000 / (a multiple)) )
             - frac32 = (decimal point * 2^32) (UQ0.32 fixed decimal point)
             - timerFactor => Software divider in the ISR

           Examples:
             * FPS_9 => 108 Hz = 864 / 8 => Timer-ISR=864 Hz => base=2314.8 => 2314 + frac~0.8148 => frac32~0xD0E14710
             * FPS_16_2_3 => 200 Hz => exact divider=10,000 => OCR1A=9999 => fraction=0 => no dither
             * etc.

         Note: All frac32 values here rounded to ~ <5 ppm.
        ---------------------------------------------------------------------------
        

        // 1) FPS_9 => 108 Hz = 864 Hz ISR / 8
        //    => idealDiv = 2314.8148148..., base=2314, fraction=~0.8148148
        //    => frac32 = round(0.8148148 * 2^32) = 0xD0BE9C00
        //    => Endfreq ~ 108.000000 => 0 ppm
        // {2314, 0xD0BE9C00, 8, 9.000000},

        // 2) FPS_16_2_3 => 200 Hz => idealDiv=10000 => fraction=0 => no dithering
        //    => base=9999, frac32=0
        //    => Endfreq=200 => 0 ppm
        {10000, 0x00000000, 1, 16.666666},

        // 3) FPS_18 => 216 Hz = 864 Hz ISR / 4
        //    => same as 108 Hz example but factor=4 => 0 ppm
        {2314, 0xD0BE9C00, 4, 18.000000},

        // 4) FPS_23_976
        /*  => fraction = 288000/1001 ~ 287.712287712288 Hz
        timer_factor= 1
        best base   = 6951
        best frac32 = 0x638E38E4  (decimal 1670265060)
        freqActual  = 2147483648000000/7463996984889 ~ 287.712287712283 Hz
        error Hz    = -0.000000000004
        ppm error   = -0.000000
        
        {6951, 0x638E38E4, 1, 23.976024},

        // 5) FPS_24 => 288 Hz = 864 Hz ISR / 3
        //    => same base/fraction as 108 Hz, factor=3 => 0 ppm
        {2314, 0xD0BE9C00, 3, 24.000000},

        // 6) FPS_25 => 300 Hz => idealDiv=6666.666..., fraction=0.666..., frac32=0xAAAAAAAB => 0 ppm
        {6666, 0xAAAAAAAB, 1, 25.000000},
    };
*/

    void setup()
    {
        // Initialize buttons using the helper function
        initializeButton(leftButton, LEFT_BTTN_PIN);
        initializeButton(rightButton, RIGHT_BTTN_PIN);
        initializeButton(dropBackButton, DROP_BACK_BTTN_PIN);
        initializeButton(catchUpButton, CATCH_UP_BTTN_PIN);

        Serial.begin(115200);

        pinMode(SHAFT_PULSE_PIN, INPUT);
        pinMode(LED_GREEN_PIN, OUTPUT);
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

        // FreqMeasure.begin();
}

void loop()
{
    static long local_timer_frames = 0;    // for atomic reads
    static long local_shaft_frames = 0;    // for atomic reads
    static long local_timer_pulses = 0;    // for atomic reads
    static long local_shaft_pulses = 0;    // for atomic reads
    static long last_pulse_difference = 0; // Stores the last output difference. (Just used to limit the printf output)

    static long last_dac_value = 1500; // also just used to limit the printf output

    static long current_frame_difference = 0;
    static long current_pulse_difference = 0;
    uint16_t new_dac_value = 0;
    static uint8_t button, last_button = BTTN_NONE;
    static long last_pid_update_millis;
    static long current_pid_update_millis;

    /* Use the below to tune the PID with Pots connected to Analog in

    // Read the potentiometer values
    uint16_t p_pot = analogRead(P_PIN) >> 1;
    // delayMicroseconds(10); // Short delay (adjust if needed)
    uint16_t i_pot = analogRead(I_PIN) >> 1;
    // delayMicroseconds(10); // Short delay (adjust if needed)
    uint16_t d_pot = analogRead(D_PIN) >> 6;
    // delayMicroseconds(10); // Short delay (adjust if needed)

    // Variables to track previous values
    static uint16_t last_p_pot = 0;
    static uint16_t last_i_pot = 0;
    static uint16_t last_d_pot = 0;

    // Check for changes and print if any of the values have changed
    if (p_pot != last_p_pot || i_pot != last_i_pot || d_pot != last_d_pot)
    {
        Serial.print("P: ");
        Serial.print(p_pot);
        Serial.print("   I: ");
        Serial.print(i_pot);
        Serial.print("   D: ");
        Serial.println(d_pot);

        // Update the last values
        last_p_pot = p_pot;
        last_i_pot = i_pot;
        last_d_pot = d_pot;
    
        // Update the PID with new values
        myPID.SetTunings((double)p_pot, (double)i_pot, (double)d_pot);
    }
    */

    // Poll the buttons
    leftButton.loop();
    rightButton.loop();
    dropBackButton.loop();
    catchUpButton.loop();

    if (projector_state == PROJ_IDLE)
    {
        checkProjectorRunningYet();
    }
    else if (projector_state == PROJ_RUNNING)
    {
        noInterrupts();
        local_timer_pulses = timer_pulses;
        local_shaft_pulses = shaft_pulses;
        interrupts();

        // Compute Error and feed PID and DAC
        current_pulse_difference = local_timer_pulses - local_shaft_pulses;

        // Disabled: Adaptive PID
        // double gap = abs(pid_setpoint - pid_input); // distance away from setpoint
        // if (gap < 10)
        // { // we're close to setpoint, use conservative tuning parameters
        //     myPID.SetTunings(cons_Kp, cons_Ki, cons_Kd);
        // }
        // else
        // {
        //     // we're far from setpoint, use aggressive tuning parameters
        //     myPID.SetTunings(agg_Kp, agg_Ki, agg_Kd);
        // }

        pid_input = current_pulse_difference;
        myPID.Compute();
        new_dac_value = pid_output;
        dac.setVoltage(new_dac_value, false);

        // Detect if we have are > half a frame off and light the red LED
        if (current_pulse_difference < -6 || current_pulse_difference > 6)
        {
            digitalWrite(LED_RED_PIN, HIGH);
        }
        else
        {
            digitalWrite(LED_RED_PIN, LOW);
        }

        // Debug output
     // if (new_dac_value != last_dac_value)
        if (current_pulse_difference != last_pulse_difference)
        {
            // Uncomment to throttle the Console Output
            if (millis() % 10000 == 1) {
                Serial.print(F("Mode: "));
                Serial.print(speed.name);
                Serial.print(F(", Error: "));
                Serial.print(current_pulse_difference);
                Serial.print(F(", DAC: "));
                Serial.print(new_dac_value);
                Serial.print(F(", timer: "));
                Serial.print(timer_pulses);
                Serial.print(F(", shaft: "));
                Serial.print(shaft_pulses);
                Serial.print(F(", Enable: "));
                Serial.print(digitalRead(ENABLE_PIN));
                Serial.print(F(", Running: "));
                Serial.println(projector_state);

                /* Add this if PID-Tuning Pots are connected
            Serial.print(" - P ");
            Serial.print(p_pot);
            Serial.print("  I ");
            Serial.print(i_pot);
            Serial.print("  D ");
            Serial.println(d_pot);
            */
            }
        }

            last_pulse_difference = current_pulse_difference; // Update the last_pulse_differencees);
            last_dac_value = new_dac_value;
        // }

        // Stop detection
        if (shaft_pulse_count_updated) // This is set in the shaft ISR
            last_pulse_timestamp = micros();
 
         if (hasStoppedSince(last_pulse_timestamp, STOP_THRESHOLD))
        {
            projector_state = PROJ_IDLE; // Projector is stopped
            Serial.println(F("[DEBUG] Projector stopped."));
            stopTimer1();
            // Todo: This might be obsolete, wince the ISR would just reset them. Do we need these flags at all?
            timer_frame_count_updated = false; // just in case the ISR fired again AND the shaft was still breaking. This could cause false PID computations.
            timer_pulse_count_updated = false; // just in case the ISR fired again AND the shaft was still breaking. This could cause false PID computations.
            shaft_pulses = 0;
            shaft_frames = 0;
            timer_pulses = 0;
            timer_frames = 0;

            current_pulse_difference = 0;
            local_timer_pulses = 0;
            local_shaft_pulses = 0;

            // Reset DAC and PID
            dac.setVoltage(speed.dac_init, false); // reset the DAc to compensate for wound-up break corrections
            myPID.SetMode(MANUAL);
            pid_output = speed.dac_init;
            pid_input = 0;
            myPID.Compute();
            Serial.print(F("PID Reset to initial DAC value: "));
            Serial.println(pid_output);
            myPID.SetMode(AUTOMATIC);
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(LED_RED_PIN, LOW);

            FreqMeasure.end();
        }
        else 
        // reset the flag and be ready for another incoming pulse
        {
            shaft_pulse_count_updated = false;
        }
    }
}

void measureFrequency()
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
    // Serial.print(micros());
    // Serial.print(" - ");
    // Serial.print(start);
    // Serial.print(" = ");
    // Serial.println(micros() - start);

    return (micros() - start) > duration;
}

void checkProjectorRunningYet()
{
    if (!new_shaft_impulse_available)
        return; // Skip processing if no new data

    new_shaft_impulse_available = false; // Reset the ISR's "new data available" flag

    projector_start_millis = millis(); // Track the start time of the projector running

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
            activateSpeedConfig(detected_frequency <= 21 * SHAFT_SEGMENT_COUNT ? FPS_18 : FPS_24);
            projector_speed_switch = (detected_frequency <= 21 * SHAFT_SEGMENT_COUNT ? 18 : 24);
        }
        // Init the pulse timer to not lose those first frames
        timer_pulses = freq_count;

        // Update projector state
        projector_state = PROJ_RUNNING;
        Serial.print(F("Setting up Timer for "));
        Serial.println(speed.name);
        setupTimer1forFps(currently_selected_mode);
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
    Serial.print(F("New Mode: "));
    Serial.println(currently_selected_mode);

    if (false)
    {
        // read prev (next?) speed.end_freq
        // add the error to the timerpulses
        // calculate timer_correction_factor for currently_selected_mode to next_speed_mode

        /* uint32_t updated_timer_pulses = local_timer_pulses * next_freq / ((local_shaft_pulses / SHAFT_SEGMENT_COUNT) / ((millis() - projector_start_millis) / 1000)); */
        
        // timer_correction_factor = new-speed / prev-speed, also zB 24 / 18 oder 18 / 16.666666
        // memcpy_P(&previous_avg_freq, &s_speed_table[(currently_selected_mode - change + MODES_COUNT) % MODES_COUNT].end_freq, sizeof(float));

        // uint8_t new_index = currently_selected_mode;
        // uint8_t before_index = previously_selected_mode;

        // Serial.print(F("new-index: "));
        // Serial.print(new_index);
        // Serial.print(F(", before-index: "));
        // Serial.println(before_index);

        // memcpy_P(&previous_avg_freq, &s_speed_table[(previously_selected_mode - change + MODES_COUNT) % MODES_COUNT].end_freq, sizeof(float));
    } 

    if (currently_selected_mode == FPS_AUTO)
    {
        Serial.println(F("Im Auto Switch"));
        switch (projector_speed_switch)
        {
        case 0:
            // switch pos is still unknown for this run
            // measure the current freq
            // set projector_speed_switch to 18 or 24

            break;
        case 18:
            Serial.println(F("Restoring 18"));
            activateSpeedConfig(FPS_18); // activates the speed struct, inits PID and DAC
            break;
        case 24:
            Serial.println(F("Restoring 24"));
            activateSpeedConfig(FPS_24);
            break;
        default:
            Serial.println(F("invalid value"));
        }
    } 
    else 
    {
        activateSpeedConfig(currently_selected_mode); 
    }

    // write the timerpulses (+timer_correction_factor)
    noInterrupts();
    // timer_pulses = timer_pulses * timer_correction_factor;
    timer_frame_count_updated = true;
    interrupts();

    /* ------------ old code below -----------------------------------------------
    // Prepare for re-calculating the ISR's timer_frames in sepmag mode
    float next_freq;

    noInterrupts();
    uint32_t local_shaft_pulses = shaft_pulses;
    uint32_t local_timer_pulses = timer_pulses;
    interrupts();

    // Debug Code
    // Serial.print(F("Average fps since start: "));
    // Serial.print((float)local_shaft_pulses / SHAFT_SEGMENT_COUNT);
    // Serial.print(F(" / "));
    // Serial.print((float)(millis() - projector_start_millis) / 1000);
    // Serial.print(F(" = "));
    // Serial.println(((float)local_shaft_pulses / SHAFT_SEGMENT_COUNT) / (((float)millis() - projector_start_millis) / 1000), 2);

    // Put an IF instead of the SWITCH here to do the auto measurement

    switch (currently_selected_mode) // This is the new currently_selected_mode we are about to switch TO

    {
        // we pick the right next_freq here.
        case FPS_NONE:
        break;
        case FPS_AUTO:
            // Since
            Serial.print(F("Estimated fps so far (for previous_freq):"));
            Serial.println((float)(millis() - projector_start_millis) / local_shaft_pulses / SHAFT_SEGMENT_COUNT, 2);
            Serial.print(F("Proj. Speed Switch (to): "));
            Serial.println(projector_speed_switch);
            break;
        case FPS_16_2_3:
        case FPS_18:
        case FPS_23_976:
        case FPS_24:
        case FPS_25:
            // copy the endFreq of the struct wuth currently_selected_mode index - 2 (0 and 1 are NONE and AUTO)
            memcpy_P(&next_freq, &s_speed_table[((currently_selected_mode))].end_freq, sizeof(float));
            Serial.println(next_freq);
            break;
    }

    if (projector_state == PROJ_RUNNING)  // This is "SEMAPG Mode" where we try to catch up to the new sync point
    {
        // Update the timer counter to the new truth
        // Example: After 10 Sec from 18 to 24 fps: -> 180 / 18 * 24 = 240
        //                                           = 180 * 24 / 18
        //                                           so

        // Debug Code
        // Serial.println(F("timer_pulses * next_freq / (millis() - proojector_start_millis) / 1000))"));
        // Serial.print(F("From "));
        // Serial.print(local_timer_pulses);
        // Serial.print(F(" Pulses to "));
        // Serial.print(local_timer_pulses);
        // Serial.print(F(" * "));
        // Serial.print(next_freq);
        // Serial.print(F(" / ("));
        // Serial.print(local_shaft_pulses);
        // Serial.print(F("/"));
        // Serial.print(F("12) / ("));
        // Serial.print(millis() - projector_start_millis);
        // Serial.print(F(")/1000) = "));
        // Serial.println(local_timer_pulses * next_freq / ((local_shaft_pulses / SHAFT_SEGMENT_COUNT) / ((millis() - projector_start_millis) / 1000)));



        uint32_t updated_timer_pulses = local_timer_pulses * next_freq / ((local_shaft_pulses / SHAFT_SEGMENT_COUNT) / ((millis() - projector_start_millis) / 1000));

        // This should be a bit more precise, but actually leads to results that apepar rather off. Not sure yet why
        // uint32_t fuckdated_timer_pulses = (local_timer_pulses * next_freq * (millis() - projector_start_millis)) / (local_shaft_pulses * 1000 / SHAFT_SEGMENT_COUNT);

        Serial.print(F("My calculation: "));
        Serial.println(updated_timer_pulses);

        noInterrupts();
        timer_pulses = updated_timer_pulses;
        timer_frame_count_updated = true;
        interrupts();
    }

    activateSpeedConfig(currently_selected_mode);
 ------------ old code above ----------------------------------------------- */
}

void activateSpeedConfig(byte next_speed)
    {
        // Copy next config from PROGMEM to struct
        memcpy_P(&speed, &s_speed_table[next_speed], sizeof(SpeedConfig));

        // To keep the sync Sepmag-style, we need to correct the timer to teh new speed
        float timer_correction_factor;
        if (projector_state == PROJ_RUNNING)
        {
            // work with ints as long as possible
            uint32_t now = millis();
            unsigned long elapsed_time_ms = now - projector_start_millis;
            unsigned long frames_per_1000_ms = (unsigned long)shaft_frames * 1000; // inflate shaft_frames to improve acuracy
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

        // old switch case
        // switch (next_speed)
        // {
        // case FPS_NONE:
        //     break;
        // case FPS_AUTO:
        //     // reset the PID Output
        //     myPID.SetMode(MANUAL);
        //     pid_output = speed.dac_init;
        //     myPID.Compute();
        //     myPID.SetMode(AUTOMATIC);
        //     // set DAC to initial value
        //     dac.setVoltage(speed.dac_init, false);
        //     Serial.println(F("XTAL_AUTO"));
        //     break;
        // case FPS_16_2_3:
        //     Serial.println(F("XTAL_16_2_3"));
        //     setupTimer1forFps(FPS_16_2_3);
        //     break;
        // case FPS_18:
        //     Serial.println(F("XTAL_18"));
        //     setupTimer1forFps(FPS_18);
        //     break;
        // case FPS_23_976:
        //     Serial.println(F("XTAL_23_976"));
        //     setupTimer1forFps(FPS_23_976);
        //     break;
        // case FPS_24:
        //     Serial.println(F("XTAL_24"));
        //     setupTimer1forFps(FPS_24);
        //     break;
        // case FPS_25:
        //     Serial.println(F("XTAL_25"));
        //     setupTimer1forFps(FPS_25);
        //     break;
        // default:
        //     Serial.println(F("Unknown Mode"));
        //     break;
        // }
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

bool setupTimer1forFps(byte desiredFps)
// !!!!  Wird nur noch in der speed-detection in checkPorjjectorRunningYet() aufgerufen !!!


{
    // Read the required speed config from PROGMEM, this saves RAM
    // loadSpeedConfig(desiredFps);

    // start with a new sync point, no need to catch up differences from before 
    if (projector_state == PROJ_IDLE)
    {
        noInterrupts();
        timer_frames = 0;
        // shaft_frames = 0;
        // timer_modulus = 0; // This sees to clear the modulus inbetween ISR calls, whyever?
        shaft_modulus = 0; 
        interrupts();
    }

    // DitherConfig cfg;
    // memcpy_P(&cfg, &s_ditherTable[desiredFps - 2], sizeof(DitherConfig));

    dither_accumulator_32 = 0;

    // Setup Timer1
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    TCNT1 = 0;
    OCR1A = speed.dither_base;              // OCR1A start value
    TCCR1B |= (1 << WGM12) | (1 << CS11);   // CTC-Mode (WGM12=1), Prescaler=8 => CS11=1
    TIMSK1 |= (1 << OCIE1A);                // enable Compare-A-Interrupt
    interrupts();

    Serial.print(F("*** Timer 1 set up for speed "));
    Serial.println(speed.end_freq);

    return true;
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
        shaft_frame_count_updated = true;
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
        timer_pulse_count_updated = true;
    }

    // Dither logic (fixed-point accumulator)
    dither_accumulator_32 += speed.dither_frac32;

    // If overflow => dither_accumulator_32 < ditherFrac32
    OCR1A = (dither_accumulator_32 < speed.dither_frac32) ? (speed.dither_base + 1) : speed.dither_base;
}

void stopTimer1()
{ // Stops Timer1, for when we are not in craystal running next_speed
    // TCCR1B &= ~(1 << CS11);
    noInterrupts();
    TIMSK1 &= ~(1 << OCIE1A);
    interrupts();
}

